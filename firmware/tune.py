#!/usr/bin/env python3
"""
Motor FOC Tuning GUI — web-based dashboard for SimpleFOC Commander tuning.

Connects to the motor controller serial port and serves a local web GUI with:
- PID gain controls for current, velocity, and position loops
- Step response testing with real-time plotting
- Live serial console
- Motor state monitoring

Usage: python3 tune.py [--port /dev/ttyACMx] [--baud 115200]
"""

import argparse
import glob
import http.server
import json
import os
import queue
import socketserver
import sys
import threading
import time
import webbrowser
from collections import deque

import serial

# ---------------------------------------------------------------------------
# Step response analysis
# ---------------------------------------------------------------------------

def analyze_step(step_data, target):
    """Compute metrics from step response data.

    The firmware records a 100ms baseline at target=0 then steps to the
    target value.  This function finds the step edge in column 1 (target)
    and computes metrics only from the post-step region.

    Returns dict with overshoot_pct, rise_time_ms, settle_time_ms,
    ss_error_pct, oscillating.
    """
    header = step_data.get("header", [])
    data = step_data.get("data", [])
    if not data or len(header) < 3:
        return {
            "overshoot_pct": 0, "rise_time_ms": 0, "settle_time_ms": 0,
            "ss_error_pct": 100, "oscillating": False,
        }

    # Find the step edge: first row where target column (col 1) is nonzero.
    # This skips the pre-step baseline so metrics reflect the actual response.
    step_idx = 0
    for i, row in enumerate(data):
        if abs(row[1]) > abs(target) * 0.5:
            step_idx = i
            break

    post = data[step_idx:]
    if len(post) < 5:
        return {
            "overshoot_pct": 0, "rise_time_ms": 0, "settle_time_ms": 0,
            "ss_error_pct": 100, "oscillating": False,
        }

    t_step = post[0][0]  # time of step edge
    times = [r[0] - t_step for r in post]  # zero-referenced to step
    actual = [r[2] for r in post]
    abs_target = abs(target) if target != 0 else 1.0

    # Overshoot: peak beyond target (in the direction of the step)
    if target >= 0:
        peak = max(actual)
    else:
        peak = min(actual)
    overshoot_pct = ((peak - target) / target * 100) if target != 0 else 0.0

    # Rise time: time from step to first reaching 80% of target
    threshold_80 = target * 0.8
    rise_time_ms = times[-1]
    for i, v in enumerate(actual):
        if target > 0 and v >= threshold_80:
            rise_time_ms = times[i]
            break
        elif target < 0 and v <= threshold_80:
            rise_time_ms = times[i]
            break

    # Settle time: last time outside 5% band around target
    band = abs_target * 0.05
    settle_time_ms = times[-1]
    for i in range(len(post) - 1, -1, -1):
        if abs(actual[i] - target) > band:
            settle_time_ms = times[min(i + 1, len(times) - 1)]
            break
    else:
        settle_time_ms = times[0]

    # Steady-state error: mean of last 20% vs target
    tail_start = int(len(actual) * 0.8)
    tail = actual[tail_start:]
    ss_avg = sum(tail) / len(tail) if tail else 0
    ss_error_pct = abs((ss_avg - target) / abs_target * 100) if abs_target else 0

    # Oscillating: count sign changes of error after first reaching 63%
    threshold_63 = target * 0.63
    start_idx = 0
    for i, v in enumerate(actual):
        if target > 0 and v >= threshold_63:
            start_idx = i
            break
        elif target < 0 and v <= threshold_63:
            start_idx = i
            break
        elif target == 0:
            start_idx = 0
            break

    sign_changes = 0
    for i in range(start_idx + 1, len(actual)):
        diff_prev = actual[i - 1] - target
        diff_curr = actual[i] - target
        if diff_prev * diff_curr < 0:
            sign_changes += 1

    oscillating = sign_changes > 4

    return {
        "overshoot_pct": round(overshoot_pct, 2),
        "rise_time_ms": round(rise_time_ms, 2),
        "settle_time_ms": round(settle_time_ms, 2),
        "ss_error_pct": round(ss_error_pct, 2),
        "oscillating": oscillating,
    }


# ---------------------------------------------------------------------------
# AutoTuner — automated PID tuning via step response analysis
# ---------------------------------------------------------------------------

class AutoTuner:
    def __init__(self, serial_mgr, sse_broadcast_fn):
        self.serial_mgr = serial_mgr
        self._emit_fn = sse_broadcast_fn
        self._thread = None
        self._abort = False
        self.running = False
        self.final_gains = None

    def start(self, loops=None):
        if loops is None:
            loops = ["current", "velocity"]
        if self.running:
            return
        self._abort = False
        self.running = True
        self.final_gains = None
        self._thread = threading.Thread(
            target=self._run, args=(loops,), daemon=True
        )
        self._thread.start()

    def stop(self):
        self._abort = True

    def _emit(self, payload):
        """Send an autotune SSE event."""
        self._emit_fn("autotune", json.dumps(payload))

    def _check_abort(self):
        if self._abort:
            raise _AutoTuneAborted()

    def _set_and_test(self, param_cmd, value, step_cmd, timeout=5.0):
        """Set a parameter, wait briefly, run step test, return analysis and raw data."""
        self.serial_mgr.send(f"{param_cmd}{value}")
        time.sleep(0.1)
        step_data = self.serial_mgr.run_step_test(step_cmd, timeout=timeout)
        # Parse target from step command (e.g. "Sq0.3" -> 0.3)
        # Format is always S<mode_char><number>
        target_str = step_cmd[2:]  # skip 'S' and mode character
        try:
            target = float(target_str)
        except ValueError:
            target = 0
        metrics = analyze_step(step_data, target)
        return metrics, step_data

    def _settle(self):
        """Stop motor and wait for settle."""
        self.serial_mgr.send("T0")
        time.sleep(0.5)

    @staticmethod
    def _flip_if_reversed(step_data):
        """If encoder direction is reversed (actual goes opposite to target),
        negate the actual column so the plot shows aligned traces."""
        data = step_data.get("data", [])
        header = step_data.get("header", [])
        if not data or len(header) < 3:
            return step_data
        # Use mean of last 20% of both target (col 1) and actual (col 2)
        tail_start = int(len(data) * 0.8)
        tail = data[tail_start:]
        if not tail:
            return step_data
        target_mean = sum(r[1] for r in tail) / len(tail)
        actual_mean = sum(r[2] for r in tail) / len(tail)
        # If they have opposite signs and target is nonzero, flip actual
        if target_mean != 0 and (target_mean * actual_mean < 0):
            flipped = []
            for row in data:
                new_row = list(row)
                new_row[2] = -new_row[2]
                flipped.append(new_row)
            return {"header": header, "data": flipped}
        return step_data

    @staticmethod
    def _estimate_rl(step_data, p_gain, target):
        """Estimate motor phase resistance R and inductance L from a P-only
        current step response (I=0).

        With P-only control on the R-L plant 1/(Ls+R):
          closed-loop DC gain G = P/(R+P)  →  R = P·(1-G)/G
          closed-loop time constant τ = L/(R+P)  →  L = τ·(R+P)
        """
        data = step_data.get("data", [])
        if not data or len(data) < 10:
            return None, None

        # Skip pre-step baseline: find where target column goes nonzero
        step_idx = 0
        for i, row in enumerate(data):
            if abs(row[1]) > abs(target) * 0.5:
                step_idx = i
                break
        post = data[step_idx:]
        if len(post) < 10:
            return None, None

        t_step = post[0][0]
        actual = [r[2] for r in post]

        # Flip if encoder reads reversed
        tail_start = int(len(post) * 0.8)
        tail_actual = actual[tail_start:]
        ss_actual = sum(tail_actual) / len(tail_actual)
        if target != 0 and (ss_actual * target < 0):
            actual = [-v for v in actual]
            ss_actual = -ss_actual

        if abs(target) < 1e-6 or abs(ss_actual) < 1e-6:
            return None, None

        # DC gain → R
        G = ss_actual / target
        if G <= 0.01 or G >= 0.99:
            return None, None
        R = p_gain * (1.0 - G) / G

        # 63% rise time → L (time referenced from step edge)
        threshold_63 = ss_actual * 0.63
        times = [r[0] - t_step for r in post]
        tau_ms = None
        for i, v in enumerate(actual):
            if target > 0 and v >= threshold_63:
                tau_ms = times[i]
                break
            elif target < 0 and v <= threshold_63:
                tau_ms = times[i]
                break

        if tau_ms is not None and tau_ms > 0:
            tau_s = tau_ms / 1000.0
            L = tau_s * (R + p_gain)
        else:
            # Can't resolve rise time (faster than sample rate) — estimate
            # conservatively assuming L/R = 0.5ms (typical small BLDC)
            L = R * 0.0005

        return R, L

    @staticmethod
    def _estimate_oscillation_period(step_data, target):
        """Estimate oscillation period (ms) from zero-crossings of error
        signal after the initial rise."""
        data = step_data.get("data", [])
        if not data or len(data) < 20:
            return 0

        # Skip pre-step baseline
        step_idx = 0
        for i, row in enumerate(data):
            if abs(row[1]) > abs(target) * 0.5:
                step_idx = i
                break
        post = data[step_idx:]
        if len(post) < 20:
            return 0

        t_step = post[0][0]
        times = [r[0] - t_step for r in post]
        actual = [r[2] for r in post]

        # Find where signal first reaches 50% of target
        threshold = target * 0.5
        start_idx = 0
        for i, v in enumerate(actual):
            if (target > 0 and v >= threshold) or (target < 0 and v <= threshold):
                start_idx = i
                break

        # Find zero-crossings of (actual - target)
        crossings = []
        for i in range(start_idx + 1, len(actual)):
            err_prev = actual[i - 1] - target
            err_curr = actual[i] - target
            if err_prev * err_curr < 0:
                # Linear interpolation for crossing time
                frac = abs(err_prev) / (abs(err_prev) + abs(err_curr))
                t = times[i - 1] + (times[i] - times[i - 1]) * frac
                crossings.append(t)

        if len(crossings) < 2:
            return 0

        # Full period = 2 × average half-period
        half_periods = [crossings[i + 1] - crossings[i]
                        for i in range(len(crossings) - 1)]
        avg_half = sum(half_periods) / len(half_periods)
        return round(2.0 * avg_half, 2)

    def _run(self, loops):
        gains = {}
        try:
            if "current" in loops:
                gains["current"] = self._tune_current_loop()
            if "velocity" in loops:
                gains["velocity"] = self._tune_velocity_loop()

            self.final_gains = gains
            self._emit({"type": "done", "gains": gains})

        except _AutoTuneAborted:
            self._emit({"type": "error", "msg": "Auto-tune stopped by user"})
        except Exception as e:
            self._emit({"type": "error", "msg": str(e)})
        finally:
            self.serial_mgr.send("T0")
            self.running = False

    # ------------------------------------------------------------------
    # Current loop: analytical pole-zero cancellation from estimated R, L
    # ------------------------------------------------------------------

    def _tune_current_loop(self):
        self._emit({"type": "status", "msg": "Estimating motor R and L",
                     "phase": "current_p"})

        STEP_CMD = "Sq0.3"
        TARGET = 0.3
        tf = 0.005

        # Set torque mode, P-only (I=0) for plant characterisation
        self.serial_mgr.send("MC0")
        time.sleep(0.2)
        self.serial_mgr.send("MQI0")
        time.sleep(0.1)

        # Run a P-only step to measure plant DC gain and time constant
        probe_p = 1.0
        self._settle()
        self._check_abort()
        metrics, step_data = self._set_and_test("MQP", probe_p, STEP_CMD)
        self._settle()

        self._emit({"type": "trial", "iteration": 1,
                     "params": {"P": probe_p, "I": 0},
                     "metrics": metrics, "plot": step_data})

        R, L = self._estimate_rl(step_data, probe_p, TARGET)

        if R is None:
            # Couldn't estimate — motor may not be responding.  Abort.
            raise RuntimeError(
                "Could not estimate motor R/L from step response. "
                "Check motor alignment and current sense.")

        self._emit({"type": "status",
                     "msg": f"Estimated R={R:.4f} Ω, L={L*1e3:.3f} mH",
                     "phase": "current_p"})

        # Compute analytical gains via pole-zero cancellation.
        # Start conservatively — 200 rad/s is reasonable for small BLDC
        # motors on low-voltage supplies (4V).  For reference, the user's
        # hand-tuned defaults (P=1, I=20) correspond to ~80-100 rad/s.
        bandwidth = 200.0  # rad/s

        for iteration in range(2, 7):  # up to 5 validation attempts
            self._check_abort()
            Kp = bandwidth * L
            Ki = bandwidth * R
            # Clamp to sane ranges for SimpleFOC current PID
            Kp = max(0.05, min(Kp, 10.0))
            Ki = max(0.1, min(Ki, 100.0))

            self._emit({"type": "status",
                         "msg": f"Trying BW={bandwidth:.0f} rad/s → P={Kp:.4f} I={Ki:.4f}",
                         "phase": "current_i"})

            self.serial_mgr.send(f"MQP{round(Kp, 5)}")
            time.sleep(0.05)
            self.serial_mgr.send(f"MQI{round(Ki, 5)}")
            time.sleep(0.05)

            self._settle()
            metrics, step_data = self._set_and_test("MQP", round(Kp, 5), STEP_CMD)
            self._settle()

            self._emit({"type": "trial", "iteration": iteration,
                         "params": {"P": round(Kp, 5), "I": round(Ki, 5)},
                         "metrics": metrics, "plot": step_data})

            if metrics["oscillating"] or metrics["overshoot_pct"] > 20:
                bandwidth *= 0.5  # too aggressive — halve bandwidth
                self._emit({"type": "status",
                             "msg": f"Overshoot {metrics['overshoot_pct']:.1f}%, reducing BW",
                             "phase": "current_i"})
            elif metrics["overshoot_pct"] < 1 and metrics["rise_time_ms"] > 50:
                bandwidth *= 1.5  # too sluggish — increase bandwidth gently
                self._emit({"type": "status",
                             "msg": f"Sluggish (rise {metrics['rise_time_ms']:.0f}ms), increasing BW",
                             "phase": "current_i"})
            else:
                break  # acceptable response

        p = round(Kp, 5)
        i_val = round(Ki, 5)

        # Apply same gains to Id axis
        self.serial_mgr.send(f"MDP{p}")
        time.sleep(0.05)
        self.serial_mgr.send(f"MDI{i_val}")
        time.sleep(0.05)
        self.serial_mgr.send(f"MDF{tf}")
        time.sleep(0.05)

        self._emit({"type": "status",
                     "msg": f"Current loop done: P={p} I={i_val} Tf={tf} "
                            f"(R={R:.3f}Ω L={L*1e3:.2f}mH BW={bandwidth:.0f}rad/s)",
                     "phase": "current_i"})

        # Final validation from rest
        self._settle()
        self._check_abort()
        self._emit({"type": "status", "msg": "Running final current validation step",
                     "phase": "current_i"})
        _, final_data = self._set_and_test("MQP", p, STEP_CMD)
        final_data = self._flip_if_reversed(final_data)
        final_metrics = analyze_step(final_data, TARGET)
        self._settle()
        self._emit({"type": "final_plot", "loop": "current", "plot": final_data,
                     "metrics": final_metrics,
                     "gains": {"P": p, "I": i_val, "Tf": tf}})

        return {"P": p, "I": i_val, "Tf": tf}

    # ------------------------------------------------------------------
    # Velocity loop: Ziegler-Nichols ultimate gain method
    # ------------------------------------------------------------------

    def _tune_velocity_loop(self):
        self._emit({"type": "status",
                     "msg": "Finding ultimate gain (Ku) for velocity loop",
                     "phase": "velocity_p"})

        STEP_CMD = "Sv5"   # 5 rad/s — moderate speed to reduce cogging effects
        TARGET = 5.0
        tf = 0.01

        # Set velocity mode, P-only
        self.serial_mgr.send("MC1")
        time.sleep(0.2)
        self.serial_mgr.send("MVI0")
        time.sleep(0.1)
        self.serial_mgr.send("MVD0")
        time.sleep(0.1)

        # Ramp P gain upward until oscillation or excessive overshoot.
        # This finds the Ziegler-Nichols ultimate gain Ku and period Tu.
        p = 0.05
        Ku = p
        Tu = 0
        iteration = 0

        while p <= 10.0:
            iteration += 1
            self._check_abort()

            self._emit({"type": "status",
                         "msg": f"Trying P={p:.4f} (iter {iteration})",
                         "phase": "velocity_p"})

            self._settle()
            metrics, step_data = self._set_and_test("MVP", round(p, 5), STEP_CMD)
            self._settle()

            self._emit({"type": "trial", "iteration": iteration,
                         "params": {"P": round(p, 5)},
                         "metrics": metrics, "plot": step_data})

            if metrics["oscillating"]:
                Ku = p
                Tu = self._estimate_oscillation_period(step_data, TARGET)
                self._emit({"type": "status",
                             "msg": f"Oscillation at P={p:.4f}, Tu={Tu:.1f}ms",
                             "phase": "velocity_p"})
                break

            if metrics["overshoot_pct"] > 40:
                # Near instability — use this as Ku estimate
                Ku = p
                Tu = self._estimate_oscillation_period(step_data, TARGET)
                self._emit({"type": "status",
                             "msg": f"High overshoot ({metrics['overshoot_pct']:.0f}%) at "
                                    f"P={p:.4f}, using as Ku, Tu={Tu:.1f}ms",
                             "phase": "velocity_p"})
                break

            Ku = p  # last stable P
            p *= 1.5  # 50% increments — fine enough to not jump past Ku

        # Apply Ziegler-Nichols PI formulas with conservative derating.
        # Standard Z-N: Kp = 0.45·Ku, Ki = 0.54·Ku/Tu
        # Derated for less aggressive tuning (better phase margin):
        if Tu > 0:
            Tu_s = Tu / 1000.0
            Kp = round(0.35 * Ku, 5)
            Ki = round(0.40 * Ku / Tu_s, 5)
            self._emit({"type": "status",
                         "msg": f"Z-N: Ku={Ku:.4f} Tu={Tu:.1f}ms → P={Kp} I={Ki}",
                         "phase": "velocity_i"})
        else:
            # Couldn't determine Tu (never oscillated) — use Ku with a
            # conservative ratio.  Without Tu we can't use Z-N properly,
            # so fall back to a heuristic: I ≈ 2·P.
            Kp = round(0.5 * Ku, 5)
            Ki = round(Kp * 2.0, 5)
            self._emit({"type": "status",
                         "msg": f"No oscillation found up to P={Ku:.4f}. "
                                f"Using P={Kp} I={Ki} (heuristic)",
                         "phase": "velocity_i"})

        Kp = max(0.01, min(Kp, 10.0))
        Ki = max(0.01, min(Ki, 200.0))

        # Validate and refine
        self.serial_mgr.send(f"MVP{Kp}")
        time.sleep(0.05)
        self.serial_mgr.send(f"MVI{Ki}")
        time.sleep(0.05)

        for refine_iter in range(1, 4):
            self._check_abort()
            iteration += 1

            self._settle()
            metrics, step_data = self._set_and_test("MVP", Kp, STEP_CMD)
            self._settle()

            self._emit({"type": "trial", "iteration": iteration,
                         "params": {"P": Kp, "I": Ki},
                         "metrics": metrics, "plot": step_data})

            if metrics["oscillating"] or metrics["overshoot_pct"] > 30:
                Kp = round(Kp * 0.7, 5)
                Ki = round(Ki * 0.7, 5)
                self.serial_mgr.send(f"MVP{Kp}")
                time.sleep(0.05)
                self.serial_mgr.send(f"MVI{Ki}")
                time.sleep(0.05)
                self._emit({"type": "status",
                             "msg": f"Reducing gains: P={Kp} I={Ki}",
                             "phase": "velocity_i"})
            elif metrics["ss_error_pct"] > 8:
                Ki = round(Ki * 1.5, 5)
                self.serial_mgr.send(f"MVI{Ki}")
                time.sleep(0.05)
                self._emit({"type": "status",
                             "msg": f"SS error {metrics['ss_error_pct']:.1f}%, "
                                    f"increasing I to {Ki}",
                             "phase": "velocity_i"})
            else:
                break

        d = 0
        ramp = 200

        self._emit({"type": "status",
                     "msg": f"Velocity loop done: P={Kp} I={Ki} D={d} ramp={ramp} Tf={tf}",
                     "phase": "velocity_i"})

        # Final validation from rest
        self._settle()
        self._check_abort()
        self._emit({"type": "status", "msg": "Running final velocity validation step",
                     "phase": "velocity_i"})
        _, final_data = self._set_and_test("MVP", Kp, STEP_CMD)
        final_data = self._flip_if_reversed(final_data)
        final_metrics = analyze_step(final_data, TARGET)
        self._settle()
        self._emit({"type": "final_plot", "loop": "velocity", "plot": final_data,
                     "metrics": final_metrics,
                     "gains": {"P": Kp, "I": Ki, "D": d, "ramp": ramp, "Tf": tf}})

        return {"P": Kp, "I": Ki, "D": d, "ramp": ramp, "Tf": tf}


class _AutoTuneAborted(Exception):
    pass


# ---------------------------------------------------------------------------
# Serial manager — background thread that reads serial and distributes lines
# ---------------------------------------------------------------------------

class SerialManager:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.lock = threading.Lock()
        self.sse_clients = []  # list of queue.Queue
        self.sse_lock = threading.Lock()
        self.step_queue = None  # set during step test to capture lines
        self.step_lock = threading.Lock()
        self.running = True
        self.firmware_type = "simplefoc"  # default/fallback
        self._read_loop_started = False
        self.thread = threading.Thread(target=self._read_loop, daemon=True)

    def detect_firmware(self):
        """Send '?' and check response to identify firmware type.
        Must be called before start() so the read loop doesn't steal bytes."""
        # Drain any pending data
        time.sleep(0.1)
        while self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)
        self.ser.write(b"?\n")
        deadline = time.time() + 1.0
        while time.time() < deadline:
            try:
                line = self.ser.readline()
                if line:
                    text = line.decode("utf-8", errors="replace").strip()
                    if "FW:current_test" in text:
                        self.firmware_type = "current_test"
                        return
            except Exception:
                break
        self.firmware_type = "simplefoc"

    def start(self):
        """Start the background read loop. Call after detect_firmware()."""
        if not self._read_loop_started:
            self._read_loop_started = True
            self.thread.start()

    def _read_loop(self):
        while self.running:
            try:
                line = self.ser.readline()
                if not line:
                    continue
                text = line.decode("utf-8", errors="replace").rstrip("\r\n")
                if not text:
                    continue
                # Push to step capture if active
                with self.step_lock:
                    if self.step_queue is not None:
                        self.step_queue.put(text)
                # Always show in console
                self.broadcast_sse(None, text)
            except Exception:
                if not self.running:
                    break
                time.sleep(0.1)

    def broadcast_sse(self, event_name, data):
        """Send an SSE message to all connected clients.

        event_name: if not None, emits 'event: <name>' line.
        data: the data payload string.
        """
        msg = (event_name, data)
        with self.sse_lock:
            dead = []
            for i, q in enumerate(self.sse_clients):
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    dead.append(i)
            for i in reversed(dead):
                self.sse_clients.pop(i)

    def send(self, cmd):
        with self.lock:
            self.ser.write((cmd + "\n").encode())

    def add_sse_client(self):
        q = queue.Queue(maxsize=1000)
        with self.sse_lock:
            self.sse_clients.append(q)
        return q

    def remove_sse_client(self, q):
        with self.sse_lock:
            try:
                self.sse_clients.remove(q)
            except ValueError:
                pass

    def run_step_test(self, cmd, timeout=5.0):
        """Send step command, collect CSV lines until DONE, return parsed data."""
        q = queue.Queue(maxsize=50000)
        with self.step_lock:
            self.step_queue = q
        self.send(cmd)
        header = None
        rows = []
        error = None
        line_count = 0
        skipped = 0
        deadline = time.time() + timeout
        try:
            while time.time() < deadline:
                try:
                    line = q.get(timeout=0.1)
                except queue.Empty:
                    continue
                line_count += 1
                stripped = line.strip()
                if line_count <= 5:
                    print(f"[step] line {line_count}: {stripped[:120]}")
                if stripped == "DONE":
                    break
                if stripped.startswith("ERR:"):
                    error = stripped
                    continue
                parts = line.split(",")
                if header is None:
                    # First line with commas is the header
                    if len(parts) >= 3 and not parts[0].replace(".", "").replace("-", "").isdigit():
                        header = [p.strip() for p in parts]
                        print(f"[step] header: {header}")
                        continue
                if header and len(parts) == len(header):
                    try:
                        row = []
                        for p in parts:
                            try:
                                row.append(float(p))
                            except ValueError:
                                row.append(0.0)  # replace ovf/nan/inf with 0
                        rows.append(row)
                    except Exception:
                        skipped += 1
                else:
                    skipped += 1
        finally:
            with self.step_lock:
                self.step_queue = None
        print(f"[step] total lines={line_count} header_cols={len(header) if header else 0} rows={len(rows)} skipped={skipped}")
        return {"header": header or [], "data": rows, "error": error}

    def read_params(self):
        """Query all PID/LPF/limit params from firmware, return dict of id→value."""
        params = ["MQP", "MQI", "MQF", "MVP", "MVI", "MVD", "MVR", "MVF",
                  "MAP", "MAI", "MAD", "MAF", "MLU", "MLC", "MLV"]
        results = {}
        q = queue.Queue(maxsize=100)
        with self.step_lock:
            self.step_queue = q
        try:
            for param_id in params:
                # Drain any pending
                while not q.empty():
                    try: q.get_nowait()
                    except queue.Empty: break
                self.send(param_id)
                # Wait for response (single numeric line)
                deadline = time.time() + 0.5
                while time.time() < deadline:
                    try:
                        line = q.get(timeout=0.1)
                        stripped = line.strip()
                        # SimpleFOC responds with just the number
                        try:
                            val = float(stripped)
                            results[param_id] = val
                            break
                        except ValueError:
                            continue  # skip non-numeric lines
                    except queue.Empty:
                        continue
        finally:
            with self.step_lock:
                self.step_queue = None
        return results

    def close(self):
        self.running = False
        self.ser.close()


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Motor Tuning GUI</title>
<style>
:root {
    --bg: #0d1117; --surface: #161b22; --surface2: #21262d;
    --border: #30363d; --text: #e6edf3; --text2: #8b949e;
    --accent: #58a6ff; --green: #3fb950; --red: #f85149;
    --yellow: #d29922; --purple: #bc8cff; --orange: #f0883e;
    --mono: 'JetBrains Mono', 'Fira Code', 'Cascadia Code', monospace;
}
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
    background: var(--bg); color: var(--text); line-height: 1.5; font-size: 14px; }
button { cursor: pointer; font-family: inherit; font-size: 13px;
    background: var(--surface2); color: var(--text); border: 1px solid var(--border);
    border-radius: 4px; padding: 4px 12px; }
button:hover { background: var(--border); }
button.primary { background: #1f6feb; border-color: #388bfd; }
button.primary:hover { background: #388bfd; }
button.danger { background: #6e2b2b; border-color: var(--red); }
button.danger:hover { background: var(--red); }
input[type="number"], input[type="text"] {
    background: var(--surface); color: var(--text); border: 1px solid var(--border);
    border-radius: 4px; padding: 4px 8px; font-family: var(--mono); font-size: 13px;
    width: 80px; }
input[type="number"]:focus, input[type="text"]:focus { border-color: var(--accent); outline: none; }
select { background: var(--surface); color: var(--text); border: 1px solid var(--border);
    border-radius: 4px; padding: 4px 8px; font-size: 13px; }

/* Layout */
.top-bar { background: var(--surface); border-bottom: 1px solid var(--border);
    padding: 10px 20px; display: flex; align-items: center; gap: 16px; }
.top-bar h1 { font-size: 1.2em; white-space: nowrap; }
.top-bar h1 span { color: var(--accent); }
.status-dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; }
.status-dot.ok { background: var(--green); }
.status-dot.err { background: var(--red); }
.top-bar .port-info { color: var(--text2); font-family: var(--mono); font-size: 0.85em; }
.top-bar .spacer { flex: 1; }

.main { display: grid; grid-template-columns: 340px 1fr; grid-template-rows: 1fr 240px;
    height: calc(100vh - 49px); }
.sidebar { grid-row: 1 / 3; overflow-y: auto; border-right: 1px solid var(--border);
    padding: 12px; }
.plot-area { padding: 12px; overflow: hidden; display: flex; flex-direction: column; }
.console-area { grid-column: 2; border-top: 1px solid var(--border);
    display: flex; flex-direction: column; padding: 8px 12px; }

/* Sidebar sections */
.section { margin-bottom: 12px; }
.section-header { background: var(--surface2); border: 1px solid var(--border);
    border-radius: 6px 6px 0 0; padding: 8px 12px; font-weight: 600; font-size: 0.9em;
    cursor: pointer; user-select: none; display: flex; align-items: center; gap: 8px; }
.section-header::before { content: "\25BC"; font-size: 0.7em; color: var(--accent);
    transition: transform 0.15s; }
.section.collapsed .section-header::before { transform: rotate(-90deg); }
.section-body { background: var(--surface); border: 1px solid var(--border);
    border-top: none; border-radius: 0 0 6px 6px; padding: 10px; }
.section.collapsed .section-body { display: none; }

.param-row { display: flex; align-items: center; gap: 6px; margin-bottom: 6px; }
.param-label { width: 70px; font-size: 0.85em; color: var(--text2); flex-shrink: 0; }
.param-row input[type="number"] { flex: 1; min-width: 0; }
.param-row button { padding: 4px 8px; font-size: 12px; flex-shrink: 0; }

/* Step test */
.step-controls { display: flex; gap: 8px; align-items: center; margin-bottom: 10px; flex-wrap: wrap; }
.step-controls select { width: auto; }
.step-controls input { width: 70px; }
.plot-container { flex: 1; position: relative; min-height: 0; }
.plot-container canvas { width: 100%; height: 100%; }
.plot-metrics { display: flex; gap: 16px; padding: 6px 0; font-size: 0.85em; color: var(--text2); }
.plot-metrics span { font-family: var(--mono); }
.plot-metrics .val { color: var(--text); font-weight: 600; }

/* Console */
.console-output { flex: 1; overflow-y: auto; padding: 8px 16px; font-family: var(--mono);
    font-size: 12px; white-space: pre-wrap; color: var(--text2); background: var(--bg); }
.console-input-row { display: flex; border-top: 1px solid var(--border); }
.console-input-row input { flex: 1; border: none; border-radius: 0;
    padding: 8px 16px; font-family: var(--mono); width: auto; }
.console-input-row button { border-radius: 0; border: none; border-left: 1px solid var(--border); padding: 8px 16px; }

/* Plot tooltip */
.plot-tooltip { position: absolute; pointer-events: none; background: var(--surface2);
    border: 1px solid var(--border); border-radius: 4px; padding: 6px 10px;
    font-family: var(--mono); font-size: 11px; color: var(--text); z-index: 10;
    white-space: nowrap; line-height: 1.6; }
.plot-tooltip .tt-row { display: flex; align-items: center; gap: 6px; }
.plot-tooltip .tt-swatch { width: 8px; height: 8px; border-radius: 2px; flex-shrink: 0; }
.plot-container canvas { cursor: crosshair; }

/* Legend */
.legend { display: flex; flex-wrap: wrap; gap: 6px 14px; padding: 4px 0; font-size: 0.8em; align-items: center; }
.legend-item { display: flex; align-items: center; gap: 4px; cursor: pointer; user-select: none; padding: 2px 4px; border-radius: 3px; }
.legend-item:hover { background: var(--surface2); }
.legend-item.hidden { opacity: 0.35; text-decoration: line-through; }
.legend-swatch { width: 14px; height: 3px; border-radius: 1px; }
.legend-group { font-size: 0.75em; padding: 2px 6px; border-radius: 3px; border: 1px solid var(--border); background: var(--surface); color: var(--text2); cursor: pointer; user-select: none; }
.legend-group:hover { border-color: var(--accent); color: var(--text); }
.legend-group.active { border-color: var(--accent); background: var(--surface2); color: var(--accent); }

/* Auto-tune */
.autotune-btns { display: flex; flex-wrap: wrap; gap: 6px; margin-bottom: 8px; }
.autotune-btns button { flex: 1; min-width: 90px; font-size: 12px; padding: 6px 8px; }
.autotune-log { max-height: 200px; overflow-y: auto; font-family: var(--mono); font-size: 11px;
    background: var(--bg); border: 1px solid var(--border); border-radius: 4px;
    padding: 6px; color: var(--text2); white-space: pre-wrap; line-height: 1.4; }
.autotune-result { border: 1px solid var(--green); border-radius: 6px; padding: 8px;
    margin-top: 8px; background: rgba(63,185,80,0.06); font-family: var(--mono); font-size: 11px; }
.autotune-result h4 { color: var(--green); margin-bottom: 4px; font-size: 12px; }
.sidebar.tuning-active .section:not(#sec-autotune) .section-body button,
.sidebar.tuning-active .section:not(#sec-autotune) .section-body input {
    opacity: 0.4; pointer-events: none; }

/* Final result plot tabs */
.final-plot-tabs { display: flex; gap: 2px; padding-bottom: 6px; }
.final-plot-tab { padding: 4px 14px; font-size: 12px; border-radius: 4px 4px 0 0;
    cursor: pointer; background: var(--surface2); border: 1px solid var(--border);
    border-bottom: none; color: var(--text2); }
.final-plot-tab.active { background: var(--surface); color: var(--accent);
    border-color: var(--accent); border-bottom: 1px solid var(--surface); }
</style>
</head>
<body>

<div class="top-bar">
    <h1 id="topTitle"><span>FOC</span> Tuning</h1>
    <span class="status-dot ok" id="statusDot"></span>
    <span class="port-info" id="portInfo"></span>
    <span id="vmotDisplay" style="margin-left:12px;font-family:var(--mono);font-size:13px;color:var(--text2)">VMOT: --</span>
    <div class="spacer"></div>
    <button id="btnHwInit" onclick="sendCmd('H')">HW Init</button>
    <button id="btnAlign" onclick="doAlignBtn()">Align</button>
    <button id="btnReport" onclick="sendCmd('R')">Report</button>
    <button id="btnAdcDiag" onclick="sendCmd('D')">ADC Diag</button>
    <button id="btnWindingR" onclick="sendCmd('W')">Winding R</button>
    <button id="btnRead" onclick="sendCmd('R')" style="display:none">Read</button>
    <button class="danger" id="btnStop" onclick="sendStop()">Stop</button>
    <button onclick="doReconnect()">Reconnect</button>
</div>

<div class="main">
    <div class="sidebar" id="sidebar">
        <!-- Motor Config -->
        <div class="section" id="sec-motor-config">
            <div class="section-header" onclick="toggleSection('sec-motor-config')">Motor Config</div>
            <div class="section-body">
                <div class="param-row"><span class="param-label">Pole Pairs</span>
                    <input type="number" id="polePairs" min="1" max="50" step="1" value="11">
                    <button onclick="sendCmd('N'+document.getElementById('polePairs').value)">Set</button><button onclick="sendCmd('N')">?</button></div>
                <div class="param-row"><span class="param-label">Calibration</span>
                    <button onclick="doCalSave()" style="flex:1">Save</button>
                    <button onclick="doCalLoad()" style="flex:1">Load</button></div>
            </div>
        </div>

        <!-- Auto-tune -->
        <div class="section" id="sec-autotune">
            <div class="section-header" onclick="toggleSection('sec-autotune')">Auto-tune</div>
            <div class="section-body">
                <div class="autotune-btns" id="autotuneBtns">
                    <button class="primary" onclick="startAutotune(['current'])">Current</button>
                    <button class="primary" onclick="startAutotune(['velocity'])">Velocity</button>
                    <button class="primary" onclick="startAutotune(['current','velocity'])">All</button>
                    <button class="danger" id="autotuneStopBtn" onclick="stopAutotune()" style="display:none">Stop</button>
                </div>
                <div class="autotune-log" id="autotuneLog"></div>
                <div id="autotuneResult" style="display:none"></div>
            </div>
        </div>

        <!-- Current Loop -->
        <div class="section" id="sec-current">
            <div class="section-header" onclick="toggleSection('sec-current')">Current Loop (Iq/Id)</div>
            <div class="section-body">
                <div class="param-row"><span class="param-label">P</span>
                    <input type="number" id="MQP" step="0.01" value="0.6">
                    <button onclick="setCurrentParam('P')">Set</button><button onclick="readParam('MQP')">?</button></div>
                <div class="param-row"><span class="param-label">I</span>
                    <input type="number" id="MQI" step="0.1" value="0.3">
                    <button onclick="setCurrentParam('I')">Set</button><button onclick="readParam('MQI')">?</button></div>
                <div class="param-row"><span class="param-label">LPF Tf</span>
                    <input type="number" id="MQF" step="0.001" value="0.02">
                    <button onclick="setCurrentParam('F')">Set</button><button onclick="readParam('MQF')">?</button></div>
            </div>
        </div>

        <!-- Velocity Loop -->
        <div class="section" id="sec-velocity">
            <div class="section-header" onclick="toggleSection('sec-velocity')">Velocity Loop</div>
            <div class="section-body">
                <div class="param-row"><span class="param-label">P</span>
                    <input type="number" id="MVP" step="0.1" value="0.3">
                    <button onclick="setParam('MVP')">Set</button><button onclick="readParam('MVP')">?</button></div>
                <div class="param-row"><span class="param-label">I</span>
                    <input type="number" id="MVI" step="0.1" value="0.1">
                    <button onclick="setParam('MVI')">Set</button><button onclick="readParam('MVI')">?</button></div>
                <div class="param-row"><span class="param-label">D</span>
                    <input type="number" id="MVD" step="0.01" value="0">
                    <button onclick="setParam('MVD')">Set</button><button onclick="readParam('MVD')">?</button></div>
                <div class="param-row"><span class="param-label">Ramp</span>
                    <input type="number" id="MVR" step="10" value="200">
                    <button onclick="setParam('MVR')">Set</button><button onclick="readParam('MVR')">?</button></div>
                <div class="param-row"><span class="param-label">LPF Tf</span>
                    <input type="number" id="MVF" step="0.001" value="0.01">
                    <button onclick="setParam('MVF')">Set</button><button onclick="readParam('MVF')">?</button></div>
            </div>
        </div>

        <!-- Position Loop -->
        <div class="section collapsed" id="sec-position">
            <div class="section-header" onclick="toggleSection('sec-position')">Position Loop</div>
            <div class="section-body">
                <div class="param-row"><span class="param-label">P</span>
                    <input type="number" id="MAP" step="0.1" value="0.5">
                    <button onclick="setParam('MAP')">Set</button><button onclick="readParam('MAP')">?</button></div>
                <div class="param-row"><span class="param-label">I</span>
                    <input type="number" id="MAI" step="0.1" value="0">
                    <button onclick="setParam('MAI')">Set</button><button onclick="readParam('MAI')">?</button></div>
                <div class="param-row"><span class="param-label">D</span>
                    <input type="number" id="MAD" step="0.01" value="0">
                    <button onclick="setParam('MAD')">Set</button><button onclick="readParam('MAD')">?</button></div>
                <div class="param-row"><span class="param-label">LPF Tf</span>
                    <input type="number" id="MAF" step="0.001" value="0">
                    <button onclick="setParam('MAF')">Set</button><button onclick="readParam('MAF')">?</button></div>
            </div>
        </div>

        <!-- Limits -->
        <div class="section" id="sec-limits">
            <div class="section-header" onclick="toggleSection('sec-limits')">Limits</div>
            <div class="section-body">
                <div class="param-row"><span class="param-label">Voltage</span>
                    <input type="number" id="MLU" step="0.5" value="8.0">
                    <button onclick="setParam('MLU')">Set</button><button onclick="readParam('MLU')">?</button></div>
                <div class="param-row"><span class="param-label">Current</span>
                    <input type="number" id="MLC" step="0.1" value="3.9">
                    <button onclick="setParam('MLC')">Set</button><button onclick="readParam('MLC')">?</button></div>
                <div class="param-row"><span class="param-label">Velocity</span>
                    <input type="number" id="MLV" step="1" value="50">
                    <button onclick="setParam('MLV')">Set</button><button onclick="readParam('MLV')">?</button></div>
            </div>
        </div>

        <!-- Motor Control -->
        <div class="section" id="sec-mode">
            <div class="section-header" onclick="toggleSection('sec-mode')">Motor Control</div>
            <div class="section-body">
                <div class="param-row"><span class="param-label">Velocity (rad/s)</span>
                    <input type="number" id="motorSpeed" step="1" value="50" style="flex:1"></div>
                <div class="param-row" style="gap:4px">
                    <button class="primary" onclick="motorRunVel(1)" style="flex:1">Forward</button>
                    <button class="primary" onclick="motorRunVel(-1)" style="flex:1">Reverse</button>
                </div>
                <div class="param-row"><span class="param-label">Current (A)</span>
                    <input type="number" id="motorCurrent" step="0.1" value="0.5" style="flex:1"></div>
                <div class="param-row" style="gap:4px">
                    <button class="primary" onclick="motorRunCur(1)" style="flex:1">Forward</button>
                    <button class="primary" onclick="motorRunCur(-1)" style="flex:1">Reverse</button>
                </div>
                <div class="param-row" style="margin-top:4px;gap:4px">
                    <button class="danger" onclick="motorStop()" style="flex:1">Stop</button>
                    <button onclick="motorCoast()" style="flex:1">Coast</button>
                </div>
            </div>
        </div>

        <!-- Voltage Control (current_test mode only) -->
        <div class="section" id="sec-voltage" style="display:none">
            <div class="section-header" onclick="toggleSection('sec-voltage')">Voltage Control</div>
            <div class="section-body">
                <div class="param-row"><span class="param-label">Voltage</span>
                    <input type="number" id="ctVoltage" step="0.1" value="0.5">
                    <button onclick="sendCmd('V'+document.getElementById('ctVoltage').value); setPidEnabled(false)">Set</button></div>
                <div class="param-row">
                    <button onclick="sendCmd('0'); setPidEnabled(false)" style="flex:1">Zero</button>
                    <button onclick="sendCmd('R')" style="flex:1">Read</button>
                    <button class="primary" onclick="runSweep()" style="flex:1">Sweep</button>
                </div>
            </div>
        </div>

        <!-- Current Control (current_test mode only) -->
        <div class="section" id="sec-current-ctrl" style="display:none">
            <div class="section-header" onclick="toggleSection('sec-current-ctrl')">Current Control</div>
            <div class="section-body">
                <div class="param-row"><span class="param-label">P</span>
                    <input type="number" id="ctKp" step="0.1" value="0.1"></div>
                <div class="param-row"><span class="param-label">I</span>
                    <input type="number" id="ctKi" step="1" value="0"></div>
                <div class="param-row"><span class="param-label">D</span>
                    <input type="number" id="ctKd" step="0.01" value="0"></div>
                <div class="param-row">
                    <button id="pidToggleBtn" class="danger" onclick="togglePid()" style="flex:1">Disable</button>
                </div>
            </div>
        </div>

        <!-- ADC / Filter (current_test mode only) -->
        <div class="section" id="sec-adc-filter" style="display:none">
            <div class="section-header" onclick="toggleSection('sec-adc-filter')">ADC / Filter</div>
            <div class="section-body">
                <div class="param-row"><span class="param-label">Oversample</span>
                    <input type="number" id="ctOversample" min="1" max="16" step="1" value="4">
                    <button onclick="sendCmd('O'+document.getElementById('ctOversample').value)">Set</button></div>
                <div class="param-row"><span class="param-label">EMA &#945;</span>
                    <input type="number" id="ctEmaAlpha" min="0.01" max="1.0" step="0.05" value="0.3">
                    <button onclick="sendCmd('E'+document.getElementById('ctEmaAlpha').value)">Set</button></div>
            </div>
        </div>
    </div>

    <div class="plot-area">
        <div class="step-controls">
            <select id="stepMode" onchange="onStepModeChange()">
                <option value="q">Current (Iq)</option>
                <option value="i">Current Impulse (fixed angle)</option>
                <option value="v">Velocity</option>
                <option value="w">Velocity Sine</option>
                <option value="p">Position</option>
            </select>
            <input type="number" id="stepValue" step="0.1" value="0.5" placeholder="step">
            <input type="number" id="sinePeriod" step="100" value="1000" placeholder="period ms" title="Sine period (ms)" style="display:none;width:80px">
            <button class="primary" id="stepBtn" onclick="runStepTest()">Run Step Test</button>
            <button id="contBtn" onclick="toggleContinuous()">Continuous</button>
            <span id="stepStatus" style="color:var(--text2);font-size:0.85em"></span>
        </div>
        <div class="legend" id="plotLegend"></div>
        <div class="plot-container" id="livePlotContainer">
            <canvas id="plotCanvas"></canvas>
            <div class="plot-tooltip" id="plotTooltip" style="display:none"></div>
        </div>
        <div class="plot-metrics" id="plotMetrics"></div>
        <!-- Final result plots (shown after autotune) -->
        <div id="finalPlotArea" style="display:none;flex:1;min-height:0;flex-direction:column">
            <div class="final-plot-tabs" id="finalPlotTabs"></div>
            <div class="plot-container" style="flex:1;min-height:0">
                <canvas id="finalPlotCanvas"></canvas>
            </div>
            <div class="plot-metrics" id="finalPlotMetrics"></div>
            <div style="padding:4px 0">
                <button onclick="showLivePlot()" style="font-size:12px">Back to live plot</button>
            </div>
        </div>
    </div>

    <div class="console-area">
        <div class="console-output" id="consoleOut"></div>
        <div class="console-input-row">
            <input type="text" id="consoleIn" placeholder="Type command..." onkeydown="if(event.key==='Enter')sendConsole()">
            <button onclick="sendConsole()">Send</button>
            <button onclick="document.getElementById('consoleOut').textContent=''">Clear</button>
        </div>
    </div>
</div>

<script>
// --- State ---
let plotData = null;
let evtSource = null;
let firmwareType = 'simplefoc';
let pidEnabled = false;
let continuousMode = false;
let stepLabelEl = null;  // reference to step label element in current_test mode
let hiddenSeries = new Set(['raw0', 'raw1', 'raw2']);  // set of column names to hide from plot

function updateStepLabel() {
    if (stepLabelEl) {
        stepLabelEl.textContent = pidEnabled ? 'Current Step (A)' : 'Voltage Step (V)';
    }
}

function setPidEnabled(enabled) {
    pidEnabled = enabled;
    updateStepLabel();
    const btn = document.getElementById('pidToggleBtn');
    if (btn) {
        if (enabled) {
            btn.textContent = 'Disable';
            btn.className = 'danger';
        } else {
            btn.textContent = 'Enable';
            btn.className = 'primary';
        }
    }
}

function togglePid() {
    if (pidEnabled) {
        sendCmd('0');
        setPidEnabled(false);
    } else {
        setPidEnabled(true);
    }
}

// --- SSE connection ---
let consoleBuf = [];
let consoleRafPending = false;
function flushConsole() {
    consoleRafPending = false;
    if (!consoleBuf.length) return;
    const el = document.getElementById('consoleOut');
    el.textContent += consoleBuf.join('\n') + '\n';
    consoleBuf = [];
    if (el.scrollHeight - el.scrollTop - el.clientHeight < 100) {
        el.scrollTop = el.scrollHeight;
    }
    if (el.textContent.length > 100000) {
        el.textContent = el.textContent.slice(-50000);
    }
}

function connectSSE() {
    if (evtSource) evtSource.close();
    evtSource = new EventSource('/stream');
    evtSource.onmessage = function(e) {
        const isVmot = /^VMOT=/.test(e.data);
        if (isVmot) { parseVmot(e.data); return; }
        consoleBuf.push(e.data);
        if (!consoleRafPending) {
            consoleRafPending = true;
            requestAnimationFrame(flushConsole);
        }
    };
    evtSource.addEventListener('autotune', function(e) {
        try {
            const payload = JSON.parse(e.data);
            handleAutotuneEvent(payload);
        } catch(err) { console.error('autotune SSE parse error', err); }
    });
    evtSource.addEventListener('step_result', function(e) {
        try {
            handleStepResult(JSON.parse(e.data));
        } catch(err) { console.error('step_result SSE parse error', err); }
    });
    evtSource.addEventListener('align_result', function(e) {
        try {
            handleAlignResult(JSON.parse(e.data));
        } catch(err) { console.error('align_result SSE parse error', err); }
    });
    evtSource.onopen = function() {
        document.getElementById('statusDot').className = 'status-dot ok';
    };
    evtSource.onerror = function() {
        document.getElementById('statusDot').className = 'status-dot err';
        setTimeout(connectSSE, 2000);
    };
}

// --- Commands ---
async function sendCmd(cmd) {
    try {
        await fetch('/send', { method: 'POST', headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({cmd: cmd}) });
    } catch(e) { console.error(e); }
}

async function doAlignBtn() {
    const btn = document.getElementById('btnAlign');
    btn.disabled = true;
    btn.textContent = 'Aligning...';
    try {
        await fetch('/align', { method: 'POST' });
        // Result arrives via SSE align_result event -> handleAlignResult()
    } catch(e) {
        btn.disabled = false;
        btn.textContent = 'Align';
        alert('Align request failed: ' + e.message);
    }
}

function handleAlignResult(data) {
    const btn = document.getElementById('btnAlign');
    btn.disabled = false;
    btn.textContent = 'Align';
    const msg = data.msg || 'No response';
    const status = document.getElementById('stepStatus');
    if (msg.toLowerCase().includes('aligned')) {
        status.innerHTML = '<span style="color:var(--green)">' + msg.replace(/\n/g, ' | ') + '</span>';
    } else {
        status.innerHTML = '<span style="color:var(--red)">' + msg.replace(/\n/g, ' | ') + '</span>';
    }
}

async function doCalSave() {
    if (!confirm('Save current calibration to flash?')) return;
    await sendCmd('Cs');
}

async function doCalLoad() {
    await sendCmd('Cl');
}

function setParam(id) {
    const val = document.getElementById(id).value;
    sendCmd(id + val);
}

function setCurrentParam(suffix) {
    // Send to both Iq (MQ*) and Id (MD*) axes
    const val = document.getElementById('MQ' + suffix).value;
    sendCmd('MQ' + suffix + val);
    sendCmd('MD' + suffix + val);
}

function readParam(id) {
    // Send command without value to read current
    sendCmd(id);
}

function sendConsole() {
    const el = document.getElementById('consoleIn');
    if (el.value.trim()) {
        sendCmd(el.value.trim());
        el.value = '';
    }
}

function sendStop() {
    continuousMode = false;
    if (firmwareType === 'current_test') {
        sendCmd('0');
        setPidEnabled(false);
    } else {
        sendCmd('T0');
    }
}

function motorRunVel(direction) {
    const speed = Math.abs(parseFloat(document.getElementById('motorSpeed').value) || 50);
    sendCmd('Bv' + (direction * speed));
}

function motorRunCur(direction) {
    const cur = Math.abs(parseFloat(document.getElementById('motorCurrent').value) || 0.5);
    sendCmd('Bt' + (direction * cur));
}

function motorStop() {
    sendCmd('B');
}

function motorCoast() {
    sendCmd('Bx');
}

async function runSweep() {
    const btn = document.getElementById('stepBtn');
    const status = document.getElementById('stepStatus');
    if (btn) btn.disabled = true;
    status.textContent = 'Running sweep...';
    try {
        const resp = await fetch('/sweep', { method: 'POST',
            headers: {'Content-Type': 'application/json'} });
        const data = await resp.json();
        if (data.data && data.data.length > 0) {
            plotData = data;
            status.textContent = data.data.length + ' samples';
            drawPlot();
            computeMetrics();
        } else {
            status.textContent = 'No data received';
        }
    } catch(e) {
        status.textContent = 'Error: ' + e.message;
    }
    if (btn) btn.disabled = false;
}

function adaptUIForFirmware(fw) {
    firmwareType = fw;
    if (fw !== 'current_test') return;

    // Update title
    document.getElementById('topTitle').innerHTML = '<span>Bare-Metal</span> Test';

    // Hide SimpleFOC-specific sections
    ['sec-autotune', 'sec-current', 'sec-velocity', 'sec-position', 'sec-limits', 'sec-mode'].forEach(function(id) {
        document.getElementById(id).style.display = 'none';
    });

    // Hide Align/Report, show Read
    document.getElementById('btnAlign').style.display = 'none';
    document.getElementById('btnReport').style.display = 'none';
    document.getElementById('btnRead').style.display = '';

    // Show voltage control, current control, and ADC/filter sections
    document.getElementById('sec-voltage').style.display = '';
    document.getElementById('sec-current-ctrl').style.display = '';
    document.getElementById('sec-adc-filter').style.display = '';

    // Replace step mode dropdown with label, default to current step
    pidEnabled = true;
    const stepMode = document.getElementById('stepMode');
    const label = document.createElement('span');
    label.textContent = 'Current Step (A)';
    label.style.color = 'var(--text2)';
    label.style.fontSize = '0.9em';
    label.id = 'stepLabel';
    stepMode.parentElement.replaceChild(label, stepMode);
    stepLabelEl = label;
}

function toggleSection(id) {
    document.getElementById(id).classList.toggle('collapsed');
}

function onStepModeChange() {
    const mode = document.getElementById('stepMode').value;
    document.getElementById('sinePeriod').style.display = (mode === 'w') ? '' : 'none';
}

// --- Continuous mode ---
let stepRunning = false;

function toggleContinuous() {
    continuousMode = !continuousMode;
    const btn = document.getElementById('contBtn');
    if (continuousMode) {
        btn.textContent = 'Stop';
        btn.className = 'danger';
        if (!stepRunning) runStepTest();
    } else {
        btn.textContent = 'Continuous';
        btn.className = '';
    }
}

// --- Step test ---
async function runStepTest() {
    if (stepRunning) return;
    stepRunning = true;
    const modeEl = document.getElementById('stepMode');
    const mode = modeEl ? modeEl.value : 'q';
    const value = document.getElementById('stepValue').value;
    const btn = document.getElementById('stepBtn');
    const status = document.getElementById('stepStatus');
    btn.disabled = true;
    status.textContent = continuousMode ? 'Running (continuous)...' : 'Running...';
    try {
        // Send PID gains and enable PID on firmware before current step
        if (firmwareType === 'current_test' && pidEnabled) {
            await sendCmd('P' + document.getElementById('ctKp').value);
            await sendCmd('I' + document.getElementById('ctKi').value);
            await sendCmd('D' + document.getElementById('ctKd').value);
            await sendCmd('C0');
            await new Promise(r => setTimeout(r, 50));
        }
        const stepBody = {mode: mode, value: value};
        if (mode === 'w') {
            stepBody.period = document.getElementById('sinePeriod').value;
        }
        await fetch('/step', { method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(stepBody) });
        // Result arrives via SSE step_result event -> handleStepResult()
    } catch(e) {
        status.textContent = 'Error: ' + e.message;
        stepRunning = false;
        continuousMode = false;
        btn.disabled = false;
    }
}

function handleStepResult(data) {
    stepRunning = false;
    const btn = document.getElementById('stepBtn');
    const status = document.getElementById('stepStatus');
    btn.disabled = false;
    if (data.error) {
        status.textContent = data.error;
        continuousMode = false;
    } else if (data.data && data.data.length > 0) {
        plotData = data;
        status.textContent = data.data.length + ' samples';
        drawPlot();
        computeMetrics();
    } else {
        status.textContent = 'No data received';
    }
    if (continuousMode) {
        setTimeout(runStepTest, 0);
    } else {
        const contBtn = document.getElementById('contBtn');
        contBtn.textContent = 'Continuous';
        contBtn.className = '';
    }
}

// --- Plotting ---
const COLORS = ['#58a6ff', '#f85149', '#3fb950', '#d29922', '#bc8cff', '#f0883e', '#56d4dd', '#db61a2'];

// Zoom/pan state for the live plot
let zoomState = null;  // {xmin, xmax, ymin, ymax} or null = auto
let panStart = null;    // {x, y, xmin, xmax, ymin, ymax} during drag

function getPlotRanges(pData) {
    const data = pData.data;
    const header = pData.header;
    const ncols = header.length;
    const xmin = data[0][0], xmax = data[data.length - 1][0];
    // Compute Y range from non-raw, non-hidden columns only
    let ymin = Infinity, ymax = -Infinity;
    for (const row of data) {
        for (let c = 1; c < ncols; c++) {
            if (header[c].startsWith('raw')) continue;
            if (hiddenSeries.has(header[c])) continue;
            if (row[c] < ymin) ymin = row[c];
            if (row[c] > ymax) ymax = row[c];
        }
    }
    if (ymin === Infinity) { ymin = 0; ymax = 1; }
    const ypad = (ymax - ymin) * 0.08 || 0.1;
    return {xmin, xmax, ymin: ymin - ypad, ymax: ymax + ypad};
}

function drawPlotOnCanvas(canvasId, pData, legendId, overrideRange) {
    if (!pData || !pData.data.length) return;
    const canvas = document.getElementById(canvasId);
    const rect = canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    const W = rect.width, H = rect.height;
    const pad = {top: 10, right: 16, bottom: 36, left: 56};
    const pw = W - pad.left - pad.right, ph = H - pad.top - pad.bottom;

    const header = pData.header;
    const data = pData.data;
    const ncols = header.length;

    // Use override range (zoom) or compute from data
    const r = overrideRange || getPlotRanges(pData);
    const xmin = r.xmin, xmax = r.xmax, ymin = r.ymin, ymax = r.ymax;

    function tx(v) { return pad.left + (v - xmin) / (xmax - xmin) * pw; }
    function ty(v) { return pad.top + (1 - (v - ymin) / (ymax - ymin)) * ph; }

    // Nice tick generation
    function niceStep(range, maxTicks) {
        if (range <= 0) return 1;
        const rough = range / maxTicks;
        const mag = Math.pow(10, Math.floor(Math.log10(rough)));
        const norm = rough / mag;
        let step;
        if (norm <= 1.5) step = 1;
        else if (norm <= 3.5) step = 2;
        else if (norm <= 7.5) step = 5;
        else step = 10;
        return step * mag;
    }
    function niceTicks(lo, hi, maxTicks) {
        const step = niceStep(hi - lo, maxTicks);
        const ticks = [];
        const first = Math.ceil(lo / step) * step;
        for (let v = first; v <= hi + step * 0.001; v += step) ticks.push(v);
        return {ticks, step};
    }

    ctx.clearRect(0, 0, W, H);

    // Clip plot area
    ctx.save();
    ctx.beginPath();
    ctx.rect(pad.left, pad.top, pw, ph);
    ctx.clip();

    // Precompute normalization for raw ADC columns: map to center of Y range
    const rawNorm = {};  // col index -> {offset, scale}
    const yMid = (ymin + ymax) / 2;
    const ySpan = ymax - ymin;
    for (let c = 1; c < ncols; c++) {
        if (!header[c].startsWith('raw')) continue;
        let rmin = Infinity, rmax = -Infinity;
        for (let i = 0; i < data.length; i++) {
            const v = data[i][c];
            if (v < rmin) rmin = v;
            if (v > rmax) rmax = v;
        }
        const rSpan = rmax - rmin || 1;
        const rMid = (rmin + rmax) / 2;
        // Map raw values: (v - rMid) / rSpan * ySpan * 0.8 + yMid
        rawNorm[c] = {rMid, rSpan};
    }

    // Data series (skip column 0 = time, skip hidden)
    for (let c = 1; c < ncols; c++) {
        if (hiddenSeries.has(header[c])) continue;
        ctx.strokeStyle = COLORS[(c - 1) % COLORS.length];
        ctx.lineWidth = c === 1 ? 2 : 1.5;
        ctx.beginPath();
        let started = false;
        for (let i = 0; i < data.length; i++) {
            const x = tx(data[i][0]);
            let val = data[i][c];
            if (rawNorm[c]) {
                const n = rawNorm[c];
                val = (val - n.rMid) / n.rSpan * ySpan * 0.8 + yMid;
            }
            const y = ty(val);
            if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
        }
        ctx.stroke();
    }
    ctx.restore();

    // Grid (drawn on top so labels aren't clipped)
    ctx.strokeStyle = '#21262d';
    ctx.lineWidth = 1;
    ctx.font = '11px ' + getComputedStyle(document.body).getPropertyValue('--mono');
    ctx.fillStyle = '#8b949e';

    const xt = niceTicks(xmin, xmax, 6);
    ctx.textAlign = 'center';
    for (const v of xt.ticks) {
        const x = tx(v);
        ctx.beginPath(); ctx.moveTo(x, pad.top); ctx.lineTo(x, pad.top + ph); ctx.stroke();
        const decimals = xt.step >= 1 ? 0 : xt.step >= 0.1 ? 1 : 2;
        ctx.fillText(v.toFixed(decimals), x, H - pad.bottom + 14);
    }

    const yt = niceTicks(ymin, ymax, 5);
    ctx.textAlign = 'right';
    for (const v of yt.ticks) {
        const y = ty(v);
        ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(pad.left + pw, y); ctx.stroke();
        const decimals = yt.step >= 1 ? 0 : yt.step >= 0.1 ? 1 : yt.step >= 0.01 ? 2 : 3;
        ctx.fillText(v.toFixed(decimals), pad.left - 6, y + 4);
    }

    ctx.textAlign = 'center';
    ctx.fillText('ms', pad.left + pw / 2, H - 2);

    // Legend with clickable toggles and group buttons
    if (legendId) {
        const legendEl = document.getElementById(legendId);
        legendEl.innerHTML = '';
        // Group buttons
        const groups = [
            {label: 'All', filter: () => header.slice(1)},
            {label: 'Currents', filter: () => header.slice(1).filter(h => h.startsWith('I') || h === 'Iq_target')},
            {label: 'Voltages', filter: () => header.slice(1).filter(h => h.startsWith('V'))},
            {label: 'Phase', filter: () => header.slice(1).filter(h => h.startsWith('Ia') || h.startsWith('Ib') || h.startsWith('Ic'))},
            {label: 'Raw ADC', filter: () => header.slice(1).filter(h => h.startsWith('raw'))},
        ];
        for (const g of groups) {
            const btn = document.createElement('button');
            btn.className = 'legend-group';
            btn.textContent = g.label;
            btn.onclick = () => {
                const cols = g.filter();
                if (!cols.length) return;
                const allVisible = cols.every(h => !hiddenSeries.has(h));
                if (g.label === 'All') {
                    if (hiddenSeries.size === 0) header.slice(1).forEach(h => hiddenSeries.add(h));
                    else hiddenSeries.clear();
                } else if (allVisible) {
                    cols.forEach(h => hiddenSeries.add(h));
                } else {
                    cols.forEach(h => hiddenSeries.delete(h));
                }
                drawPlot();
            };
            legendEl.appendChild(btn);
        }
        // Individual series toggles
        for (let c = 1; c < ncols; c++) {
            const color = COLORS[(c - 1) % COLORS.length];
            const name = header[c];
            const label = rawNorm[c] ? name + ' (scaled)' : name;
            const item = document.createElement('div');
            item.className = 'legend-item' + (hiddenSeries.has(name) ? ' hidden' : '');
            item.innerHTML = `<div class="legend-swatch" style="background:${color}"></div>${label}`;
            item.onclick = () => {
                if (hiddenSeries.has(name)) hiddenSeries.delete(name);
                else hiddenSeries.add(name);
                drawPlot();
            };
            legendEl.appendChild(item);
        }
    }

    // Store layout for mouse interaction
    canvas._plotLayout = {pad, pw, ph, W, H, xmin, xmax, ymin, ymax, tx, ty, header, data, ncols, rawNorm, yMid, ySpan};
}

function drawPlot() {
    drawPlotOnCanvas('plotCanvas', plotData, 'plotLegend', zoomState);
}

// --- Plot mouse interaction (zoom, pan, hover tooltip) ---
(function() {
    const canvas = document.getElementById('plotCanvas');
    const tooltip = document.getElementById('plotTooltip');

    // Find nearest data index for a given x pixel position
    function nearestIdx(layout, px) {
        const dataX = layout.xmin + (px - layout.pad.left) / layout.pw * (layout.xmax - layout.xmin);
        const data = layout.data;
        let lo = 0, hi = data.length - 1;
        while (lo < hi) {
            const mid = (lo + hi) >> 1;
            if (data[mid][0] < dataX) lo = mid + 1; else hi = mid;
        }
        if (lo > 0 && Math.abs(data[lo-1][0] - dataX) < Math.abs(data[lo][0] - dataX)) lo--;
        return lo;
    }

    canvas.addEventListener('mousemove', function(e) {
        const layout = canvas._plotLayout;
        if (!layout || !plotData) { tooltip.style.display = 'none'; return; }
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;

        // Handle drag-pan
        if (panStart) {
            const dx = (mx - panStart.mx) / layout.pw * (panStart.xmax - panStart.xmin);
            const dy = (my - panStart.my) / layout.ph * (panStart.ymax - panStart.ymin);
            zoomState = {
                xmin: panStart.xmin - dx, xmax: panStart.xmax - dx,
                ymin: panStart.ymin + dy, ymax: panStart.ymax + dy
            };
            drawPlot();
            tooltip.style.display = 'none';
            return;
        }

        if (mx < layout.pad.left || mx > layout.pad.left + layout.pw ||
            my < layout.pad.top || my > layout.pad.top + layout.ph) {
            tooltip.style.display = 'none';
            return;
        }

        const idx = nearestIdx(layout, mx);
        const row = layout.data[idx];
        const sx = layout.tx(row[0]);

        // Find closest series to mouse Y (account for raw normalization, skip hidden)
        function plotVal(c, val) {
            if (layout.rawNorm && layout.rawNorm[c]) {
                const n = layout.rawNorm[c];
                return (val - n.rMid) / n.rSpan * layout.ySpan * 0.8 + layout.yMid;
            }
            return val;
        }
        let closestCol = 1;
        let closestDist = Infinity;
        for (let c = 1; c < layout.ncols; c++) {
            if (hiddenSeries.has(layout.header[c])) continue;
            const sy = layout.ty(plotVal(c, row[c]));
            const d = Math.abs(sy - my);
            if (d < closestDist) { closestDist = d; closestCol = c; }
        }

        // Draw crosshair + dot on closest series
        drawPlot();
        const ctx = canvas.getContext('2d');
        const dpr = window.devicePixelRatio || 1;
        ctx.save();
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.strokeStyle = 'rgba(255,255,255,0.25)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(sx, layout.pad.top);
        ctx.lineTo(sx, layout.pad.top + layout.ph);
        ctx.stroke();
        ctx.setLineDash([]);
        // Dot on closest line
        const dotY = layout.ty(plotVal(closestCol, row[closestCol]));
        const dotColor = COLORS[(closestCol - 1) % COLORS.length];
        ctx.fillStyle = dotColor;
        ctx.beginPath();
        ctx.arc(sx, dotY, 4, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.restore();

        // Build tooltip (highlight closest series with a dot, skip hidden)
        let html = `<div class="tt-row" style="color:var(--text2)">${layout.header[0]}: ${row[0].toFixed(1)}</div>`;
        for (let c = 1; c < layout.ncols; c++) {
            if (hiddenSeries.has(layout.header[c])) continue;
            const color = COLORS[(c - 1) % COLORS.length];
            const marker = c === closestCol ? `<span style="font-size:8px;color:${color}">&#9679; </span>` : '';
            html += `<div class="tt-row"><div class="tt-swatch" style="background:${color}"></div>${marker}${layout.header[c]}: ${row[c].toFixed(4)}</div>`;
        }
        tooltip.innerHTML = html;
        tooltip.style.display = 'block';

        // Position tooltip (flip if near edges)
        const container = canvas.parentElement.getBoundingClientRect();
        let tx = mx + 14, ty = my - 10;
        if (tx + tooltip.offsetWidth > container.width - 8) tx = mx - tooltip.offsetWidth - 14;
        if (ty + tooltip.offsetHeight > container.height - 8) ty = container.height - tooltip.offsetHeight - 8;
        if (ty < 4) ty = 4;
        tooltip.style.left = tx + 'px';
        tooltip.style.top = ty + 'px';
    });

    canvas.addEventListener('mouseleave', function() {
        tooltip.style.display = 'none';
        if (!panStart) drawPlot();
    });

    // Wheel zoom
    canvas.addEventListener('wheel', function(e) {
        e.preventDefault();
        const layout = canvas._plotLayout;
        if (!layout || !plotData) return;
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;

        const factor = e.deltaY > 0 ? 1.15 : 1 / 1.15;
        const r = zoomState || getPlotRanges(plotData);

        // Zoom centered on mouse position
        const fx = (mx - layout.pad.left) / layout.pw;
        const fy = 1 - (my - layout.pad.top) / layout.ph;
        const cx = r.xmin + fx * (r.xmax - r.xmin);
        const cy = r.ymin + fy * (r.ymax - r.ymin);

        zoomState = {
            xmin: cx - (cx - r.xmin) * factor,
            xmax: cx + (r.xmax - cx) * factor,
            ymin: cy - (cy - r.ymin) * factor,
            ymax: cy + (r.ymax - cy) * factor
        };
        drawPlot();
    }, {passive: false});

    // Pan: middle-click or shift+left-click drag
    canvas.addEventListener('mousedown', function(e) {
        if (e.button === 1 || (e.button === 0 && e.shiftKey)) {
            e.preventDefault();
            const rect = canvas.getBoundingClientRect();
            const r = zoomState || (plotData ? getPlotRanges(plotData) : null);
            if (!r) return;
            panStart = {mx: e.clientX - rect.left, my: e.clientY - rect.top, ...r};
        }
    });
    window.addEventListener('mouseup', function() {
        if (panStart) { panStart = null; }
    });

    // Double-click to reset zoom
    canvas.addEventListener('dblclick', function() {
        zoomState = null;
        drawPlot();
    });
})();

function computeMetrics() {
    if (!plotData || !plotData.data.length) return;
    const el = document.getElementById('plotMetrics');
    const allData = plotData.data;
    const header = plotData.header;

    // current_test mode without PID: voltage target != current actual, just show count
    if (firmwareType === 'current_test' && !pidEnabled) {
        el.innerHTML = `<span>samples <span class="val">${allData.length}</span></span>`;
        return;
    }

    const target = allData[allData.length - 1][1];

    // Skip pre-step baseline: find where target column goes nonzero
    let stepIdx = 0;
    for (let i = 0; i < allData.length; i++) {
        if (Math.abs(allData[i][1]) > Math.abs(target) * 0.5) { stepIdx = i; break; }
    }
    const data = allData.slice(stepIdx);
    if (data.length < 2) return;

    const tStep = data[0][0];
    const actual = data.map(r => r[2]);
    const times = data.map(r => r[0] - tStep);

    // Overshoot — peak beyond target in the direction of the step
    const peak = target >= 0 ? Math.max(...actual) : Math.min(...actual);
    const overshoot = target !== 0 ? ((peak - target) / target * 100) : 0;

    // Settling time (within 5% of target)
    const band = Math.abs(target) * 0.05 || 0.01;
    let settleIdx = data.length - 1;
    for (let i = data.length - 1; i >= 0; i--) {
        if (Math.abs(actual[i] - target) > band) { settleIdx = i + 1; break; }
    }
    const settleTime = settleIdx < data.length ? times[settleIdx] : times[times.length - 1];

    // Steady-state error (last 10% average)
    const tail = actual.slice(Math.floor(actual.length * 0.9));
    const ssAvg = tail.reduce((a, b) => a + b, 0) / tail.length;
    const ssErr = target !== 0 ? ((ssAvg - target) / target * 100) : ssAvg;

    el.innerHTML = `
        <span>${header[2]}: overshoot <span class="val">${overshoot.toFixed(1)}%</span></span>
        <span>settle (5%) <span class="val">${settleTime.toFixed(0)} ms</span></span>
        <span>SS error <span class="val">${ssErr.toFixed(1)}%</span></span>
        <span>samples <span class="val">${data.length}</span></span>
    `;
}

// --- Auto-tune ---
let autotuneRunning = false;
let finalPlots = {};       // {current: {plot, metrics, gains}, velocity: {...}}
let activeFinalTab = null; // currently displayed final-plot tab

function startAutotune(loops) {
    if (autotuneRunning) return;
    autotuneRunning = true;
    finalPlots = {};
    activeFinalTab = null;
    document.getElementById('autotuneLog').textContent = '';
    document.getElementById('autotuneResult').style.display = 'none';
    document.getElementById('autotuneStopBtn').style.display = '';
    document.getElementById('sidebar').classList.add('tuning-active');
    showLivePlot(); // ensure live view during tuning
    for (const btn of document.querySelectorAll('#autotuneBtns .primary')) btn.style.display = 'none';
    fetch('/autotune', { method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: 'start', loops: loops}) });
}

function stopAutotune() {
    fetch('/autotune', { method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: 'stop'}) });
}

function autotuneFinished() {
    autotuneRunning = false;
    document.getElementById('autotuneStopBtn').style.display = 'none';
    document.getElementById('sidebar').classList.remove('tuning-active');
    for (const btn of document.querySelectorAll('#autotuneBtns .primary')) btn.style.display = '';
}

function appendAutotuneLog(text) {
    const el = document.getElementById('autotuneLog');
    el.textContent += text + '\n';
    el.scrollTop = el.scrollHeight;
}

function showLivePlot() {
    document.getElementById('livePlotContainer').style.display = '';
    document.getElementById('plotMetrics').style.display = '';
    document.getElementById('plotLegend').style.display = '';
    document.getElementById('finalPlotArea').style.display = 'none';
}

function showFinalPlots(tab) {
    document.getElementById('livePlotContainer').style.display = 'none';
    document.getElementById('plotMetrics').style.display = 'none';
    document.getElementById('plotLegend').style.display = 'none';
    const area = document.getElementById('finalPlotArea');
    area.style.display = 'flex';
    // Build tabs
    const tabsEl = document.getElementById('finalPlotTabs');
    const loopNames = Object.keys(finalPlots);
    if (!tab) tab = loopNames[0];
    activeFinalTab = tab;
    tabsEl.innerHTML = '';
    for (const name of loopNames) {
        const label = name === 'current' ? 'Current' : 'Velocity';
        const cls = 'final-plot-tab' + (name === tab ? ' active' : '');
        tabsEl.innerHTML += '<div class="' + cls + '" onclick="showFinalPlots(\'' + name + '\')">' + label + '</div>';
    }
    // Draw the selected plot
    const fp = finalPlots[tab];
    if (fp && fp.plot) {
        drawPlotOnCanvas('finalPlotCanvas', fp.plot, null);
        // Show metrics
        const m = fp.metrics;
        const g = fp.gains;
        const metricsEl = document.getElementById('finalPlotMetrics');
        const gainStr = Object.entries(g).map(function(e) { return e[0] + '=' + e[1]; }).join(' ');
        metricsEl.innerHTML =
            '<span>overshoot <span class="val">' + m.overshoot_pct.toFixed(1) + '%</span></span>' +
            '<span>settle <span class="val">' + m.settle_time_ms.toFixed(0) + ' ms</span></span>' +
            '<span>SS err <span class="val">' + m.ss_error_pct.toFixed(1) + '%</span></span>' +
            '<span style="color:var(--accent)">' + gainStr + '</span>';
    }
}

function handleAutotuneEvent(payload) {
    if (payload.type === 'status') {
        appendAutotuneLog('[' + (payload.phase || '') + '] ' + payload.msg);
    } else if (payload.type === 'trial') {
        const m = payload.metrics;
        const params = Object.entries(payload.params).map(([k,v]) => k+'='+v).join(' ');
        appendAutotuneLog(
            '  #' + payload.iteration + ' ' + params +
            ' \u2192 overshoot=' + m.overshoot_pct.toFixed(1) + '%' +
            ' settle=' + m.settle_time_ms.toFixed(0) + 'ms' +
            ' ss_err=' + m.ss_error_pct.toFixed(1) + '%' +
            (m.oscillating ? ' OSCILLATING' : '')
        );
        // Update main plot with this trial's data
        if (payload.plot && payload.plot.data && payload.plot.data.length > 0) {
            plotData = payload.plot;
            drawPlot();
            computeMetrics();
        }
    } else if (payload.type === 'final_plot') {
        // Store the clean final-result plot for this loop
        finalPlots[payload.loop] = {
            plot: payload.plot,
            metrics: payload.metrics,
            gains: payload.gains
        };
        appendAutotuneLog('[' + payload.loop + '] Final validation captured');
    } else if (payload.type === 'done') {
        autotuneFinished();
        appendAutotuneLog('--- Auto-tune complete ---');
        showAutotuneResult(payload.gains);
        // Switch to final-plot view
        if (Object.keys(finalPlots).length > 0) {
            showFinalPlots(null);
        }
    } else if (payload.type === 'error') {
        autotuneFinished();
        appendAutotuneLog('ERROR: ' + payload.msg);
    }
}

function showAutotuneResult(gains) {
    const el = document.getElementById('autotuneResult');
    let html = '<div class="autotune-result"><h4>Tuned Gains</h4>';
    if (gains.current) {
        const c = gains.current;
        html += '<div>Current: P=' + c.P + ' I=' + c.I + ' Tf=' + c.Tf + '</div>';
        document.getElementById('MQP').value = c.P;
        document.getElementById('MQI').value = c.I;
        document.getElementById('MQF').value = c.Tf;
    }
    if (gains.velocity) {
        const v = gains.velocity;
        html += '<div>Velocity: P=' + v.P + ' I=' + v.I + ' D=' + v.D + ' ramp=' + v.ramp + ' Tf=' + v.Tf + '</div>';
        document.getElementById('MVP').value = v.P;
        document.getElementById('MVI').value = v.I;
        document.getElementById('MVD').value = v.D;
        document.getElementById('MVR').value = v.ramp;
        document.getElementById('MVF').value = v.Tf;
    }
    html += '<button class="primary" onclick="bakeFirmware()" style="margin-top:8px;width:100%">Bake to Firmware</button>';
    html += '</div>';
    el.innerHTML = html;
    el.style.display = '';
}

async function bakeFirmware() {
    try {
        const resp = await fetch('/bake', { method: 'POST', headers: {'Content-Type': 'application/json'} });
        const data = await resp.json();
        if (data.ok) {
            // Show in a modal-style overlay
            const overlay = document.createElement('div');
            overlay.style.cssText = 'position:fixed;inset:0;background:rgba(0,0,0,0.7);display:flex;align-items:center;justify-content:center;z-index:1000';
            overlay.onclick = function(e) { if (e.target === overlay) overlay.remove(); };
            const box = document.createElement('div');
            box.style.cssText = 'background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:20px;max-width:600px;width:90%;max-height:80vh;overflow:auto';
            box.innerHTML = '<h3 style="margin-bottom:8px;color:var(--green)">Gains saved to tuned_gains.txt</h3>' +
                '<pre style="font-family:var(--mono);font-size:12px;background:var(--bg);padding:12px;border-radius:4px;overflow-x:auto;white-space:pre-wrap">' +
                data.code.replace(/</g,'&lt;') + '</pre>' +
                '<p style="color:var(--text2);font-size:12px;margin-top:8px">File: ' + data.path + '</p>' +
                '<button onclick="this.parentElement.parentElement.remove()" style="margin-top:12px">Close</button>';
            overlay.appendChild(box);
            document.body.appendChild(overlay);
        } else {
            alert('Error: ' + (data.msg || 'Unknown error'));
        }
    } catch(e) {
        alert('Bake failed: ' + e.message);
    }
}

// --- VMOT polling ---
let vmotInterval = null;
function parseVmot(line) {
    const m = line.match(/VMOT=([0-9.]+)/);
    if (m) {
        const v = parseFloat(m[1]);
        const el = document.getElementById('vmotDisplay');
        if (v > 0.1) {
            el.textContent = 'VMOT: ' + v.toFixed(1) + 'V';
            el.style.color = v < 10 ? 'var(--text2)' : 'var(--green)';
        } else {
            el.textContent = 'VMOT: --';
            el.style.color = 'var(--text2)';
        }
    }
}
function startVmotPolling() {
    if (vmotInterval) return;
    vmotInterval = setInterval(() => sendCmd('V'), 1000);
}

// --- Read all params from controller ---
function readAllParams() {
    fetch('/read_params').then(r => r.json()).then(params => {
        for (const [id, val] of Object.entries(params)) {
            const el = document.getElementById(id);
            if (el) el.value = parseFloat(val.toFixed(6));
        }
        console.log('Params loaded:', params);
    }).catch(e => console.error('Failed to read params:', e));
}

function doReconnect() {
    // Close existing SSE connection
    if (evtSource) { evtSource.close(); evtSource = null; }
    // Clear console
    document.getElementById('consoleOut').textContent = '';
    // Reset status dot
    document.getElementById('statusDot').className = 'status-dot';
    // Re-fetch info and adapt UI
    fetch('/info').then(r => r.json()).then(info => {
        document.getElementById('portInfo').textContent = info.port;
        if (info.firmware) adaptUIForFirmware(info.firmware);
        setTimeout(readAllParams, 500);
    });
    // Reconnect SSE
    connectSSE();
}

// --- Init ---
window.addEventListener('load', () => {
    fetch('/info').then(r => r.json()).then(info => {
        document.getElementById('portInfo').textContent = info.port;
        if (info.firmware) {
            adaptUIForFirmware(info.firmware);
        }
        // Read params from controller after UI is set up
        setTimeout(readAllParams, 500);
    });
    connectSSE();
    startVmotPolling();
});

window.addEventListener('resize', () => {
    if (plotData) drawPlot();
    if (activeFinalTab && finalPlots[activeFinalTab]) showFinalPlots(activeFinalTab);
});
</script>
</body>
</html>"""


class TuneHandler(http.server.BaseHTTPRequestHandler):
    serial_mgr = None  # set by main
    serial_port_name = ""
    auto_tuner = None  # set by main

    def log_message(self, format, *args):
        pass  # quiet

    def do_GET(self):
        if self.path == "/":
            self._send_html(HTML_PAGE)
        elif self.path == "/stream":
            self._handle_sse()
        elif self.path == "/info":
            self._send_json({"port": self.serial_port_name,
                             "firmware": self.serial_mgr.firmware_type})
        elif self.path == "/read_params":
            results = self.serial_mgr.read_params()
            self._send_json(results)
        else:
            self.send_error(404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length) if length else b""
        try:
            data = json.loads(body) if body else {}
        except json.JSONDecodeError:
            data = {}

        if self.path == "/send":
            cmd = data.get("cmd", "")
            if cmd:
                self.serial_mgr.send(cmd)
            self._send_json({"ok": True})
        elif self.path == "/step":
            mode = data.get("mode", "q")
            value = data.get("value", "0.5")
            period = data.get("period", "")
            mgr = self.serial_mgr
            def _run_step():
                if mgr.firmware_type == "current_test":
                    cmd = f"T{value}"
                    result = mgr.run_step_test(cmd, timeout=3.0)
                else:
                    cmd = f"S{mode}{value}"
                    if mode == "w" and period:
                        cmd += f",{period}"
                        # Timeout = 3 cycles + margin
                        p_ms = float(period) if period else 1000
                        timeout = (p_ms * 3 / 1000.0) + 2.0
                    else:
                        timeout = 5.0
                    result = mgr.run_step_test(cmd, timeout=timeout)
                mgr.broadcast_sse("step_result", json.dumps(result))
            threading.Thread(target=_run_step, daemon=True).start()
            self._send_json({"ok": True})
        elif self.path == "/align":
            mgr = self.serial_mgr
            def _run_align():
                q = queue.Queue(maxsize=500)
                with mgr.step_lock:
                    mgr.step_queue = q
                mgr.send("A")
                lines = []
                deadline = time.time() + 10.0
                try:
                    while time.time() < deadline:
                        try:
                            line = q.get(timeout=0.2)
                            lines.append(line)
                            low = line.lower()
                            if "aligned" in low or "failed" in low or "err" in low:
                                break
                        except queue.Empty:
                            continue
                finally:
                    with mgr.step_lock:
                        mgr.step_queue = None
                result = "\n".join(lines) if lines else "No response from firmware (check serial port)"
                mgr.broadcast_sse("align_result", json.dumps({"msg": result}))
            threading.Thread(target=_run_align, daemon=True).start()
            self._send_json({"ok": True})
        elif self.path == "/sweep":
            result = self.serial_mgr.run_step_test("S", timeout=10.0)
            self._send_json(result)
        elif self.path == "/autotune":
            self._handle_autotune(data)
        elif self.path == "/bake":
            self._handle_bake()
        else:
            self.send_error(404)

    def _handle_autotune(self, data):
        action = data.get("action", "")
        tuner = self.auto_tuner
        if action == "start":
            if tuner.running:
                self._send_json({"ok": False, "msg": "Already running"})
                return
            loops = data.get("loops", ["current", "velocity"])
            tuner.start(loops=loops)
            self._send_json({"ok": True})
        elif action == "stop":
            tuner.stop()
            self._send_json({"ok": True})
        else:
            self._send_json({"ok": False, "msg": "Unknown action"})

    def _handle_bake(self):
        tuner = self.auto_tuner
        if not tuner.final_gains:
            self._send_json({"ok": False, "msg": "No auto-tune results available"})
            return
        gains = tuner.final_gains
        lines = [
            "// Auto-tuned PID gains",
            f"// Generated by tune.py on {time.strftime('%Y-%m-%d %H:%M:%S')}",
            "",
        ]
        if "current" in gains:
            c = gains["current"]
            lines += [
                "// Current loop (Iq)",
                f"motor.PID_current_q.P = {c['P']};",
                f"motor.PID_current_q.I = {c['I']};",
                f"motor.PID_current_q.output_ramp = 0;",
                f"motor.LPF_current_q.Tf = {c['Tf']};",
                "",
                "// Current loop (Id)",
                f"motor.PID_current_d.P = {c['P']};",
                f"motor.PID_current_d.I = {c['I']};",
                f"motor.PID_current_d.output_ramp = 0;",
                f"motor.LPF_current_d.Tf = {c['Tf']};",
                "",
            ]
        if "velocity" in gains:
            v = gains["velocity"]
            lines += [
                "// Velocity loop",
                f"motor.PID_velocity.P = {v['P']};",
                f"motor.PID_velocity.I = {v['I']};",
                f"motor.PID_velocity.D = {v['D']};",
                f"motor.PID_velocity.output_ramp = {v['ramp']};",
                f"motor.LPF_velocity.Tf = {v['Tf']};",
                "",
            ]
        snippet = "\n".join(lines)
        out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tuned_gains.txt")
        with open(out_path, "w") as f:
            f.write(snippet)
        self._send_json({"ok": True, "path": out_path, "code": snippet})

    def _send_html(self, html):
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(html.encode())

    def _send_json(self, obj):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(json.dumps(obj).encode())

    def _handle_sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        q = self.serial_mgr.add_sse_client()
        try:
            while True:
                try:
                    msg = q.get(timeout=1.0)
                    event_name, data = msg
                    if event_name:
                        self.wfile.write(f"event: {event_name}\ndata: {data}\n\n".encode())
                    else:
                        self.wfile.write(f"data: {data}\n\n".encode())
                    self.wfile.flush()
                except queue.Empty:
                    # Send keepalive comment
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.serial_mgr.remove_sse_client(q)


class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def find_serial_port():
    """Auto-detect serial port.

    Priority order:
    1. Debug probe UART bridge (VID:PID 2e8a:000c) — reliable, no USB CDC issues
    2. RP2350 USB CDC (VID:PID 2e8a:f00f) — fallback
    3. First available /dev/ttyACM* or /dev/ttyUSB*
    """
    import subprocess
    candidates = {"debugger": None, "rp2350": None}
    for port in sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*")):
        try:
            out = subprocess.check_output(
                ["udevadm", "info", "--query=property", port],
                stderr=subprocess.DEVNULL, text=True)
            props = dict(line.split("=", 1) for line in out.splitlines() if "=" in line)
            vid = props.get("ID_USB_VENDOR_ID", props.get("ID_VENDOR_ID", ""))
            pid = props.get("ID_USB_MODEL_ID", props.get("ID_MODEL_ID", ""))
            if vid == "2e8a" and pid == "000c":
                candidates["debugger"] = port
            elif vid == "2e8a" and pid == "f00f":
                candidates["rp2350"] = port
        except Exception:
            continue
    if candidates["debugger"]:
        return candidates["debugger"]
    if candidates["rp2350"]:
        return candidates["rp2350"]
    # Fallback: return first available port
    ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    return ports[0] if ports else None


def main():
    parser = argparse.ArgumentParser(description="Motor FOC Tuning GUI")
    parser.add_argument("--port", "-p", help="Serial port (auto-detect if omitted)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--http-port", type=int, default=0, help="HTTP port (0=random)")
    parser.add_argument("--no-browser", action="store_true", help="Don't open browser")
    args = parser.parse_args()

    serial_port = args.port or find_serial_port()
    if not serial_port:
        print("Error: No serial port found. Specify with --port", file=sys.stderr)
        sys.exit(1)

    print(f"Connecting to {serial_port} at {args.baud} baud...")
    try:
        mgr = SerialManager(serial_port, args.baud)
    except serial.SerialException as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    mgr.detect_firmware()
    print(f"Firmware detected: {mgr.firmware_type}")
    mgr.start()

    TuneHandler.serial_mgr = mgr
    TuneHandler.serial_port_name = serial_port
    TuneHandler.auto_tuner = AutoTuner(mgr, mgr.broadcast_sse)

    server = ThreadedHTTPServer(("", args.http_port), TuneHandler)
    port = server.server_address[1]
    url = f"http://localhost:{port}"

    print(f"Serving on {url}")

    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        mgr.close()
        server.shutdown()


if __name__ == "__main__":
    main()
