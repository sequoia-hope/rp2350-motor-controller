# Motor Firmware - Project Memory

## Flashing firmware via SWD debugger

The board has a CMSIS-DAP debug probe ("Lil debugger fren", VID:PID 0x2e8a:0x000c) connected via SWD. Use OpenOCD from the PlatformIO toolchain to flash without needing BOOTSEL mode:

```bash
~/.platformio/packages/tool-openocd-rp2040-earlephilhower/bin/openocd \
  -s ~/.platformio/packages/tool-openocd-rp2040-earlephilhower/share/openocd/scripts \
  -f interface/cmsis-dap.cfg \
  -c "adapter speed 5000" \
  -f target/rp2350.cfg \
  -c "program .pio/build/<ENV>/firmware.elf verify reset exit"
```

Replace `<ENV>` with the PlatformIO environment name (e.g. `current_test` or `motor_controller`).

**Notes:**
- The probe uses CMSIS-DAPv2 (bulk endpoints, not HID).
- System OpenOCD (`/usr/bin/openocd` v0.12.0) lacks RP2350 target support; must use the PlatformIO-bundled version.
- If OpenOCD fails with "Pipe error" / "unable to find a matching CMSIS-DAP device", power-cycle the debug probe. The motor controller's bulk capacitors can glitch the USB hub on plug/unplug. (For a silent-but-still-enumerating probe, see "Probe UART bridge wedges" below — a USB reset fixes it without unplugging.)
- `pio run -e <ENV> -t upload` uses picotool which requires BOOTSEL mode (unreliable with this board); prefer the OpenOCD method above.

## Probe UART bridge wedges — looks exactly like a firmware hang

The probe's CDC-UART bridge (`/dev/ttyACM*` with `ID_MODEL_ID=000c`) intermittently stops forwarding, usually after repeated flash/reset cycles. **The symptom is indistinguishable from a crashed board**: no boot banner, no reply to any command, while the probe still enumerates and OpenOCD still connects and flashes fine.

Do not debug the firmware on this symptom alone. Attach GDB and get a backtrace first — a wedged bridge shows the target running `loop()` normally in thread mode. Time was lost here chasing pull-downs, lockups and UART registers before checking.

Recover by USB-resetting the probe (no unplugging needed):

```python
import fcntl, os, subprocess
USBDEVFS_RESET = ord('U') << 8 | 20
line = [l for l in subprocess.check_output(['lsusb'], text=True).splitlines() if '2e8a:000c' in l][0]
fd = os.open(f'/dev/bus/usb/{line.split()[1]}/{line.split()[3].rstrip(":")}', os.O_WRONLY)
fcntl.ioctl(fd, USBDEVFS_RESET, 0); os.close(fd)
```

The port re-enumerates in ~2s, possibly at a different `/dev/ttyACM*` — re-detect by `ID_MODEL_ID=000c` rather than assuming the old path.

Related gotchas when scripting against the probe:
- Halting the core over SWD stops the firmware draining the UART FIFO. A "full RX FIFO / overrun" reading taken while halted is an artifact of the measurement.
- OpenOCD `-c "halt"` followed by `-c "exit"` leaves the core **halted**, which then looks like a dead board. Always `resume` before `exit`, or use `reset run`.

## FOC loop must never touch the serial port

`SERIAL_PORT` is `Serial1`, a 115200 UART, and `print()` blocks once the TX FIFO fills. Anything printed per FOC iteration throttles the control loop to the baud rate: a ~28-byte CSV row costs ~2.4ms, which measured out at **407Hz** for `loopFOC()`/`move()` — ~50x slower than the ~20kHz the loop reaches when left alone. The gains assume the fast loop, so the starved loop oscillates and reads as far too much velocity gain. This was invisible historically because the console used to be USB CDC (buffered, 12Mbit/s).

Step tests therefore buffer samples into RAM (`logBegin`/`logSample`/`logDump`) and dump after the loop ends. Keep it that way. `R` reports the achieved rate as `foc_hz`, and each step test prints `foc_iters=N in Nms -> N Hz`.

While the sine demo runs, output is blocked via `outputBlocked()` (guards `doVmot` and `doReport`) — tune.py polls `V` once a second and each reply costs ~1ms. Commands are still **parsed** while blocked; only replies are dropped. This is deliberate: the Stop button sends `T0`, which reaches `doTarget` → `stopSine()`. Blocking input instead would leave a spinning motor with a dead Stop button.

## UI / firmware consistency

Default values for parameters must be kept in sync between the firmware (`src/current_test.cpp` and `src/main.cpp` initializers) and the dashboard (`tune.py` HTML input `value` attributes). When changing a default in one, always update the other.

## After flashing

Always explicitly tell the user when firmware has been flashed. Don't silently combine build+flash — confirm the flash happened.

## RP2350 + SimpleFOC USB CDC bug

**Do NOT pass `Serial` to any global constructor** (e.g. `Commander commander(Serial)`). On RP2350 with arduino-pico USB CDC, taking a reference to `Serial` before the USB stack initializes corrupts the CDC subsystem — the device enumerates but never sends/receives data. Use the no-arg constructor and assign `commander.com_port = &Serial` in `setup()` after `Serial.begin()`.

Similarly, SimpleFOC's `current_sense.init()` starts a DMA-driven ADC engine that can starve USB if running too early. All heavy hardware init (driver, current sense, motor) is deferred to `doAlign()` via `initHardware()`.

## FreeRTOS must stay unlinked (`lib_ignore = FreeRTOS`)

The framework's FreeRTOS library compiles and links even though it's absent from the dependency graph and unreferenced by our source. The core enables FreeRTOS purely on presence — `extern void initFreeRTOS() __attribute__((weak))` (arduino-pico `cores/rp2040/main.cpp:41`) resolving non-null sets `__isFreeRTOS`. `lib_archive = true` does **not** prevent this; only `lib_ignore = FreeRTOS` does.

When FreeRTOS is active, any LittleFS write (i.e. `Cs` / `save_calibration()`) deadlocks: `lfs_flash_prog` → `RP2040::idleOtherCore()` → `__freertos_idle_other_core()`, which notifies core1's idle task and busy-waits on `__otherCoreIdled` forever. **The failure looks like a serial problem** — interrupts still drain the UART FIFO, so the board transmits its boot banner and the RX line looks healthy, but it stops answering commands until reset. Don't chase the UART; get a backtrace.

With FreeRTOS unlinked and no `setup1`/`loop1`, `_multicore` is false and `idleOtherCore()` returns immediately, so flash writes work.

## Commander verbose mode

`commander->verbose = VerboseMode::on_request` is required in `setup()`. The default `user_friendly` answers a query with `PID curr q| P: 3.000`, but `tune.py`'s `read_params()` calls `float()` on the reply and silently drops every parameter, leaving the dashboard on its hardcoded HTML defaults. `VerboseMode::nothing` is wrong here — it suppresses the value too, not just the label.

Note params only reflect firmware values *after* `H`, since `initHardware()` applies them; querying before that returns SimpleFOC library defaults (curr P=3.0, vel P=0.5, curr limit 2.0).

## Hardware: current sense resistors

Current board uses 20mΩ shunt resistors. INA240A1D gain is 20×. With 20mΩ shunts, the INA240 output saturates at ~4A (1.65V headroom / 0.4V per amp). `voltage_sensor_align` is set to 1.0V (reduced from 2.0V) to keep alignment current below the saturation threshold.

## Encoder: MT6701 (not MT6835)

The magnetic encoder is an MT6701 (14-bit, SSI), not MT6835 (21-bit, SPI) as originally labeled. The driver auto-detects the chip type via CRC validation (CRC-6 for MT6701, CRC-8 for MT6835). The 14-bit angle is scaled to 21-bit range internally so the rest of the code is unaffected. The encoder connects via SPI0 through differential transceivers (SIT3088ETK) which shift the response 2 bytes early and 1 bit right.

## USB CDC starvation

Tight loops calling `loopFOC()` + `move()` without yielding will starve TinyUSB's cooperative processing on RP2350 (no background USB task without FreeRTOS). Any continuous FOC mode must include periodic `yield()` calls or rate limiting to keep CDC alive.

## Bus overvoltage guard and brake chopper (F-28)

`busGuard()` runs on every `loop()` pass and inside both step-test spin loops,
self-throttled (RAM-read VMOT at ~FOC rate once the DMA ADC engine is up,
500Hz before that). It folds `motor->current_limit` toward zero across
63-66V and restores the user's limit after recovery — `guard_base_limit`
re-tracks dashboard changes only while the guard is idle, so don't set
current_limit from other code while VMOT is above 63V and expect it to stick.

The brake chopper is compiled with `-DBRAKE_CHOPPER=1` and needs the power
resistor fitted from the "D" terminal (J2.2) to GND (J10.1). The D leg is
dual-role: stepper/second-brushed builds use it as a motor phase and must NOT
define BRAKE_CHOPPER. GPIO9 is D_PWM_L-bar (ACTIVE LOW, pull-down = low FET
ON): the chopper init drives it HIGH and nothing else may touch it, or the
chopper stops conducting — the EG3113 *does* interlock (datasheet V1.2 §8.2:
HIN=1 with LIN-bar=0 gives HO=0 AND LO=0), so leaving CL2 commanded on parks
both FETs off instead of letting CH2 chop. The failure is a silent dead
chopper, not shoot-through. The chopper owns arduino-pico's global
`analogWriteFreq/Range` (nothing else calls analogWrite).

The serial protocol was deliberately left unchanged (tune.py parses `V` and
`R` replies); chopper/guard state is internal only for now.
