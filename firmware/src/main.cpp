#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include "../patches/simplefoc_rp2040_current_sense.h"

// Motor configuration
// MOTOR_MT6701 (1): MT6701 magnetic encoder (SPI, differential transceivers)
// MOTOR_HALLS  (2): Hall sensors (3-wire digital, GPIO 31/32/33)
#define MOTOR_MT6701 1
#define MOTOR_HALLS  2

#ifndef MOTOR_CONFIG
#define MOTOR_CONFIG MOTOR_MT6701
#endif

#if MOTOR_CONFIG == MOTOR_MT6701
#include "MagneticSensorMT6835.h"
#endif

// --- Pin definitions ---
// Board labels A/B/C are swapped vs schematic for A and C.
// PWM and current sense are consistent with each other (schematic naming),
// but the output connector labels on the board are swapped:
//   Board "A" = GPIO6/7 (schematic "C"), ADC2 (GPIO42)
//   Board "B" = GPIO4/5 (schematic "B"), ADC1 (GPIO41)
//   Board "C" = GPIO2/3 (schematic "A"), ADC0 (GPIO40)
// Pin defines use board labels since that's what you see on the hardware.
#define PIN_AH 6
#define PIN_AL 7
#define PIN_BH 4
#define PIN_BL 5
#define PIN_CH 2
#define PIN_CL 3

// Current sense pins (INA240A1D, 20x gain, 20mOhm shunt)
// Matched to board labels (same swap as PWM)
#define PIN_SENSE_A 42  // GPIO42 = ADC2
#define PIN_SENSE_B 41  // GPIO41 = ADC1
#define PIN_SENSE_C 40  // GPIO40 = ADC0

// INA240A1D: 20x gain, shunt resistor selected per motor config
// Available shunt values (swap resistors on board to match)
#define SHUNT_20MOHM 0.020f  // 20mΩ: ±4A range, 0.4 V/A
#define SHUNT_10MOHM 0.010f  // 10mΩ: ±8A range, 0.2 V/A
#define SHUNT_5MOHM  0.005f  //  5mΩ: ±16A range, 0.1 V/A
#if MOTOR_CONFIG == MOTOR_MT6701
#define SHUNT_RESISTOR SHUNT_20MOHM
#elif MOTOR_CONFIG == MOTOR_HALLS
#define SHUNT_RESISTOR SHUNT_10MOHM
#endif
#define CURRENT_AMP_GAIN 20.0f

// --- Current sense saturation: the hard ceiling on current_limit ---
//
// The INA240 output swings between its mid-rail reference (~1.65V) and the 3.3V
// rail, so the largest current it can REPORT is:
//     I_sat = 1.65V / (SHUNT_RESISTOR * CURRENT_AMP_GAIN)
// 20mOhm/20x -> 4.12A.  10mOhm/20x -> 8.25A.
//
// Past I_sat the measurement stops rising while the real current keeps climbing.
// The current PID then sees a too-low current, winds up, and commands MORE
// voltage -- positive feedback with the sense chain blind, bounded only by
// voltage_limit and the winding resistance. current_limit MUST stay under I_sat.
//
// This is the same ceiling that already forced voltage_sensor_align down to 1.0V.
#define CURRENT_SENSE_REF_V  1.65f
#define CURRENT_SENSE_SAT_A  (CURRENT_SENSE_REF_V / (SHUNT_RESISTOR * CURRENT_AMP_GAIN))
#define CURRENT_LIMIT_MAX_A  (CURRENT_SENSE_SAT_A * 0.98f)  // just under saturation

// VMOT sensing: GPIO46 = ADC channel 6, voltage divider 100k / 5.1k
#define PIN_VMOT 46
#define VMOT_DIVIDER_RATIO (105.1f / 5.1f)  // Vmot = Vadc * ratio

// Motor configuration
#if MOTOR_CONFIG == MOTOR_MT6701
#define POLE_PAIRS 11
#elif MOTOR_CONFIG == MOTOR_HALLS
#define POLE_PAIRS 11  // TODO: Set pole pairs for hall sensor motor
#endif

// Encoder power switch (U17, TS5A3159DCK)
// V_SW LOW = V_ENC outputs +3V3 (NC path)
// V_SW HIGH = V_ENC outputs Vdrive (NO path)
#define PIN_V_SW 13

// Encoder SPI (SPI0, MT6701 — auto-detected via CRC)
// GPIO16 = SPI0 RX  = MISO (data from encoder)
// GPIO17 = SPI0 CSn = chip select
// GPIO18 = SPI0 SCK = clock
#define PIN_ENC_MISO 16  // ENC_A_DATA
#define PIN_ENC_CS   17  // ENC_B_DATA
#define PIN_ENC_SCK  18  // ENC_C_DATA

// Encoder differential transceiver direction (SIT3088ETK)
// SW LOW  = receive (differential → MCU)
// SW HIGH = transmit (MCU → differential)
#define PIN_ENC_A_SW 26  // MISO direction: receive
#define PIN_ENC_B_SW 27  // CS direction: transmit
#define PIN_ENC_C_SW 28  // SCK direction: transmit

// Encoder I/O path switches (TS5A3159DCK)
// SW LOW  = NC path (differential transceiver → encoder connector)
// SW HIGH = NO path (hall sensor input)
#define PIN_H1_SW 34  // ENC_A_P routing
#define PIN_H2_SW 35  // ENC_A_N routing
#define PIN_H3_SW 36  // ENC_B_P routing

// Hall sensor pins (active when H*_SW = HIGH, directly to MCU)
#define PIN_HALL_A 31
#define PIN_HALL_B 32
#define PIN_HALL_C 33

// USB PD controller (FUSB302BMPX on I2C1)
#define PIN_PD_SDA  22
#define PIN_PD_SCL  23
#define PIN_PD_INT  39

// Status LED (WS2812 on GPIO11)
#define PIN_LED 11
static Adafruit_NeoPixel led(1, PIN_LED, NEO_GRB + NEO_KHZ800);

static void setLED(uint8_t r, uint8_t g, uint8_t b) {
    led.setPixelColor(0, led.Color(r / 10, g / 10, b / 10));
    led.show();
}

// Status-LED palette. Red boot / green ready / purple PD match the original
// demo; blue (arming) and purple (running) extend it for the self-running demo.
#define LED_BOOT()     setLED(255, 0, 0)    // red:    powered, waiting / not ready
#define LED_PD()       setLED(128, 0, 255)  // purple: USB PD contract negotiated
#define LED_ARMING()   setLED(0, 0, 255)    // blue:   power seen, loading calibration
#define LED_READY()    setLED(0, 255, 0)    // green:  FOC aligned/ready
#define LED_RUNNING()  setLED(128, 0, 255)  // purple: demo sine running
#define LED_ERROR()    setLED(255, 40, 0)   // orange: demo could not start

// Debug serial via UART0 (GPIO0 TX, GPIO1 RX) through debug probe UART bridge.
// Falls back to USB CDC if debug probe not connected.
#define PIN_UART_TX 0
#define PIN_UART_RX 1
#define SERIAL_PORT Serial1

// --- Global objects (pointers, constructed in setup() to avoid static initializers) ---
// RP2350 USB CDC is corrupted by ANY SimpleFOC global constructors running before main().
static BLDCDriver6PWM *driver;
static InlineCurrentSense *current_sense;
static BLDCMotor *motor;
#if MOTOR_CONFIG == MOTOR_MT6701
static MagneticSensorMT6835 *encoder;
#elif MOTOR_CONFIG == MOTOR_HALLS
static HallSensor *hall_sensor;
#endif
static Commander *commander;
static bool enc_detected = false;
static bool foc_ready = false;

#if MOTOR_CONFIG == MOTOR_HALLS
static bool hall_order_determined = false;

void hallA_ISR() { hall_sensor->handleA(); }
void hallB_ISR() { hall_sensor->handleB(); }
void hallC_ISR() { hall_sensor->handleC(); }
#endif

// Loop frequency measurement
static unsigned long loop_count = 0;
static unsigned long loop_freq_t0 = 0;
static float loop_freq_hz = 0;

// --- Automatic demo ---
// 1 = self-starting demo: wait for motor power, load calibration from flash,
// then run a continuous sine with all output suppressed. 0 = manual bring-up
// (H / A / Cs / Y from tune.py). Nothing moves on boot when this is 0.
#define DEMO_MODE 1
#define DEMO_SPEED      50.0f    // rad/s amplitude
#define DEMO_PERIOD_MS  2000.0f  // -> 0.5Hz
#define DEMO_MIN_VMOT   15.0f    // any bench supply above this also arms it

// Blocking per-iteration debug dump (angle/alpha/beta/dq for the first 10
// iterations). Costs ~9ms per line at 115200 and stalls the control loop, so
// it stays off unless you are chasing a commutation problem.
#define STEP_DEBUG 0

// --- Step-test sample log ---
//
// Samples are captured into RAM during the FOC loop and dumped once the loop
// ends. SERIAL_PORT is Serial1 (115200 UART), and print() blocks when the TX
// FIFO fills: a ~28-byte CSV row costs ~2.4ms, so printing per iteration
// throttled loopFOC()/move() to a measured 407Hz -- the control loop ran ~100x
// slower than its gains assume, which oscillates and reads as far too much
// velocity gain. (The original demo printed to USB CDC, a buffered 12Mbit/s
// pipe, which hid this.) Buffering keeps the plots and costs ~100ns per sample.
//
// 2000 x 12 floats + timestamps = ~104KB of the 512KB SRAM.
#define LOG_MAX_SAMPLES 2000
#define LOG_MAX_COLS    12
static uint32_t log_t_ms[LOG_MAX_SAMPLES];
static float    log_col[LOG_MAX_SAMPLES][LOG_MAX_COLS];
static uint8_t  log_ncols = 0;
static uint16_t log_n = 0;
static uint32_t log_last_us = 0;
static uint32_t log_interval_us = 1000;
static const char *log_header = "";

// Pick an interval that fits the whole run in the buffer, no faster than 1kHz.
static void logBegin(const char *header, uint8_t ncols, unsigned long duration_ms) {
    log_header = header;
    log_ncols = ncols < LOG_MAX_COLS ? ncols : LOG_MAX_COLS;
    log_n = 0;
    uint32_t need = (uint32_t)((uint64_t)duration_ms * 1000UL / LOG_MAX_SAMPLES);
    log_interval_us = need > 1000 ? need : 1000;
    log_last_us = micros() - log_interval_us;  // capture the first sample immediately
}

// Returns a row to fill, or nullptr if it isn't time yet / the buffer is full.
static inline float *logSample(uint32_t t_ms) {
    uint32_t now = micros();
    if ((uint32_t)(now - log_last_us) < log_interval_us) return nullptr;
    if (log_n >= LOG_MAX_SAMPLES) return nullptr;
    log_last_us = now;
    log_t_ms[log_n] = t_ms;
    return log_col[log_n++];
}

// Dump after the control loop has stopped, where blocking is harmless.
static void logDump() {
    SERIAL_PORT.println(log_header);
    for (uint16_t i = 0; i < log_n; i++) {
        SERIAL_PORT.print(log_t_ms[i]);
        for (uint8_t c = 0; c < log_ncols; c++) {
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(log_col[i][c], 4);
        }
        SERIAL_PORT.println();
    }
}

// FOC loop rate limit. Left ungated, loop() spins loopFOC()/move() as fast as
// the core allows (measured ~208kHz), which oversamples the encoder and feeds
// a noisy differentiated velocity into PID_velocity.
#define FOC_LOOP_HZ 40000
#define FOC_LOOP_PERIOD_US (1000000UL / FOC_LOOP_HZ)
static uint32_t foc_last_us = 0;
static float foc_rate_hz = 0;    // measured, reported by R
static uint32_t foc_run_count = 0;

// True once per FOC_LOOP_PERIOD_US. Unsigned subtraction handles micros() rollover.
static inline bool focLoopDue() {
    uint32_t now = micros();
    if ((uint32_t)(now - foc_last_us) < FOC_LOOP_PERIOD_US) return false;
    foc_last_us = now;
    foc_run_count++;
    return true;
}

// Continuous velocity sine mode (non-blocking, runs in loop)
static bool sine_running = false;
static float sine_amplitude = 0;
static float sine_freq_hz = 1.0f;
static unsigned long sine_t0 = 0;

#ifdef HAS_USB_PD
#include <PD_UFP.h>
static PD_UFP_Log_c *pd_ufp;
static bool pd_ready = false;
// get_voltage() returns 50mV units, get_current() returns 10mA units
static uint16_t pd_voltage_raw = 0;
static uint16_t pd_current_raw = 0;
#endif

// --- VMOT reading ---
// GPIO46 = ADC channel 6 on RP2350B (base GPIO 40)
#define VMOT_ADC_CHAN (PIN_VMOT - 40)
static float readVMOT() {
    RP2040ADCEngine *eng = getADCEngine();
    if (eng->initialized && eng->channelsEnabled[VMOT_ADC_CHAN]) {
        uint16_t raw = eng->getRawChannel(VMOT_ADC_CHAN);
        return raw * eng->adc_conv * VMOT_DIVIDER_RATIO;
    }
    // Before DMA engine init, use one-shot analogRead (10-bit default on arduino-pico)
    return analogRead(PIN_VMOT) * (3.3f / 1024.0f) * VMOT_DIVIDER_RATIO;
}

// Forward declarations
void stopSine();
void startSine(float amplitude, float freq);
void doHallScan(char *cmd);

// Serial output is blocked while the sine runs: SERIAL_PORT is a 115200 UART and
// every reply stalls loopFOC(). "VMOT=19.86" is ~1ms, about 16 lost FOC
// iterations, and tune.py polls V once a second; an R report is ~200 bytes,
// ~17ms -- which is what would make a "continuous" sine stutter. Commands are
// still PARSED while blocked -- only the reply is dropped -- so tune.py's Stop
// button (T0 -> doTarget -> stopSine) still works.
static inline bool outputBlocked() { return sine_running; }

// Continuous sine demo: Y<amplitude>[,<freq_hz>] starts it, Y alone stops it.
// This is the only way to reach startSine() -- the PD 20V/5A auto-start in
// loop() is commented out, so without this the demo path is unreachable.
void doDemo(char *cmd) {
    if (cmd[0] == '\0' || cmd[0] == '\n' || cmd[0] == '\r') {
        stopSine();
        return;
    }
    float amplitude = atof(cmd);
    float freq = 1.0f;
    char *comma = strchr(cmd, ',');
    if (comma) freq = atof(comma + 1);
    startSine(amplitude, freq);
}
void doCSDebug(char *cmd);

// --- Commander callbacks ---
static bool hw_initialized = false;
void doVmot(char *cmd) {
    if (outputBlocked()) return;  // tune.py polls this once a second
    SERIAL_PORT.print("VMOT=");
    SERIAL_PORT.println(readVMOT(), 2);
}
// Hard ceiling on current_limit, enforced everywhere it can be set. Above
// CURRENT_LIMIT_MAX_A the sense chain saturates and the current loop runs open
// with no feedback. updateCurrentLimit() is used rather than assigning the field
// so PID_velocity.limit (which bounds the velocity loop's current setpoint)
// tracks it -- assigning current_limit alone would leave the PID uncapped.
static void clampCurrentLimit() {
    if (motor->current_limit > CURRENT_LIMIT_MAX_A)
        motor->updateCurrentLimit(CURRENT_LIMIT_MAX_A);
    if (motor->PID_velocity.limit > CURRENT_LIMIT_MAX_A)
        motor->PID_velocity.limit = CURRENT_LIMIT_MAX_A;
}

// MLC from the dashboard calls updateCurrentLimit() with whatever is typed, so
// re-clamp after every motor command.
void doMotor(char *cmd) {
    commander->motor(motor, cmd);
    clampCurrentLimit();
}
void doPolePairs(char *cmd) {
    if (cmd[0] == '\0' || cmd[0] == '\n' || cmd[0] == '\r') {
        // Read
        SERIAL_PORT.print("PP=");
        SERIAL_PORT.println(motor->pole_pairs);
    } else {
        int pp = atoi(cmd);
        if (pp >= 1 && pp <= 50) {
            motor->pole_pairs = pp;
            SERIAL_PORT.print("PP=");
            SERIAL_PORT.println(motor->pole_pairs);
            SERIAL_PORT.println("Re-run Align (A) for this to take effect.");
        } else {
            SERIAL_PORT.println("ERR: pole pairs must be 1-50");
        }
    }
}
void doTarget(char *cmd) {
    if (sine_running) stopSine();
    commander->scalar(&motor->target, cmd);
}

// Run motor continuously: Bv<target> = velocity, Bt<target> = torque, B = stop, Bx = coast
void doRun(char *cmd) {
    if (sine_running) stopSine();

    // B alone = stop (active brake: target=0, FOC keeps running)
    if (cmd[0] == '\0' || cmd[0] == '\n' || cmd[0] == '\r') {
        motor->target = 0;
        SERIAL_PORT.println("Motor stopped (braking).");
        return;
    }

    // Bx = coast (disable driver, motor spins freely)
    if (cmd[0] == 'x' || cmd[0] == 'X') {
        motor->target = 0;
        motor->disable();
        SERIAL_PORT.println("Motor coasting.");
        return;
    }

    if (!foc_ready) {
        SERIAL_PORT.println("ERR: Not aligned. Run 'A' first.");
        return;
    }

    char mode = cmd[0];
    float target = atof(&cmd[1]);

    // Reset PID/LPF state
    motor->PID_current_q.reset();
    motor->PID_current_d.reset();
    motor->PID_velocity.reset();
    motor->LPF_current_q.y_prev = 0;
    motor->LPF_current_d.y_prev = 0;
    motor->LPF_velocity.y_prev = 0;
    motor->current_sp = 0;
    motor->feed_forward_current = {0, 0};
    motor->feed_forward_voltage = {0, 0};

    if (mode == 'v' || mode == 'V') {
        motor->controller = MotionControlType::velocity;
    } else if (mode == 't' || mode == 'T') {
        motor->controller = MotionControlType::torque;
    } else {
        SERIAL_PORT.println("Usage: Bv<rad/s>, Bt<amps>, B (stop)");
        return;
    }

    motor->target = target;
    motor->enable();
    float vn = driver->voltage_power_supply / 2.0f;
    driver->setPwm(vn, vn, vn);

    const char *mode_name = (mode == 'v' || mode == 'V') ? "velocity" : "torque";
    SERIAL_PORT.print("Running ");
    SERIAL_PORT.print(mode_name);
    SERIAL_PORT.print(": target=");
    SERIAL_PORT.println(target, 2);
}

// Motor tuning defaults. Applied in setup() as well as initHardware() so a
// query (MQP, MVI, ...) reports the value we actually intend before the
// hardware is up — otherwise the controller answers with SimpleFOC's library
// defaults (curr I=300, vel I=10, ramp=NOT_SET), and tune.py's read_params
// faithfully loads those into the dashboard on page load.
// Keep in sync with the input `value` attributes in tune.py.
static void applyMotorTuning() {
#if MOTOR_CONFIG == MOTOR_MT6701
    motor->voltage_limit = 8.0;
    motor->voltage_sensor_align = 1.0;  // reduced from 2.0: 20mOhm shunts saturate INA240 at ~4A
    // 3.9A: bench-validated to move the motor while staying under the 4.04A
    // clamp ceiling and the 4.12A the 20mOhm shunts can measure (1A was too
    // low to turn it).
    motor->current_limit = 3.9;
    motor->velocity_limit = 50.0;
    motor->controller = MotionControlType::velocity;
    motor->torque_controller = TorqueControlType::foc_current;
    motor->PID_current_q.P = 0.6;
    motor->PID_current_q.I = 0.3;
    motor->PID_current_d.P = 0.6;
    motor->PID_current_d.I = 0.3;
    motor->LPF_current_q.Tf = 0.02;
    motor->LPF_current_d.Tf = 0.02;
    motor->PID_velocity.P = 0.3;
    motor->PID_velocity.I = 0.1;
    motor->PID_velocity.D = 0.0;
    motor->PID_velocity.output_ramp = 200.0;
    motor->LPF_velocity.Tf = 0.01;
#elif MOTOR_CONFIG == MOTOR_HALLS
    motor->voltage_limit = 4.0;
    motor->voltage_sensor_align = 2.5;  // 10mΩ shunts: ±8A range, comfortable margin
    motor->current_limit = 3.0;
    motor->velocity_limit = 10.0;
    motor->controller = MotionControlType::velocity;
    motor->torque_controller = TorqueControlType::foc_current;
    // Current loop
    motor->PID_current_q.P = 1.5;
    motor->PID_current_q.I = 0.1;
    motor->PID_current_d.P = 1.5;
    motor->PID_current_d.I = 0.1;
    motor->LPF_current_q.Tf = 0.01;
    motor->LPF_current_d.Tf = 0.01;
    // Velocity loop
    motor->PID_velocity.P = 1.0;
    motor->PID_velocity.I = 0.1;
    motor->PID_velocity.D = 0.0;
    motor->PID_velocity.output_ramp = 200.0;
    motor->LPF_velocity.Tf = 0.1;
#endif
    clampCurrentLimit();  // backstop: never configure past the sense range
}

static void initHardware() {
    if (hw_initialized) return;

    SERIAL_PORT.println("GPIO init...");
    pinMode(PIN_V_SW, OUTPUT);
    pinMode(PIN_ENC_A_SW, OUTPUT);
    pinMode(PIN_ENC_B_SW, OUTPUT);
    pinMode(PIN_ENC_C_SW, OUTPUT);
    pinMode(PIN_H1_SW, OUTPUT);
    pinMode(PIN_H2_SW, OUTPUT);
    pinMode(PIN_H3_SW, OUTPUT);
#if MOTOR_CONFIG == MOTOR_MT6701
    // Encoder 3.3V supply
    digitalWrite(PIN_V_SW, LOW);
    // Differential transceiver directions: MISO=receive, CS/SCK=transmit
    digitalWrite(PIN_ENC_A_SW, LOW);
    digitalWrite(PIN_ENC_B_SW, HIGH);
    digitalWrite(PIN_ENC_C_SW, HIGH);
    // Route to differential transceiver (NC path)
    digitalWrite(PIN_H1_SW, LOW);
    digitalWrite(PIN_H2_SW, LOW);
    digitalWrite(PIN_H3_SW, LOW);
#elif MOTOR_CONFIG == MOTOR_HALLS
    // Hall sensor Vdrive supply
    digitalWrite(PIN_V_SW, HIGH);
    // Transceivers unused for halls, all receive (safe default)
    digitalWrite(PIN_ENC_A_SW, LOW);
    digitalWrite(PIN_ENC_B_SW, LOW);
    digitalWrite(PIN_ENC_C_SW, LOW);
    // Route to hall sensor inputs (NO path)
    digitalWrite(PIN_H1_SW, HIGH);
    digitalWrite(PIN_H2_SW, HIGH);
    digitalWrite(PIN_H3_SW, HIGH);
#endif

#if MOTOR_CONFIG == MOTOR_MT6701
    // Encoder SPI
    SERIAL_PORT.println("SPI init...");
    SPI.setRX(PIN_ENC_MISO);
    SPI.setSCK(PIN_ENC_SCK);
    encoder->init(&SPI);
    delay(10);
    uint32_t raw1 = encoder->readRawAngle21();
    delay(1);
    uint32_t raw2 = encoder->readRawAngle21();
    if (raw1 == 0x1FFFFF && raw2 == 0x1FFFFF) {
        SERIAL_PORT.println("ERR: Encoder not detected (SPI all 1s)");
        enc_detected = false;
    } else {
        enc_detected = true;
        SERIAL_PORT.print("Encoder OK (raw=");
        SERIAL_PORT.print(raw1);
        SERIAL_PORT.println(")");
    }
#elif MOTOR_CONFIG == MOTOR_HALLS
    SERIAL_PORT.println("Hall sensor init...");
    hall_sensor->init();
    hall_sensor->enableInterrupts(hallA_ISR, hallB_ISR, hallC_ISR);
    enc_detected = true;
    SERIAL_PORT.print("Hall sensors OK (state=");
    SERIAL_PORT.print(hall_sensor->electric_sector);
    SERIAL_PORT.println(")");
#endif

    // Driver
    SERIAL_PORT.println("Driver init...");
    driver->pwm_frequency = 20000;
    driver->dead_zone = 0.02;
    driver->voltage_power_supply = 30.0f;  // initial estimate, updated from VMOT ADC after init
    driver->init();

    // Current sense + VMOT ADC channel
    SERIAL_PORT.println("Current sense init...");
    current_sense->linkDriver(driver);
    getADCEngine()->addPin(PIN_VMOT);  // register VMOT before engine->init() runs
    // Enable phase state so calibrateOffsets() inside init() actually sees
    // 50% duty switching (not the static braking state from driver->init()).
    // Without this, _writeDutyCycle6PWM ignores the setPwm(Vs/2,...) call
    // because phase_state is still PHASE_OFF.
    driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
    current_sense->init();
    driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF);
    driver->setPwm(0, 0, 0);

    // Update driver voltage from live VMOT reading
    delay(1);  // allow at least one DMA cycle to complete
    driver->voltage_power_supply = readVMOT();
    SERIAL_PORT.print("VMOT=");
    SERIAL_PORT.print(driver->voltage_power_supply, 1);
    SERIAL_PORT.println("V");

    // Motor config
    motor->linkDriver(driver);
    motor->linkCurrentSense(current_sense);
#if MOTOR_CONFIG == MOTOR_MT6701
    if (enc_detected) motor->linkSensor(encoder);
#elif MOTOR_CONFIG == MOTOR_HALLS
    if (enc_detected) motor->linkSensor(hall_sensor);
#endif
    applyMotorTuning();
    motor->useMonitoring(SERIAL_PORT);
    motor->monitor_downsample = 0;

    if (enc_detected) {
        SERIAL_PORT.println("Motor init...");
        motor->init();
        motor->disable();
    }

    hw_initialized = true;
    SERIAL_PORT.println("Hardware ready.");
}

void doHwInit(char *cmd) {
    initHardware();
    SERIAL_PORT.print("encoder=");
    SERIAL_PORT.println(enc_detected ? "OK" : "NOT DETECTED");
}

void doAlign(char *cmd) {
    initHardware();

    if (!enc_detected) {
        SERIAL_PORT.println("ERR: Sensor not detected — cannot align.");
        return;
    }

#if MOTOR_CONFIG == MOTOR_HALLS
    if (!hall_order_determined) {
        SERIAL_PORT.println("First align — running hall auto-scan...");
        doHallScan(cmd);
        if (!hall_order_determined) {
            SERIAL_PORT.println("ERR: Hall scan failed — cannot align.");
            return;
        }
    }
#endif

    // Force full re-calibration every time (don't reuse stale values)
    motor->sensor_direction = Direction::UNKNOWN;
    motor->zero_electric_angle = NOT_SET;

    SERIAL_PORT.print("align_voltage=");
    SERIAL_PORT.println(motor->voltage_sensor_align, 2);
    SERIAL_PORT.println("Aligning...");

#if MOTOR_CONFIG == MOTOR_HALLS
    // Nudge motor away from its current position before initFOC.
    // After the hall scan (or cold boot), the motor may be sitting at the
    // exact angle initFOC tests first, so the halls see no transitions
    // → "Failed to notice movement" → bad zero_electric_angle
    // → driverAlign applies voltage at wrong angles → "all currents same magnitude".
    {
        float v = motor->voltage_sensor_align;
        float nudge = _2PI / 3.0f;  // 120° electrical — 2 hall sectors away
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
        float na = v * cosf(nudge);
        float nb = v * cosf(nudge - _2PI / 3.0f);
        float nc = v * cosf(nudge + _2PI / 3.0f);
        driver->setPwm((na + v) * 0.5f, (nb + v) * 0.5f, (nc + v) * 0.5f);
        delay(700);
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF);
        delay(200);
        SERIAL_PORT.println("Motor nudged to offset position.");
    }
#endif

    motor->enable();
    int result = motor->initFOC();

    // Print alignment diagnostics
    SERIAL_PORT.println("--- Alignment Results ---");
    SERIAL_PORT.print("initFOC result: ");
    SERIAL_PORT.println(result);
    SERIAL_PORT.print("sensor_direction: ");
    SERIAL_PORT.println(motor->sensor_direction == Direction::CW ? "CW" : "CCW");
    SERIAL_PORT.print("zero_electric_angle: ");
    SERIAL_PORT.println(motor->zero_electric_angle, 4);
    SERIAL_PORT.print("pp_check: ");
    SERIAL_PORT.println(motor->pp_check_result ? "OK" : "FAIL");
    SERIAL_PORT.print("pole_pairs: ");
    SERIAL_PORT.println(motor->pole_pairs);

    // Current sense alignment results
    SERIAL_PORT.print("CS gain_a: ");
    SERIAL_PORT.println(current_sense->gain_a, 5);
    SERIAL_PORT.print("CS gain_b: ");
    SERIAL_PORT.println(current_sense->gain_b, 5);
    SERIAL_PORT.print("CS gain_c: ");
    SERIAL_PORT.println(current_sense->gain_c, 5);
    SERIAL_PORT.print("CS offset_ia: ");
    SERIAL_PORT.println(current_sense->offset_ia, 4);
    SERIAL_PORT.print("CS offset_ib: ");
    SERIAL_PORT.println(current_sense->offset_ib, 4);
    SERIAL_PORT.print("CS offset_ic: ");
    SERIAL_PORT.println(current_sense->offset_ic, 4);

    // Read current at rest after alignment
#if MOTOR_CONFIG == MOTOR_MT6701
    encoder->update();
#elif MOTOR_CONFIG == MOTOR_HALLS
    hall_sensor->update();
#endif
    float elec_angle = motor->electricalAngle();
    SERIAL_PORT.print("electrical_angle: ");
    SERIAL_PORT.println(elec_angle, 4);
    PhaseCurrent_s phase = current_sense->getPhaseCurrents();
    SERIAL_PORT.print("phase_currents (rest): a=");
    SERIAL_PORT.print(phase.a, 4);
    SERIAL_PORT.print(" b=");
    SERIAL_PORT.print(phase.b, 4);
    SERIAL_PORT.print(" c=");
    SERIAL_PORT.println(phase.c, 4);
    DQCurrent_s dq = current_sense->getFOCCurrents(elec_angle);
    SERIAL_PORT.print("DQ_currents (rest): Iq=");
    SERIAL_PORT.print(dq.q, 4);
    SERIAL_PORT.print(" Id=");
    SERIAL_PORT.println(dq.d, 4);
    SERIAL_PORT.println("--- End Alignment ---");

    if (result) {
        foc_ready = true;
        motor->disable();
        LED_READY();  // green when initialized
        SERIAL_PORT.println("Aligned. Motor disabled until step/run command.");
    } else {
        foc_ready = false;
        motor->disable();
        SERIAL_PORT.println("ERR: Alignment failed!");
    }
}

// Step response test: Sq<val>, Sv<val>, Sp<val>
void doStep(char *cmd) {
    if (cmd[0] == '\0') {
        SERIAL_PORT.println("Usage: Si<A>, Sq<A>, Sv<rad/s>, Sp<rad>, Sw<rad/s>[,<period_ms>]");
        return;
    }
    if (!foc_ready) {
        SERIAL_PORT.println("ERR: Not aligned. Run 'A' first.");
        SERIAL_PORT.println("DONE");
        return;
    }
    char mode = cmd[0];
    float value = atof(&cmd[1]);

    // --- Fixed-angle current impulse test (no commutation) ---
    if (mode == 'i' || mode == 'I') {
        SERIAL_PORT.println("t_ms,Iq_target,Iq,Id,Vq,Vd,Ia,Ib,Ic,raw0,raw1,raw2");

        // Reset current PIDs and LPFs
        motor->PID_current_q.reset();
        motor->PID_current_d.reset();
        motor->LPF_current_q.y_prev = 0;
        motor->LPF_current_d.y_prev = 0;

        // Freeze electrical angle at current rotor position
#if MOTOR_CONFIG == MOTOR_MT6701
        encoder->update();
#elif MOTOR_CONFIG == MOTOR_HALLS
        hall_sensor->update();
#endif
        float angle = motor->electricalAngle();

        // Enable driver, set neutral PWM
        motor->enable();
        float vn = driver->voltage_power_supply / 2.0f;
        driver->setPwm(vn, vn, vn);

        // Pre-load LPF with actual idle readings to avoid cold-start transient
        DQCurrent_s idle = current_sense->getFOCCurrents(angle);
        motor->LPF_current_q.y_prev = idle.q;
        motor->LPF_current_d.y_prev = idle.d;

        // Duration: 100ms baseline + 500ms step + 100ms recovery
        unsigned long step_on = 100;
        unsigned long step_off = 600;
        unsigned long duration = 700;
        float target_i = 0;

        RP2040ADCEngine *eng = getADCEngine();
        unsigned long t0 = millis();
        while (millis() - t0 < duration) {
            unsigned long t_ms = millis() - t0;

            // Step on/off
            if (t_ms >= step_on && t_ms < step_off)
                target_i = value;
            else
                target_i = 0;

            // Current control at fixed angle (no commutation)
            DQCurrent_s i = current_sense->getFOCCurrents(angle);
            float iq_filt = motor->LPF_current_q(i.q);
            float id_filt = motor->LPF_current_d(i.d);
            float Vq = motor->PID_current_q(target_i - iq_filt);
            float Vd = motor->PID_current_d(0 - id_filt);
            motor->setPhaseVoltage(Vq, Vd, angle);

            // Log
            PhaseCurrent_s phase = current_sense->getPhaseCurrents();
            SERIAL_PORT.print(t_ms);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(target_i, 4);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(iq_filt, 4);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(id_filt, 4);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(Vq, 4);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(Vd, 4);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(phase.a, 4);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(phase.b, 4);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(phase.c, 4);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(eng->getRawChannel(0));
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(eng->getRawChannel(1));
            SERIAL_PORT.print(',');
            SERIAL_PORT.println(eng->getRawChannel(2));
        }

        // Disable
        motor->setPhaseVoltage(0, 0, angle);
        motor->disable();
        SERIAL_PORT.println("DONE");
        return;
    }

    // --- Sinusoidal velocity tracking test ---
    if (mode == 'w' || mode == 'W') {
        // Reset PID/LPF state
        motor->PID_current_q.reset();
        motor->PID_current_d.reset();
        motor->PID_velocity.reset();
        motor->LPF_current_q.y_prev = 0;
        motor->LPF_current_d.y_prev = 0;
        motor->LPF_velocity.y_prev = 0;
        motor->current_sp = 0;
        motor->feed_forward_current = {0, 0};
        motor->feed_forward_voltage = {0, 0};

        motor->controller = MotionControlType::velocity;
        motor->enable();
        float vn = driver->voltage_power_supply / 2.0f;
        driver->setPwm(vn, vn, vn);

        // Parse optional period: Sw<amplitude>,<period_ms>  (default 1000ms = 1Hz)
        float amplitude = value;  // max velocity in rad/s
        float period_ms = 1000.0f;
        char *comma = strchr(&cmd[1], ',');
        if (comma) period_ms = atof(comma + 1);
        if (period_ms < 50) period_ms = 50;  // clamp minimum
        float freq_hz = 1000.0f / period_ms;
        unsigned long duration = (unsigned long)(period_ms * 3);  // 3 full cycles

        motor->target = 0;
        uint32_t foc0 = foc_run_count;
        logBegin("t_ms,vel_target,vel,Iq", 3, duration);
        unsigned long t0 = millis();
        while (millis() - t0 < duration) {
            if (!focLoopDue()) continue;

            unsigned long t_ms = millis() - t0;
            float t_sec = t_ms * 0.001f;
            motor->target = amplitude * sinf(2.0f * 3.14159265f * freq_hz * t_sec);

            motor->loopFOC();
            motor->move();

            float *s = logSample(t_ms);
            if (s) {
                s[0] = motor->target;
                s[1] = motor->shaft_velocity;
                s[2] = motor->current.q;
            }
        }
        unsigned long elapsed = millis() - t0;
        uint32_t iters = foc_run_count - foc0;

        motor->target = 0;
        motor->disable();
        logDump();
        // Achieved FOC rate, printed once after the loop so it costs nothing.
        SERIAL_PORT.print("foc_iters=");
        SERIAL_PORT.print(iters);
        SERIAL_PORT.print(" in ");
        SERIAL_PORT.print(elapsed);
        SERIAL_PORT.print("ms -> ");
        SERIAL_PORT.print(elapsed ? (iters * 1000.0f / elapsed) : 0, 0);
        SERIAL_PORT.println(" Hz");
        SERIAL_PORT.println("DONE");
        return;
    }

    // --- Standard commutated step tests (q/v/p) ---

    // Save current state
    MotionControlType prev_controller = motor->controller;

    // Configure mode for the test. Headers are emitted by logDump() afterwards.
    const char *hdr;
    uint8_t ncols;
    switch (mode) {
        case 'q': case 'Q':
            motor->controller = MotionControlType::torque;
            hdr = "t_ms,Iq_target,Iq,Id,Vq,Vd,Ia,Ib,Ic,raw0,raw1,raw2";
            ncols = 11;
            break;
        case 'v': case 'V':
            motor->controller = MotionControlType::velocity;
            hdr = "t_ms,vel_target,vel,Iq";
            ncols = 3;
            break;
        case 'p': case 'P':
            motor->controller = MotionControlType::angle;
            hdr = "t_ms,angle_target,angle,vel";
            ncols = 3;
            break;
        default:
            SERIAL_PORT.println("Unknown mode. Use i, q, v, or p.");
            return;
    }

    // Reset PID/LPF state — clears any NaN/Inf from previous instability
    motor->PID_current_q.reset();
    motor->PID_current_d.reset();
    motor->PID_velocity.reset();
    motor->LPF_current_q.y_prev = 0;
    motor->LPF_current_d.y_prev = 0;
    motor->LPF_velocity.y_prev = 0;
    motor->current_sp = 0;
    // Zero feed-forward terms (SimpleFOC doesn't initialize these structs!)
    motor->feed_forward_current = {0, 0};
    motor->feed_forward_voltage = {0, 0};
    SERIAL_PORT.print("DBG ff_cur.q=");
    SERIAL_PORT.print(motor->feed_forward_current.q, 6);
    SERIAL_PORT.print(" ff_vol.q=");
    SERIAL_PORT.println(motor->feed_forward_voltage.q, 6);

    // Duration: 100ms baseline + step + 100ms recovery tail
    unsigned long step_on = 100;
    unsigned long step_off = (mode == 'q' || mode == 'Q') ? 600 : 2100;
    unsigned long duration = step_off + 100;

    // Enable motor for the duration of the test
    motor->enable();
    // Immediately drive to neutral 50% duty to avoid the brake→switching
    // transient. enable() sets dc=0 which with active-low low-side means
    // all low-sides ON (braking to GND). First loopFOC would jump to 50%
    // causing a 15V step and audible click. Setting neutral here eliminates that.
    float vn = driver->voltage_power_supply / 2.0f;
    driver->setPwm(vn, vn, vn);

    // Record 100ms of baseline at target=0, then step to value
    motor->target = 0;
    int iter_count = 0;
    uint32_t foc0 = foc_run_count;
    logBegin(hdr, ncols, duration);
    unsigned long t0 = millis();
    while (millis() - t0 < duration) {
        if (!focLoopDue()) continue;

        // Instrument first 10 iterations: print angle, alpha/beta, dq before PID
        if (STEP_DEBUG && iter_count < 10 && (mode == 'q' || mode == 'Q')) {
#if MOTOR_CONFIG == MOTOR_MT6701
            encoder->update();
#elif MOTOR_CONFIG == MOTOR_HALLS
            hall_sensor->update();
#endif
            float el_angle = motor->electricalAngle();
            PhaseCurrent_s raw_phase = current_sense->getPhaseCurrents();
            ABCurrent_s ab = current_sense->getABCurrents(raw_phase);
            DQCurrent_s dq = current_sense->getDQCurrents(ab, el_angle);
            SERIAL_PORT.print("DBG[");
            SERIAL_PORT.print(iter_count);
            SERIAL_PORT.print("] angle_el=");
            SERIAL_PORT.print(el_angle, 4);
            SERIAL_PORT.print(" Ia=");
            SERIAL_PORT.print(raw_phase.a, 4);
            SERIAL_PORT.print(" Ib=");
            SERIAL_PORT.print(raw_phase.b, 4);
            SERIAL_PORT.print(" Ic=");
            SERIAL_PORT.print(raw_phase.c, 4);
            SERIAL_PORT.print(" alpha=");
            SERIAL_PORT.print(ab.alpha, 4);
            SERIAL_PORT.print(" beta=");
            SERIAL_PORT.print(ab.beta, 4);
            SERIAL_PORT.print(" Iq=");
            SERIAL_PORT.print(dq.q, 4);
            SERIAL_PORT.print(" Id=");
            SERIAL_PORT.println(dq.d, 4);
        }
        iter_count++;

        motor->loopFOC();
        motor->move();
        unsigned long t_ms = millis() - t0;

        // Step on at t=100ms, step off at end for recovery tail
        if (t_ms >= step_on && t_ms < step_off)
            motor->target = value;
        else
            motor->target = 0;

        float *s = logSample(t_ms);
        if (!s) continue;

        switch (mode) {
            case 'q': case 'Q': {
                PhaseCurrent_s phase = current_sense->getPhaseCurrents();
                RP2040ADCEngine *eng = getADCEngine();
                s[0] = motor->target;
                s[1] = motor->current.q;
                s[2] = motor->current.d;
                s[3] = motor->voltage.q;
                s[4] = motor->voltage.d;
                s[5] = phase.a;
                s[6] = phase.b;
                s[7] = phase.c;
                s[8] = eng->getRawChannel(0);
                s[9] = eng->getRawChannel(1);
                s[10] = eng->getRawChannel(2);
                break;
            }
            case 'v': case 'V':
                s[0] = motor->target;
                s[1] = motor->shaft_velocity;
                s[2] = motor->current.q;
                break;
            case 'p': case 'P':
                s[0] = motor->target;
                s[1] = motor->shaft_angle;
                s[2] = motor->shaft_velocity;
                break;
        }
    }

    unsigned long elapsed = millis() - t0;
    uint32_t iters = foc_run_count - foc0;

    // Restore previous state and disable motor
    motor->target = 0;
    motor->controller = prev_controller;
    motor->disable();
    logDump();
    SERIAL_PORT.print("foc_iters=");
    SERIAL_PORT.print(iters);
    SERIAL_PORT.print(" in ");
    SERIAL_PORT.print(elapsed);
    SERIAL_PORT.print("ms -> ");
    SERIAL_PORT.print(elapsed ? (iters * 1000.0f / elapsed) : 0, 0);
    SERIAL_PORT.println(" Hz");
    SERIAL_PORT.println("DONE");
}

// Report current motor state
void doReport(char *cmd) {
    if (outputBlocked()) return;  // ~200 bytes, ~17ms of stalled FOC
    SERIAL_PORT.print("loop_hz=");
    SERIAL_PORT.print(loop_freq_hz, 0);
    SERIAL_PORT.print(" foc_hz=");
    SERIAL_PORT.print(foc_rate_hz, 0);
    SERIAL_PORT.print(" enabled=");
    SERIAL_PORT.print(motor->enabled);
    SERIAL_PORT.print(" mode=");
    switch (motor->controller) {
        case MotionControlType::torque:     SERIAL_PORT.print("torque"); break;
        case MotionControlType::velocity:   SERIAL_PORT.print("velocity"); break;
        case MotionControlType::angle:      SERIAL_PORT.print("angle"); break;
        case MotionControlType::velocity_openloop: SERIAL_PORT.print("vel_OL"); break;
        case MotionControlType::angle_openloop:    SERIAL_PORT.print("ang_OL"); break;
    }
    SERIAL_PORT.print(" vel=");
    SERIAL_PORT.print(motor->shaft_velocity, 3);
    SERIAL_PORT.print(" Iq=");
    SERIAL_PORT.print(motor->current.q, 3);
    SERIAL_PORT.print(" Id=");
    SERIAL_PORT.print(motor->current.d, 3);
    SERIAL_PORT.print(" angle=");
    SERIAL_PORT.print(motor->shaft_angle, 3);
    SERIAL_PORT.print(" target=");
    SERIAL_PORT.print(motor->target, 3);

    // ADC diagnostics: raw values and current sense offsets
    PhaseCurrent_s phase = current_sense->getPhaseCurrents();
    SERIAL_PORT.print(" Ia=");
    SERIAL_PORT.print(phase.a, 4);
    SERIAL_PORT.print(" Ib=");
    SERIAL_PORT.print(phase.b, 4);
    SERIAL_PORT.print(" Ic=");
    SERIAL_PORT.print(phase.c, 4);
    SERIAL_PORT.print(" off_a=");
    SERIAL_PORT.print(current_sense->offset_ia, 4);
    SERIAL_PORT.print(" off_b=");
    SERIAL_PORT.print(current_sense->offset_ib, 4);
    SERIAL_PORT.print(" off_c=");
    SERIAL_PORT.print(current_sense->offset_ic, 4);
    SERIAL_PORT.print(" vmot=");
    SERIAL_PORT.print(readVMOT(), 2);
    // Raw ADC channels for debugging DMA mapping
    RP2040ADCEngine *eng = getADCEngine();
    SERIAL_PORT.print(" raw0=");
    SERIAL_PORT.print(eng->getRawChannel(0));
    SERIAL_PORT.print(" raw1=");
    SERIAL_PORT.print(eng->getRawChannel(1));
    SERIAL_PORT.print(" raw2=");
    SERIAL_PORT.print(eng->getRawChannel(2));
    SERIAL_PORT.print(" raw6=");
    SERIAL_PORT.print(eng->getRawChannel(6));
#if MOTOR_CONFIG == MOTOR_MT6701
    SERIAL_PORT.print(" enc=");
    switch(encoder->chip_type) {
        case CHIP_MT6835: SERIAL_PORT.print("MT6835"); break;
        case CHIP_MT6701: SERIAL_PORT.print("MT6701"); break;
        default: SERIAL_PORT.print("unknown"); break;
    }
    SERIAL_PORT.print(" crc_err=");
    SERIAL_PORT.print(encoder->crc_errors);
    SERIAL_PORT.print("/");
    SERIAL_PORT.print(encoder->read_count);
    SERIAL_PORT.print(" enc_status=0x");
    SERIAL_PORT.println(encoder->last_status, HEX);
#elif MOTOR_CONFIG == MOTOR_HALLS
    SERIAL_PORT.print(" sensor=halls");
    SERIAL_PORT.print(" hall_state=");
    SERIAL_PORT.println(hall_sensor->electric_sector);
#endif
}

// ADC diagnostic: compare raw readings in disabled vs switching states
void doAdcTest(char *cmd) {
    if (!hw_initialized) {
        SERIAL_PORT.println("ERR: Run H first");
        return;
    }
    RP2040ADCEngine *eng = getADCEngine();

    // State 1: Motor disabled (current state after align)
    motor->disable();
    delay(5);
    SERIAL_PORT.print("DISABLED  raw0=");
    SERIAL_PORT.print(eng->getRawChannel(0));
    SERIAL_PORT.print(" raw1=");
    SERIAL_PORT.print(eng->getRawChannel(1));
    SERIAL_PORT.print(" raw2=");
    SERIAL_PORT.println(eng->getRawChannel(2));

    // State 2: Motor enabled, force neutral 50% duty (no FOC)
    driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
    float vn = driver->voltage_power_supply / 2.0f;
    driver->setPwm(vn, vn, vn);
    delay(5);  // let switching settle
    SERIAL_PORT.print("SWITCHING raw0=");
    SERIAL_PORT.print(eng->getRawChannel(0));
    SERIAL_PORT.print(" raw1=");
    SERIAL_PORT.print(eng->getRawChannel(1));
    SERIAL_PORT.print(" raw2=");
    SERIAL_PORT.println(eng->getRawChannel(2));

    // State 3: Still enabled, dc=0 (low-sides on, braking)
    driver->setPwm(0, 0, 0);
    delay(5);
    SERIAL_PORT.print("BRAKE     raw0=");
    SERIAL_PORT.print(eng->getRawChannel(0));
    SERIAL_PORT.print(" raw1=");
    SERIAL_PORT.print(eng->getRawChannel(1));
    SERIAL_PORT.print(" raw2=");
    SERIAL_PORT.println(eng->getRawChannel(2));

    // Restore disabled state
    driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF);
    driver->setPwm(0, 0, 0);

    SERIAL_PORT.print("Offsets: a=");
    SERIAL_PORT.print(current_sense->offset_ia, 4);
    SERIAL_PORT.print(" b=");
    SERIAL_PORT.print(current_sense->offset_ib, 4);
    SERIAL_PORT.print(" c=");
    SERIAL_PORT.println(current_sense->offset_ic, 4);
}

// --- Calibration save/load (LittleFS on SPI flash) ---

// Mount LittleFS, reporting the result. begin() returns false when the
// filesystem will not mount -- unformatted flash, or metadata left half-written
// by a power cut mid-save. The old code ignored the return, so a failed mount
// surfaced only as "no cal found" / "failed to open for writing". When
// format_on_fail is set (save path), reformat and retry so the write can
// succeed; a fresh 64KB LittleFS formats in well under a second.
static bool fsMount(bool format_on_fail) {
    if (LittleFS.begin()) return true;
    SERIAL_PORT.println("LittleFS: mount FAILED.");
    if (!format_on_fail) return false;
    SERIAL_PORT.println("LittleFS: formatting...");
    if (!LittleFS.format() || !LittleFS.begin()) {
        SERIAL_PORT.println("LittleFS: format FAILED — flash problem.");
        return false;
    }
    SERIAL_PORT.println("LittleFS: formatted and mounted.");
    return true;
}

// Saves motor sensor alignment AND current sense driver alignment results.
// Format: offset,direction,gain_a,gain_b,gain_c[,hall_a,hall_b,hall_c]
void save_calibration() {
    if (!fsMount(true)) {
        SERIAL_PORT.println("Calibration not saved!");
        return;
    }
    File file = LittleFS.open("calibration.txt", "w");
    if (!file) {
        SERIAL_PORT.println("Failed to open file for writing.");
        SERIAL_PORT.println("Calibration not saved!");
        LittleFS.end();
        return;
    }
    file.print(motor->zero_electric_angle, 6);
    file.print(",");
    file.print((int)motor->sensor_direction);
    file.print(",");
    file.print(current_sense->gain_a, 6);
    file.print(",");
    file.print(current_sense->gain_b, 6);
    file.print(",");
    file.print(current_sense->gain_c, 6);
#if MOTOR_CONFIG == MOTOR_HALLS
    file.print(",");
    file.print(hall_sensor->pinA);
    file.print(",");
    file.print(hall_sensor->pinB);
    file.print(",");
    file.print(hall_sensor->pinC);
#endif
    file.close();
    LittleFS.end();
    SERIAL_PORT.println("Saved calibration:");
    SERIAL_PORT.print("  zero_electric_angle=");
    SERIAL_PORT.println(motor->zero_electric_angle, 4);
    SERIAL_PORT.print("  sensor_direction=");
    SERIAL_PORT.println(motor->sensor_direction == Direction::CW ? "CW" : "CCW");
    SERIAL_PORT.print("  gain_a=");
    SERIAL_PORT.println(current_sense->gain_a, 5);
    SERIAL_PORT.print("  gain_b=");
    SERIAL_PORT.println(current_sense->gain_b, 5);
    SERIAL_PORT.print("  gain_c=");
    SERIAL_PORT.println(current_sense->gain_c, 5);
#if MOTOR_CONFIG == MOTOR_HALLS
    SERIAL_PORT.print("  hall_pins: A=");
    SERIAL_PORT.print(hall_sensor->pinA);
    SERIAL_PORT.print(" B=");
    SERIAL_PORT.print(hall_sensor->pinB);
    SERIAL_PORT.print(" C=");
    SERIAL_PORT.println(hall_sensor->pinC);
#endif
}

void load_calibration_and_init() {
    // Do not format on the read path -- a spurious mount failure must not wipe a
    // good calibration. If the mount is genuinely bad, Cs (save) will reformat.
    if (!fsMount(false)) {
        SERIAL_PORT.println("No saved calibration found.");
        return;
    }
    File file = LittleFS.open("calibration.txt", "r");
    if (!file) {
        SERIAL_PORT.println("No saved calibration found.");
        LittleFS.end();
        return;
    }
    String line = file.readStringUntil('\n');
    file.close();
    LittleFS.end();

    // Parse: offset,direction,gain_a,gain_b,gain_c[,hall_a,hall_b,hall_c]
    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    int p3 = line.indexOf(',', p2 + 1);
    int p4 = line.indexOf(',', p3 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) {
        SERIAL_PORT.println("ERR: Invalid calibration format (old format? re-save with Cs after align)");
        return;
    }
    float offset   = line.substring(0, p1).toFloat();
    int direction   = line.substring(p1 + 1, p2).toInt();
    float gain_a   = line.substring(p2 + 1, p3).toFloat();
    float gain_b   = line.substring(p3 + 1, p4).toFloat();
    float gain_c   = line.substring(p4 + 1).toFloat();

    SERIAL_PORT.println("Loaded calibration:");
    SERIAL_PORT.print("  zero_electric_angle=");
    SERIAL_PORT.println(offset, 4);
    SERIAL_PORT.print("  sensor_direction=");
    SERIAL_PORT.println(direction == (int)Direction::CW ? "CW" : "CCW");
    SERIAL_PORT.print("  gain_a=");
    SERIAL_PORT.println(gain_a, 5);
    SERIAL_PORT.print("  gain_b=");
    SERIAL_PORT.println(gain_b, 5);
    SERIAL_PORT.print("  gain_c=");
    SERIAL_PORT.println(gain_c, 5);

#if MOTOR_CONFIG == MOTOR_HALLS
    // Parse optional hall pin order: ...,hall_a,hall_b,hall_c
    int p5 = line.indexOf(',', p4 + 1);
    int p6 = (p5 >= 0) ? line.indexOf(',', p5 + 1) : -1;
    int p7 = (p6 >= 0) ? line.indexOf(',', p6 + 1) : -1;
    if (p5 >= 0 && p6 >= 0 && p7 >= 0) {
        int hall_a = line.substring(p5 + 1, p6).toInt();
        int hall_b = line.substring(p6 + 1, p7).toInt();
        int hall_c = line.substring(p7 + 1).toInt();
        SERIAL_PORT.print("  hall_pins: A=");
        SERIAL_PORT.print(hall_a);
        SERIAL_PORT.print(" B=");
        SERIAL_PORT.print(hall_b);
        SERIAL_PORT.print(" C=");
        SERIAL_PORT.println(hall_c);

        // Reconfigure hall sensor if pin order differs from default
        if (hall_a != hall_sensor->pinA || hall_b != hall_sensor->pinB || hall_c != hall_sensor->pinC) {
            SERIAL_PORT.println("  Reconfiguring hall sensor pins...");
            detachInterrupt(digitalPinToInterrupt(hall_sensor->pinA));
            detachInterrupt(digitalPinToInterrupt(hall_sensor->pinB));
            detachInterrupt(digitalPinToInterrupt(hall_sensor->pinC));
            delete hall_sensor;
            hall_sensor = new HallSensor(hall_a, hall_b, hall_c, motor->pole_pairs);
            hall_sensor->init();
            hall_sensor->enableInterrupts(hallA_ISR, hallB_ISR, hallC_ISR);
            motor->linkSensor(hall_sensor);
        }
        hall_order_determined = true;
    } else {
        SERIAL_PORT.println("  WARN: No hall pin data — will need hall scan on next align");
    }
#endif

    // Restore sensor alignment — initFOC() skips physical alignment when set.
    motor->zero_electric_angle = offset;
    motor->sensor_direction = (Direction)direction;

    // Restore current sense gains — driverAlign() determined phase-to-ADC
    // mapping and gain polarity during the original align. These MUST be
    // restored before skipping driverAlign, otherwise FOC runs with wrong
    // current sense phasing (causes rough/unstable operation).
    current_sense->gain_a = gain_a;
    current_sense->gain_b = gain_b;
    current_sense->gain_c = gain_c;

    // Skip driverAlign — we just restored its results above.
    current_sense->skip_align = true;

    motor->enable();
    int result = motor->initFOC();
    current_sense->skip_align = false;

    SERIAL_PORT.print("initFOC result: ");
    SERIAL_PORT.println(result);

    if (result) {
        foc_ready = true;
        motor->disable();
        SERIAL_PORT.println("FOC ready from saved calibration.");
    } else {
        foc_ready = false;
        motor->disable();
        SERIAL_PORT.println("ERR: initFOC failed with saved calibration!");
    }
}

void doCalibration(char *cmd) {
    if (cmd[0] == 's' || cmd[0] == 'S') {
        // Save current calibration
        if (!foc_ready) {
            SERIAL_PORT.println("ERR: Not aligned. Run 'A' first.");
            return;
        }
        save_calibration();
    } else if (cmd[0] == 'l' || cmd[0] == 'L') {
        // Load saved calibration
        initHardware();
        if (!enc_detected) {
            SERIAL_PORT.println("ERR: Encoder not detected — cannot load calibration.");
            return;
        }
        load_calibration_and_init();
    } else {
        SERIAL_PORT.println("Usage: Cs (save) | Cl (load)");
    }
}

// USB PD status and voltage control
void doPD(char *cmd) {
#ifdef HAS_USB_PD
    if (cmd[0] == '\0' || cmd[0] == '\n') {
        SERIAL_PORT.print("PD: voltage=");
        SERIAL_PORT.print(pd_ufp->get_voltage() * 50);
        SERIAL_PORT.print("mV current=");
        SERIAL_PORT.print(pd_ufp->get_current() * 10);
        SERIAL_PORT.print("mA ready=");
        SERIAL_PORT.println(pd_ufp->is_power_ready());
    } else {
        int v = atoi(cmd);
        PD_power_option_t opt;
        switch(v) {
            case 5:  opt = PD_POWER_OPTION_MAX_5V; break;
            case 9:  opt = PD_POWER_OPTION_MAX_9V; break;
            case 12: opt = PD_POWER_OPTION_MAX_12V; break;
            case 15: opt = PD_POWER_OPTION_MAX_15V; break;
            case 20: opt = PD_POWER_OPTION_MAX_20V; break;
            default:
                SERIAL_PORT.println("Usage: U[5|9|12|15|20] or U for status");
                return;
        }
        bool was_enabled = motor->enabled;
        if (was_enabled) motor->disable();
        pd_ufp->set_power_option(opt);
        pd_ready = false;
        SERIAL_PORT.print("PD: requesting ");
        SERIAL_PORT.print(v);
        SERIAL_PORT.println("V...");
    }
#else
    SERIAL_PORT.println("USB PD not enabled (build without HAS_USB_PD)");
#endif
}

// Auto hall scan: drive motor open-loop, record transitions, determine correct pin order
void doHallScan(char *cmd) {
#if MOTOR_CONFIG != MOTOR_HALLS
    SERIAL_PORT.println("ERR: Hall scan only available in MOTOR_HALLS config");
    SERIAL_PORT.println("DONE");
    return;
#else
    initHardware();

    float vmot = readVMOT();
    if (vmot < 10.0f) {
        SERIAL_PORT.print("ERR: VMOT=");
        SERIAL_PORT.print(vmot, 1);
        SERIAL_PORT.println("V, need >10V");
        SERIAL_PORT.println("DONE");
        return;
    }

    float v_align = motor->voltage_sensor_align;
    SERIAL_PORT.println("Auto hall scan: driving motor open-loop...");

    // Enable driver and settle at angle 0
    driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
    float Ua, Ub, Uc;
    Ua = v_align;  // angle=0: cos(0)=1, cos(-2π/3)=-0.5, cos(2π/3)=-0.5
    Ub = v_align * cosf(-_2PI / 3.0f);
    Uc = v_align * cosf(_2PI / 3.0f);
    driver->setPwm((Ua + v_align) * 0.5f, (Ub + v_align) * 0.5f, (Uc + v_align) * 0.5f);
    delay(500);

    // Record raw GPIO values at each hall transition
    const int MAX_TRANS = 256;
    int raw[3][MAX_TRANS];  // [gpio_index][transition]
    int transitions = 0;
    int prev_state = -1;

    // Sweep 12 electrical revolutions slowly (3ms/step, 200 steps/rev ≈ 7s)
    int e_revs = 12;
    int steps = 200 * e_revs;
    for (int i = 0; i <= steps; i++) {
        float angle = (float)i / steps * _2PI * e_revs;
        Ua = v_align * cosf(angle);
        Ub = v_align * cosf(angle - _2PI / 3.0f);
        Uc = v_align * cosf(angle + _2PI / 3.0f);
        driver->setPwm((Ua + v_align) * 0.5f, (Ub + v_align) * 0.5f, (Uc + v_align) * 0.5f);
        delay(3);

        int a = digitalRead(PIN_HALL_A);
        int b = digitalRead(PIN_HALL_B);
        int c = digitalRead(PIN_HALL_C);
        int state = (c << 2) | (b << 1) | a;

        if (state != prev_state && transitions < MAX_TRANS) {
            raw[0][transitions] = a;
            raw[1][transitions] = b;
            raw[2][transitions] = c;
            transitions++;
            prev_state = state;
        }
    }

    // Brake motor to a full stop before disabling
    driver->setPwm(0, 0, 0);  // duty=0 → low-sides on → regenerative brake
    delay(1000);
    driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF);

    SERIAL_PORT.print("Recorded ");
    SERIAL_PORT.print(transitions);
    SERIAL_PORT.println(" hall transitions");

    if (transitions < 12) {
        SERIAL_PORT.println("ERR: Too few transitions — motor may not be following open-loop drive.");
        SERIAL_PORT.println("DONE");
        return;
    }

    // Try all 6 permutations of (GPIO31, GPIO32, GPIO33) → (hallA, hallB, hallC)
    // Score each by net monotonic transitions (tolerates bouncing at sector boundaries)
    // SimpleFOC sector table: hall_state → sector
    //   state = (C<<2)|(B<<1)|A    sectors: {-1, 0, 4, 5, 2, 1, 3, -1}
    int perms[6][3] = {
        {0, 1, 2}, {0, 2, 1}, {1, 0, 2},
        {1, 2, 0}, {2, 0, 1}, {2, 1, 0}
    };
    int gpio_pins[3] = {PIN_HALL_A, PIN_HALL_B, PIN_HALL_C};
    int sector_table[] = {-1, 0, 4, 5, 2, 1, 3, -1};

    int best_perm = -1;
    int best_net = 0;
    bool best_fwd = true;

    for (int p = 0; p < 6; p++) {
        int fwd = 0, rev = 0, invalid = 0, bad_states = 0;
        int prev_sector = -1;

        for (int i = 0; i < transitions; i++) {
            int a = raw[perms[p][0]][i];
            int b = raw[perms[p][1]][i];
            int c = raw[perms[p][2]][i];
            int state = (c << 2) | (b << 1) | a;
            int sector = sector_table[state];

            if (sector == -1) { bad_states++; continue; }  // skip glitch, don't break
            if (prev_sector >= 0) {
                int diff = (sector - prev_sector + 6) % 6;
                if (diff == 1) fwd++;
                else if (diff == 5) rev++;  // -1 mod 6 = 5
                else invalid++;
            }
            prev_sector = sector;
        }

        int net = fwd > rev ? fwd - rev : rev - fwd;
        SERIAL_PORT.print("  perm ");
        SERIAL_PORT.print(p);
        SERIAL_PORT.print(": A=GPIO");
        SERIAL_PORT.print(gpio_pins[perms[p][0]]);
        SERIAL_PORT.print(" B=GPIO");
        SERIAL_PORT.print(gpio_pins[perms[p][1]]);
        SERIAL_PORT.print(" C=GPIO");
        SERIAL_PORT.print(gpio_pins[perms[p][2]]);
        SERIAL_PORT.print(" fwd=");
        SERIAL_PORT.print(fwd);
        SERIAL_PORT.print(" rev=");
        SERIAL_PORT.print(rev);
        SERIAL_PORT.print(" bad=");
        SERIAL_PORT.print(invalid);
        SERIAL_PORT.print(" glitch=");
        SERIAL_PORT.print(bad_states);
        SERIAL_PORT.print(" net=");
        SERIAL_PORT.println(net);

        if (net > best_net) {
            best_net = net;
            best_perm = p;
            best_fwd = fwd > rev;
        }
    }

    // Accept if net monotonic count is at least 80% of total transitions
    int min_net = (transitions - 1) * 80 / 100;  // 80% of transition pairs
    if (best_perm < 0 || best_net < min_net) {
        SERIAL_PORT.print("ERR: No valid hall pin permutation found (best_net=");
        SERIAL_PORT.print(best_net);
        SERIAL_PORT.print(" need>=");
        SERIAL_PORT.print(min_net);
        SERIAL_PORT.println(")");
        SERIAL_PORT.println("Check hall sensor wiring and motor phase connections.");
        SERIAL_PORT.println("DONE");
        return;
    }

    SERIAL_PORT.print("Winner: perm ");
    SERIAL_PORT.print(best_perm);
    SERIAL_PORT.print(" A=GPIO");
    SERIAL_PORT.print(gpio_pins[perms[best_perm][0]]);
    SERIAL_PORT.print(" B=GPIO");
    SERIAL_PORT.print(gpio_pins[perms[best_perm][1]]);
    SERIAL_PORT.print(" C=GPIO");
    SERIAL_PORT.print(gpio_pins[perms[best_perm][2]]);
    SERIAL_PORT.println(best_fwd ? " (fwd)" : " (rev)");

    if (best_perm != 0) {
        SERIAL_PORT.println("Reconfiguring hall sensor pins...");
        // Detach interrupts from all three GPIOs
        detachInterrupt(digitalPinToInterrupt(PIN_HALL_A));
        detachInterrupt(digitalPinToInterrupt(PIN_HALL_B));
        detachInterrupt(digitalPinToInterrupt(PIN_HALL_C));
        delete hall_sensor;
        hall_sensor = new HallSensor(
            gpio_pins[perms[best_perm][0]],
            gpio_pins[perms[best_perm][1]],
            gpio_pins[perms[best_perm][2]],
            motor->pole_pairs
        );
        hall_sensor->init();
        hall_sensor->enableInterrupts(hallA_ISR, hallB_ISR, hallC_ISR);
        motor->linkSensor(hall_sensor);
        SERIAL_PORT.println("Hall sensor reconfigured.");
        SERIAL_PORT.print("To make permanent, update firmware defines to: "
                          "PIN_HALL_A=");
        SERIAL_PORT.print(gpio_pins[perms[best_perm][0]]);
        SERIAL_PORT.print(" PIN_HALL_B=");
        SERIAL_PORT.print(gpio_pins[perms[best_perm][1]]);
        SERIAL_PORT.print(" PIN_HALL_C=");
        SERIAL_PORT.println(gpio_pins[perms[best_perm][2]]);
    } else {
        SERIAL_PORT.println("Current hall pin order is correct.");
    }

    hall_order_determined = true;
    SERIAL_PORT.println("DONE");
#endif
}

// Winding resistance measurement: ramp voltage A→B, measure current
void doWindingResistance(char *cmd) {
    initHardware();

    float vmot = readVMOT();
    if (vmot < 10.0f) {
        SERIAL_PORT.print("ERR: VMOT=");
        SERIAL_PORT.print(vmot, 1);
        SERIAL_PORT.println("V, need >10V");
        SERIAL_PORT.println("DONE");
        return;
    }

    SERIAL_PORT.println("Measuring winding resistance (A-B)...");
    SERIAL_PORT.println("V_applied,Ia,Ib,R_ab");

    float vs2 = driver->voltage_power_supply / 2.0f;
    driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
    // Start at neutral
    driver->setPwm(vs2, vs2, vs2);
    delay(50);

    float last_r = 0;
    float last_v = 0;
    float last_i = 0;

    // Ramp from 100mV to 1V in 25mV steps
    for (float v = 0.1f; v <= 1.0f; v += 0.025f) {
        // Drive current A→B: A = Vs/2 + v/2, B = Vs/2 - v/2, C = neutral
        driver->setPwm(vs2 + v / 2.0f, vs2 - v / 2.0f, vs2);
        delay(50);  // settle (>> L/R time constant)

        // Average a few readings
        float sum_a = 0, sum_b = 0;
        int n = 10;
        for (int i = 0; i < n; i++) {
            PhaseCurrent_s p = current_sense->getPhaseCurrents();
            sum_a += p.a;
            sum_b += p.b;
            delayMicroseconds(200);
        }
        float ia = sum_a / n;
        float ib = sum_b / n;
        float i_meas = (fabsf(ia) + fabsf(ib)) / 2.0f;
        float r = (i_meas > 0.01f) ? v / i_meas : 0;
        last_r = r;
        last_v = v;
        last_i = i_meas;

        SERIAL_PORT.print(v, 4);
        SERIAL_PORT.print(',');
        SERIAL_PORT.print(ia, 4);
        SERIAL_PORT.print(',');
        SERIAL_PORT.print(ib, 4);
        SERIAL_PORT.print(',');
        SERIAL_PORT.println(r, 4);

        if (fabsf(ia) > 1.0f || fabsf(ib) > 1.0f) {
            SERIAL_PORT.println("Current limit (1A) reached.");
            break;
        }
    }

    // Return to neutral and disable
    driver->setPwm(vs2, vs2, vs2);
    delay(5);
    driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF);
    driver->setPwm(0, 0, 0);

    SERIAL_PORT.print("R_ab=");
    SERIAL_PORT.print(last_r, 4);
    SERIAL_PORT.print(" ohm  R_phase=");
    SERIAL_PORT.print(last_r / 2.0f, 4);
    SERIAL_PORT.print(" ohm  (at V=");
    SERIAL_PORT.print(last_v, 3);
    SERIAL_PORT.print(" I=");
    SERIAL_PORT.print(last_i, 3);
    SERIAL_PORT.println("A)");
    SERIAL_PORT.println("DONE");
}

// CS alignment diagnostic: replicate what driverAlign does and print all values
void doCSDebug(char *cmd) {
    initHardware();

    float vmot = readVMOT();
    if (vmot < 10.0f) {
        SERIAL_PORT.print("ERR: VMOT=");
        SERIAL_PORT.print(vmot, 1);
        SERIAL_PORT.println("V, need >10V");
        SERIAL_PORT.println("DONE");
        return;
    }

    float voltage = motor->voltage_sensor_align;
    // Same logic as driverAlign: modulation_centered=1 by default
    float zero = 0;
    if (motor->modulation_centered) zero = driver->voltage_limit / 2.0f;

    SERIAL_PORT.println("=== CS driverAlign diagnostic ===");
    SERIAL_PORT.print("voltage_sensor_align=");
    SERIAL_PORT.println(voltage, 2);
    SERIAL_PORT.print("driver->voltage_limit=");
    SERIAL_PORT.println(driver->voltage_limit, 2);
    SERIAL_PORT.print("driver->voltage_power_supply=");
    SERIAL_PORT.println(driver->voltage_power_supply, 2);
    SERIAL_PORT.print("modulation_centered=");
    SERIAL_PORT.println(motor->modulation_centered);
    SERIAL_PORT.print("zero=");
    SERIAL_PORT.println(zero, 2);
    SERIAL_PORT.print("Phase A final PWM value=");
    SERIAL_PORT.println(voltage + zero, 2);
    SERIAL_PORT.print("Phase B/C PWM value=");
    SERIAL_PORT.println(zero, 2);
    float dc_a_expected = (voltage + zero) / driver->voltage_power_supply;
    float dc_bc_expected = zero / driver->voltage_power_supply;
    SERIAL_PORT.print("Expected duty: A=");
    SERIAL_PORT.print(dc_a_expected * 100, 1);
    SERIAL_PORT.print("% B/C=");
    SERIAL_PORT.print(dc_bc_expected * 100, 1);
    SERIAL_PORT.println("%");

    // Read baseline with all phases at neutral (zero)
    driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON);
    driver->setPwm(zero, zero, zero);
    delay(500);
    PhaseCurrent_s baseline = current_sense->readAverageCurrents();
    SERIAL_PORT.print("Baseline (neutral): a=");
    SERIAL_PORT.print(baseline.a, 4);
    SERIAL_PORT.print(" b=");
    SERIAL_PORT.print(baseline.b, 4);
    SERIAL_PORT.print(" c=");
    SERIAL_PORT.println(baseline.c, 4);

    // Ramp phase A exactly like driverAlign does (100 steps × 3ms = 300ms)
    SERIAL_PORT.println("Ramping phase A...");
    for (int i = 0; i < 100; i++) {
        driver->setPwm(voltage / 100.0f * ((float)i) + zero, zero, zero);
        delay(3);
    }
    delay(500);

    // Read currents same way driverAlign does (readAverageCurrents: 100 readings × 3ms)
    PhaseCurrent_s c_a = current_sense->readAverageCurrents();
    SERIAL_PORT.print("Phase A test: a=");
    SERIAL_PORT.print(c_a.a, 4);
    SERIAL_PORT.print(" b=");
    SERIAL_PORT.print(c_a.b, 4);
    SERIAL_PORT.print(" c=");
    SERIAL_PORT.println(c_a.c, 4);

    // Compute ratios same way driverAlign does
    float ca[3] = {fabsf(c_a.a), fabsf(c_a.b), fabsf(c_a.c)};
    SERIAL_PORT.print("Magnitudes: |a|=");
    SERIAL_PORT.print(ca[0], 4);
    SERIAL_PORT.print(" |b|=");
    SERIAL_PORT.print(ca[1], 4);
    SERIAL_PORT.print(" |c|=");
    SERIAL_PORT.println(ca[2], 4);

    // Find max and compute ratio (same algorithm as driverAlign)
    uint8_t max_i = 0;
    float max_c = 0;
    float max_c_ratio = 0;
    for (int i = 0; i < 3; i++) {
        if (!ca[i]) continue;
        if (ca[i] > max_c) {
            max_c = ca[i];
            max_i = i;
            for (int j = 0; j < 3; j++) {
                if (i == j) continue;
                if (!ca[j]) continue;
                float ratio = max_c / ca[j];
                if (ratio > max_c_ratio) max_c_ratio = ratio;
            }
        }
    }
    const char *phase_names[] = {"a", "b", "c"};
    SERIAL_PORT.print("Max current on phase ");
    SERIAL_PORT.print(phase_names[max_i]);
    SERIAL_PORT.print("=");
    SERIAL_PORT.print(max_c, 4);
    SERIAL_PORT.print("  max_ratio=");
    SERIAL_PORT.print(max_c_ratio, 4);
    SERIAL_PORT.println(max_c_ratio >= 1.5f ? " (PASS >= 1.5)" : " (FAIL < 1.5)");

    // Print individual ratios for clarity
    for (int i = 0; i < 3; i++) {
        if (i == max_i) continue;
        if (ca[i] > 0.001f) {
            SERIAL_PORT.print("  |");
            SERIAL_PORT.print(phase_names[max_i]);
            SERIAL_PORT.print("|/|");
            SERIAL_PORT.print(phase_names[i]);
            SERIAL_PORT.print("| = ");
            SERIAL_PORT.println(max_c / ca[i], 4);
        }
    }

    // Also read raw ADC for reference
    RP2040ADCEngine *eng = getADCEngine();
    SERIAL_PORT.print("Raw ADC (during phase A drive): ch0=");
    SERIAL_PORT.print(eng->getRawChannel(0));
    SERIAL_PORT.print(" ch1=");
    SERIAL_PORT.print(eng->getRawChannel(1));
    SERIAL_PORT.print(" ch2=");
    SERIAL_PORT.println(eng->getRawChannel(2));

    // Return to neutral and phase B test
    driver->setPwm(zero, zero, zero);
    delay(300);

    // Phase B test (same as driverAlign phase B)
    SERIAL_PORT.println("Ramping phase B...");
    for (int i = 0; i < 100; i++) {
        driver->setPwm(zero, voltage / 100.0f * ((float)i) + zero, zero);
        delay(3);
    }
    delay(500);

    PhaseCurrent_s c_b = current_sense->readAverageCurrents();
    SERIAL_PORT.print("Phase B test: a=");
    SERIAL_PORT.print(c_b.a, 4);
    SERIAL_PORT.print(" b=");
    SERIAL_PORT.print(c_b.b, 4);
    SERIAL_PORT.print(" c=");
    SERIAL_PORT.println(c_b.c, 4);

    // Phase C test too
    driver->setPwm(zero, zero, zero);
    delay(300);

    SERIAL_PORT.println("Ramping phase C...");
    for (int i = 0; i < 100; i++) {
        driver->setPwm(zero, zero, voltage / 100.0f * ((float)i) + zero);
        delay(3);
    }
    delay(500);

    PhaseCurrent_s c_c = current_sense->readAverageCurrents();
    SERIAL_PORT.print("Phase C test: a=");
    SERIAL_PORT.print(c_c.a, 4);
    SERIAL_PORT.print(" b=");
    SERIAL_PORT.print(c_c.b, 4);
    SERIAL_PORT.print(" c=");
    SERIAL_PORT.println(c_c.c, 4);

    // Clean up
    driver->setPwm(zero, zero, zero);
    delay(100);
    driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_OFF, PhaseState::PHASE_OFF);
    driver->setPwm(0, 0, 0);

    SERIAL_PORT.println("=== CS offsets ===");
    SERIAL_PORT.print("offset_ia=");
    SERIAL_PORT.print(current_sense->offset_ia, 4);
    SERIAL_PORT.print(" offset_ib=");
    SERIAL_PORT.print(current_sense->offset_ib, 4);
    SERIAL_PORT.print(" offset_ic=");
    SERIAL_PORT.println(current_sense->offset_ic, 4);
    SERIAL_PORT.print("gain_a=");
    SERIAL_PORT.print(current_sense->gain_a, 5);
    SERIAL_PORT.print(" gain_b=");
    SERIAL_PORT.print(current_sense->gain_b, 5);
    SERIAL_PORT.print(" gain_c=");
    SERIAL_PORT.println(current_sense->gain_c, 5);
    SERIAL_PORT.println("DONE");
}

// Pole pair detection: sweep through electrical angles and count
void doPoleFind(char *cmd) {
    float align_voltage = 4.0;
    SERIAL_PORT.println("Pole pair detection starting...");
    SERIAL_PORT.println("Hold motor shaft still if possible, or let it spin freely.");

    motor->enable();
    delay(200);

    // Move to electrical angle 0 and let it settle
    driver->setPwm(align_voltage, 0, 0);
    delay(1000);
#if MOTOR_CONFIG == MOTOR_MT6701
    encoder->update();
    float start_angle = encoder->getAngle();
#elif MOTOR_CONFIG == MOTOR_HALLS
    hall_sensor->update();
    float start_angle = hall_sensor->getAngle();
#endif
    SERIAL_PORT.print("Start sensor angle: ");
    SERIAL_PORT.println(start_angle, 4);

    // Sweep through 6 full electrical revolutions
    int electrical_revs = 6;
    int steps = 200 * electrical_revs;
    for (int i = 0; i <= steps; i++) {
        float elec_angle = (float)i / (float)steps * _2PI * electrical_revs;
        float Ua = align_voltage * cosf(elec_angle);
        float Ub = align_voltage * cosf(elec_angle - _2PI / 3.0f);
        float Uc = align_voltage * cosf(elec_angle + _2PI / 3.0f);
        // Shift from [-V, +V] to [0, 2V] range for the driver
        Ua = (Ua + align_voltage) * 0.5f;
        Ub = (Ub + align_voltage) * 0.5f;
        Uc = (Uc + align_voltage) * 0.5f;
        driver->setPwm(Ua, Ub, Uc);
        delay(5);
    }

    delay(500);
#if MOTOR_CONFIG == MOTOR_MT6701
    encoder->update();
    float end_angle = encoder->getAngle();
#elif MOTOR_CONFIG == MOTOR_HALLS
    hall_sensor->update();
    float end_angle = hall_sensor->getAngle();
#endif
    SERIAL_PORT.print("End sensor angle: ");
    SERIAL_PORT.println(end_angle, 4);

    float mech_revs = (end_angle - start_angle) / _2PI;
    float pp = (float)electrical_revs / mech_revs;
    SERIAL_PORT.print("Mechanical revolutions: ");
    SERIAL_PORT.println(mech_revs, 3);
    SERIAL_PORT.print("Detected pole pairs: ");
    SERIAL_PORT.println(pp, 1);
    SERIAL_PORT.print("Nearest integer: ");
    SERIAL_PORT.println((int)roundf(fabsf(pp)));

    driver->setPwm(0, 0, 0);
    motor->disable();
}

// --- Setup ---
void setup() {
    led.begin();
    LED_BOOT();  // red at boot

    // UART via debug probe (reliable, no USB CDC issues)
    Serial1.setTX(PIN_UART_TX);
    Serial1.setRX(PIN_UART_RX);
    Serial1.begin(115200);
    // Also init USB CDC in case someone connects directly
    Serial.begin(115200);
    delay(500);

    // Construct SimpleFOC objects here (NOT as globals) to avoid static
    // initializers that corrupt RP2350 USB CDC before main() runs.
    // Small yields between constructions prevent USB task starvation.
    driver = new BLDCDriver6PWM(PIN_AH, PIN_AL, PIN_BH, PIN_BL, PIN_CH, PIN_CL);
    delay(1);
    current_sense = new InlineCurrentSense(SHUNT_RESISTOR, CURRENT_AMP_GAIN,
                                           PIN_SENSE_A, PIN_SENSE_B, PIN_SENSE_C);
    delay(1);
    motor = new BLDCMotor(POLE_PAIRS);
    applyMotorTuning();
    delay(1);
#if MOTOR_CONFIG == MOTOR_MT6701
    encoder = new MagneticSensorMT6835(PIN_ENC_CS);
#elif MOTOR_CONFIG == MOTOR_HALLS
    hall_sensor = new HallSensor(PIN_HALL_A, PIN_HALL_B, PIN_HALL_C, POLE_PAIRS);
#endif
    delay(1);
    commander = new Commander();
    commander->com_port = &SERIAL_PORT;
    // on_request drops the "PID curr q| " label but keeps the value, so queries
    // return a bare number for tune.py to parse. VerboseMode::nothing would
    // suppress the value too.
    commander->verbose = VerboseMode::on_request;

#ifdef HAS_USB_PD
    Wire1.setSDA(PIN_PD_SDA);
    Wire1.setSCL(PIN_PD_SCL);
    Wire1.begin();
    pd_ufp = new PD_UFP_Log_c();
    pd_ufp->init(PIN_PD_INT, PD_POWER_OPTION_MAX_20V);
    SERIAL_PORT.println("PD: FUSB302 initialized, requesting 20V");
#endif

    SERIAL_PORT.println("FW:simplefoc");
#if MOTOR_CONFIG == MOTOR_MT6701
    SERIAL_PORT.println("=== Motor Controller (MT6701) ===");
#elif MOTOR_CONFIG == MOTOR_HALLS
    SERIAL_PORT.println("=== Motor Controller (Halls) ===");
#endif
    SERIAL_PORT.println("Ready (send A to align).");

    foc_ready = false;
    enc_detected = false;

    // Commander setup
    commander->add('V', doVmot, "vmot");
    commander->add('H', doHwInit, "hw_init");
    commander->add('M', doMotor, "motor");
    commander->add('T', doTarget, "target");
    commander->add('B', doRun, "run");
    commander->add('A', doAlign, "align");
    commander->add('N', doPolePairs, "pole_pairs");
    commander->add('P', doPoleFind, "polefind");
    commander->add('S', doStep, "step");
    commander->add('R', doReport, "report");
    commander->add('D', doAdcTest, "adc_diag");
    commander->add('C', doCalibration, "cal_save_load");
    commander->add('U', doPD, "usb_pd");
    commander->add('W', doWindingResistance, "winding_R");
    commander->add('F', doHallScan, "hall_scan");
    commander->add('G', doCSDebug, "cs_debug");
    commander->add('Y', doDemo, "sine_demo");

    // Auto-init disabled: stale calibration from flash (wrong shunt config)
    // can corrupt state and prevent align from working.
    // Use H (hw_init), A (align), Cl (load cal) manually.
}

void startSine(float amplitude, float freq) {
    if (!foc_ready) {
        SERIAL_PORT.println("ERR: Not aligned/calibrated. Cannot start sine.");
        return;
    }
    // Reset PID/LPF state
    motor->PID_current_q.reset();
    motor->PID_current_d.reset();
    motor->PID_velocity.reset();
    motor->LPF_current_q.y_prev = 0;
    motor->LPF_current_d.y_prev = 0;
    motor->LPF_velocity.y_prev = 0;
    motor->current_sp = 0;
    motor->feed_forward_current = {0, 0};
    motor->feed_forward_voltage = {0, 0};

    motor->controller = MotionControlType::velocity;
    motor->target = 0;
    motor->enable();
    float vn = driver->voltage_power_supply / 2.0f;
    driver->setPwm(vn, vn, vn);

    sine_amplitude = amplitude;
    sine_freq_hz = freq;
    sine_t0 = millis();
    sine_running = true;
    SERIAL_PORT.print("Sine started: amplitude=");
    SERIAL_PORT.print(amplitude, 1);
    SERIAL_PORT.print(" freq=");
    SERIAL_PORT.println(freq, 2);
}

void stopSine() {
    if (!sine_running) return;
    sine_running = false;
    motor->target = 0;
    motor->disable();
    SERIAL_PORT.println("Sine stopped.");
}

#if DEMO_MODE
// Self-starting demo. Arms once motor power appears -- either a negotiated USB
// PD 20V contract or any VMOT above DEMO_MIN_VMOT, so a bench supply works too
// -- then loads calibration from flash and runs the sine forever.
//
// Everything here prints BEFORE the sine starts. Once sine_running is set,
// outputBlocked() suppresses replies so nothing can stall loopFOC(): a single
// blocking print is ~1ms, i.e. ~20 dropped FOC iterations, which is exactly the
// stutter that makes a "continuous" sine not continuous.
static bool demo_done = false;
static unsigned long demo_last_check = 0;

static void demoTick() {
    if (demo_done || sine_running) return;
    // Poll at 10Hz; readVMOT() falls back to analogRead() before the ADC engine
    // is up, and there is no reason to spin on it.
    unsigned long now = millis();
    if (now - demo_last_check < 100) return;
    demo_last_check = now;

    bool pd_20v = false;
#ifdef HAS_USB_PD
    pd_20v = pd_ready && pd_voltage_raw >= 400;  // 400 * 50mV = 20V
#endif
    float vmot = readVMOT();
    if (!pd_20v && vmot < DEMO_MIN_VMOT) return;

    demo_done = true;  // one shot: never retry, even if the steps below fail
    LED_ARMING();      // blue: power seen, loading calibration
    SERIAL_PORT.print("DEMO: power up (VMOT=");
    SERIAL_PORT.print(vmot, 1);
    SERIAL_PORT.println("V), loading calibration...");

    // Same order as doCalibration('l'): the hardware must be up before
    // load_calibration_and_init() calls initFOC(), or FOC initialises against
    // an uninitialised driver/current sense/encoder and the motor never turns.
    initHardware();
    if (!enc_detected) {
        LED_ERROR();
        SERIAL_PORT.println("DEMO: encoder not detected — not starting.");
        return;
    }
    load_calibration_and_init();
    if (!foc_ready) {
        LED_ERROR();
        SERIAL_PORT.println("DEMO: no valid calibration in flash. Run A then Cs.");
        return;
    }
    startSine(DEMO_SPEED, 1000.0f / DEMO_PERIOD_MS);
    LED_RUNNING();  // purple: demo sine running
    SERIAL_PORT.println("DEMO: running. Output is now suppressed.");
}
#endif

void loop() {
    // Measure loop frequency (update every second)
    loop_count++;
    unsigned long now = millis();
    if (now - loop_freq_t0 >= 1000) {
        loop_freq_hz = loop_count * 1000.0f / (now - loop_freq_t0);
        foc_rate_hz = foc_run_count * 1000.0f / (now - loop_freq_t0);
        loop_count = 0;
        foc_run_count = 0;
        loop_freq_t0 = now;
    }

#ifdef HAS_USB_PD
    pd_ufp->run();
    if (pd_ufp->is_power_ready() && !pd_ready) {
        pd_ready = true;
        pd_voltage_raw = pd_ufp->get_voltage();  // 50mV units
        pd_current_raw = pd_ufp->get_current();  // 10mA units
        SERIAL_PORT.print("PD: ");
        SERIAL_PORT.print(pd_voltage_raw * 50);
        SERIAL_PORT.print("mV ");
        SERIAL_PORT.print(pd_current_raw * 10);
        SERIAL_PORT.println("mA");
        LED_PD();  // purple when PD connected
        if (hw_initialized) {
            delay(50);  // let voltage settle
            float vmot = readVMOT();
            driver->voltage_power_supply = vmot;
            SERIAL_PORT.print("VMOT updated: ");
            SERIAL_PORT.println(vmot, 1);
        }
    }
#endif

#if DEMO_MODE
    demoTick();  // arms on PD 20V or VMOT > DEMO_MIN_VMOT, then runs forever
#endif

    if (sine_running) {
        if (focLoopDue()) {
            float t_sec = (millis() - sine_t0) * 0.001f;
            motor->target = sine_amplitude * sinf(2.0f * 3.14159265f * sine_freq_hz * t_sec);
            motor->loopFOC();
            motor->move();
        }
    }
#if MOTOR_CONFIG == MOTOR_HALLS
    else if (foc_ready && motor->enabled) {
        // Continuous FOC loop for hall sensor motor
        if (focLoopDue()) {
            motor->loopFOC();
            motor->move();
        }
        yield();  // Keep USB CDC alive during continuous FOC
    }
#endif

    commander->run();
}
