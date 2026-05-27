#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include "MagneticSensorMT6835.h"
#include "../patches/simplefoc_rp2040_current_sense.h"

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

// INA240A1D: 20x gain, 20mOhm shunt
#define SHUNT_RESISTOR 0.020f
#define CURRENT_AMP_GAIN 20.0f

// VMOT sensing: GPIO46 = ADC channel 6, voltage divider 100k / 5.1k
#define PIN_VMOT 46
#define VMOT_DIVIDER_RATIO (105.1f / 5.1f)  // Vmot = Vadc * ratio

// Motor configuration
#define POLE_PAIRS 11

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

#define SERIAL_PORT Serial

// --- Global objects (pointers, constructed in setup() to avoid static initializers) ---
// RP2350 USB CDC is corrupted by ANY SimpleFOC global constructors running before main().
static BLDCDriver6PWM *driver;
static InlineCurrentSense *current_sense;
static BLDCMotor *motor;
static MagneticSensorMT6835 *encoder;
static Commander *commander;
static bool enc_detected = false;
static bool foc_ready = false;

// Loop frequency measurement
static unsigned long loop_count = 0;
static unsigned long loop_freq_t0 = 0;
static float loop_freq_hz = 0;

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

// --- Commander callbacks ---
static bool hw_initialized = false;
void doVmot(char *cmd) {
    SERIAL_PORT.print("VMOT=");
    SERIAL_PORT.println(readVMOT(), 2);
}
void doMotor(char *cmd) { commander->motor(motor, cmd); }
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

static void initHardware() {
    if (hw_initialized) return;

    SERIAL_PORT.println("GPIO init...");
    // Enable encoder 3.3V supply
    pinMode(PIN_V_SW, OUTPUT);
    digitalWrite(PIN_V_SW, LOW);
    // Configure encoder transceiver directions
    pinMode(PIN_ENC_A_SW, OUTPUT);
    pinMode(PIN_ENC_B_SW, OUTPUT);
    pinMode(PIN_ENC_C_SW, OUTPUT);
    digitalWrite(PIN_ENC_A_SW, LOW);
    digitalWrite(PIN_ENC_B_SW, HIGH);
    digitalWrite(PIN_ENC_C_SW, HIGH);
    // Route transceiver signals to encoder connector
    pinMode(PIN_H1_SW, OUTPUT);
    pinMode(PIN_H2_SW, OUTPUT);
    pinMode(PIN_H3_SW, OUTPUT);
    digitalWrite(PIN_H1_SW, LOW);
    digitalWrite(PIN_H2_SW, LOW);
    digitalWrite(PIN_H3_SW, LOW);

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
    if (enc_detected) motor->linkSensor(encoder);
    motor->voltage_limit = 4.0;
    motor->voltage_sensor_align = 1.0;  // reduced from 2.0: 20mOhm shunts saturate INA240 at ~4A
    motor->current_limit = 5.0;
    motor->velocity_limit = 20.0;
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
        SERIAL_PORT.println("ERR: Encoder not detected — cannot align.");
        return;
    }

    // Force full re-calibration every time (don't reuse stale values)
    motor->sensor_direction = Direction::UNKNOWN;
    motor->zero_electric_angle = NOT_SET;

    SERIAL_PORT.print("align_voltage=");
    SERIAL_PORT.println(motor->voltage_sensor_align, 2);
    SERIAL_PORT.println("Aligning...");
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
    encoder->update();
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
        setLED(0, 255, 0);  // Green when initialized
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
        SERIAL_PORT.println("Usage: Si<A>, Sq<A>, Sv<rad/s>, Sp<rad>, Sw<rad/s>");
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
        encoder->update();
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
        SERIAL_PORT.println("t_ms,vel_target,vel,Iq");

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

        // 3 full sine cycles over 3000ms (1Hz sine)
        unsigned long duration = 3000;
        float amplitude = value;  // max velocity in rad/s
        float freq_hz = 1.0f;

        motor->target = 0;
        unsigned long t0 = millis();
        while (millis() - t0 < duration) {
            unsigned long t_ms = millis() - t0;
            float t_sec = t_ms * 0.001f;
            motor->target = amplitude * sinf(2.0f * 3.14159265f * freq_hz * t_sec);

            motor->loopFOC();
            motor->move();

            SERIAL_PORT.print(t_ms);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(motor->target, 4);
            SERIAL_PORT.print(',');
            SERIAL_PORT.print(motor->shaft_velocity, 4);
            SERIAL_PORT.print(',');
            SERIAL_PORT.println(motor->current.q, 4);
        }

        motor->target = 0;
        motor->disable();
        SERIAL_PORT.println("DONE");
        return;
    }

    // --- Standard commutated step tests (q/v/p) ---

    // Save current state
    MotionControlType prev_controller = motor->controller;

    // Configure mode for the test
    switch (mode) {
        case 'q': case 'Q':
            motor->controller = MotionControlType::torque;
            SERIAL_PORT.println("t_ms,Iq_target,Iq,Id,Vq,Vd,Ia,Ib,Ic,raw0,raw1,raw2");
            break;
        case 'v': case 'V':
            motor->controller = MotionControlType::velocity;
            SERIAL_PORT.println("t_ms,vel_target,vel,Iq");
            break;
        case 'p': case 'P':
            motor->controller = MotionControlType::angle;
            SERIAL_PORT.println("t_ms,angle_target,angle,vel");
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
    unsigned long t0 = millis();
    while (millis() - t0 < duration) {
        // Instrument first 10 iterations: print angle, alpha/beta, dq before PID
        if (iter_count < 10 && (mode == 'q' || mode == 'Q')) {
            encoder->update();
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

        switch (mode) {
            case 'q': case 'Q': {
                PhaseCurrent_s phase = current_sense->getPhaseCurrents();
                RP2040ADCEngine *eng = getADCEngine();
                SERIAL_PORT.print(t_ms);
                SERIAL_PORT.print(',');
                SERIAL_PORT.print(motor->target, 4);
                SERIAL_PORT.print(',');
                SERIAL_PORT.print(motor->current.q, 4);
                SERIAL_PORT.print(',');
                SERIAL_PORT.print(motor->current.d, 4);
                SERIAL_PORT.print(',');
                SERIAL_PORT.print(motor->voltage.q, 4);
                SERIAL_PORT.print(',');
                SERIAL_PORT.print(motor->voltage.d, 4);
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
                break;
            }
            case 'v': case 'V':
                SERIAL_PORT.print(t_ms);
                SERIAL_PORT.print(',');
                SERIAL_PORT.print(motor->target, 4);
                SERIAL_PORT.print(',');
                SERIAL_PORT.print(motor->shaft_velocity, 4);
                SERIAL_PORT.print(',');
                SERIAL_PORT.println(motor->current.q, 4);
                break;
            case 'p': case 'P':
                SERIAL_PORT.print(t_ms);
                SERIAL_PORT.print(',');
                SERIAL_PORT.print(motor->target, 4);
                SERIAL_PORT.print(',');
                SERIAL_PORT.print(motor->shaft_angle, 4);
                SERIAL_PORT.print(',');
                SERIAL_PORT.println(motor->shaft_velocity, 4);
                break;
        }
    }

    // Restore previous state and disable motor
    motor->target = 0;
    motor->controller = prev_controller;
    motor->disable();
    SERIAL_PORT.println("DONE");
}

// Report current motor state
void doReport(char *cmd) {
    SERIAL_PORT.print("loop_hz=");
    SERIAL_PORT.print(loop_freq_hz, 0);
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
    // Encoder diagnostics
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
// Saves motor sensor alignment AND current sense driver alignment results.
// Format: offset,direction,gain_a,gain_b,gain_c
void save_calibration() {
    LittleFS.begin();
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
}

void load_calibration_and_init() {
    LittleFS.begin();
    File file = LittleFS.open("calibration.txt", "r");
    if (!file) {
        SERIAL_PORT.println("No saved calibration found.");
        LittleFS.end();
        return;
    }
    String line = file.readStringUntil('\n');
    file.close();
    LittleFS.end();

    // Parse: offset,direction,gain_a,gain_b,gain_c
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
    encoder->update();
    float start_angle = encoder->getAngle();
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
    encoder->update();
    float end_angle = encoder->getAngle();
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
    setLED(255, 0, 0);  // Red at boot

    SERIAL_PORT.begin(115200);
    delay(2000);

    // Construct SimpleFOC objects here (NOT as globals) to avoid static
    // initializers that corrupt RP2350 USB CDC before main() runs.
    // Small yields between constructions prevent USB task starvation.
    driver = new BLDCDriver6PWM(PIN_AH, PIN_AL, PIN_BH, PIN_BL, PIN_CH, PIN_CL);
    delay(1);
    current_sense = new InlineCurrentSense(SHUNT_RESISTOR, CURRENT_AMP_GAIN,
                                           PIN_SENSE_A, PIN_SENSE_B, PIN_SENSE_C);
    delay(1);
    motor = new BLDCMotor(POLE_PAIRS);
    delay(1);
    encoder = new MagneticSensorMT6835(PIN_ENC_CS);
    delay(1);
    commander = new Commander();
    commander->com_port = &SERIAL_PORT;

#ifdef HAS_USB_PD
    Wire1.setSDA(PIN_PD_SDA);
    Wire1.setSCL(PIN_PD_SCL);
    Wire1.begin();
    pd_ufp = new PD_UFP_Log_c();
    pd_ufp->init(PIN_PD_INT, PD_POWER_OPTION_MAX_20V);
    SERIAL_PORT.println("PD: FUSB302 initialized, requesting 20V");
#endif

    SERIAL_PORT.println("FW:simplefoc");
    SERIAL_PORT.println("=== Motor Controller ===");
    SERIAL_PORT.println("Ready (send A to align).");

    foc_ready = false;
    enc_detected = false;

    // Commander setup
    commander->add('V', doVmot, "vmot");
    commander->add('H', doHwInit, "hw_init");
    commander->add('M', doMotor, "motor");
    commander->add('T', doTarget, "target");
    commander->add('A', doAlign, "align");
    commander->add('N', doPolePairs, "pole_pairs");
    commander->add('P', doPoleFind, "polefind");
    commander->add('S', doStep, "step");
    commander->add('R', doReport, "report");
    commander->add('D', doAdcTest, "adc_diag");
    commander->add('C', doCalibration, "cal_save_load");
    commander->add('U', doPD, "usb_pd");

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

void loop() {
    // Measure loop frequency (update every second)
    loop_count++;
    unsigned long now = millis();
    if (now - loop_freq_t0 >= 1000) {
        loop_freq_hz = loop_count * 1000.0f / (now - loop_freq_t0);
        loop_count = 0;
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
        setLED(128, 0, 255);  // Purple when PD connected
        if (hw_initialized) {
            delay(50);  // let voltage settle
            float vmot = readVMOT();
            driver->voltage_power_supply = vmot;
            SERIAL_PORT.print("VMOT updated: ");
            SERIAL_PORT.println(vmot, 1);
        }
        // Auto-start sine test if PD negotiated 20V / 5A
        // 20V = 400 (50mV units), 5A = 500 (10mA units)
        if (pd_voltage_raw >= 400 && pd_current_raw >= 500 && foc_ready) {
            SERIAL_PORT.println("PD: 20V/5A available — starting continuous sine test");
            // startSine(30.0f, 1.0f);
        }
    }
#endif

    if (sine_running) {
        float t_sec = (millis() - sine_t0) * 0.001f;
        motor->target = sine_amplitude * sinf(2.0f * 3.14159265f * sine_freq_hz * t_sec);
        motor->loopFOC();
        motor->move();
    }

    commander->run();
}
