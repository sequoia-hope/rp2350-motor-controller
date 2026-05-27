#include <Arduino.h>
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"

// --- Pin definitions (identical to main.cpp) ---
#define PIN_AH 6
#define PIN_AL 7
#define PIN_BH 4
#define PIN_BL 5
#define PIN_CH 2
#define PIN_CL 3

// Current sense pins (INA240A1D, 20x gain, 1.5mOhm shunt)
#define PIN_SENSE_A 42  // GPIO42 = ADC2
#define PIN_SENSE_B 41  // GPIO41 = ADC1
#define PIN_SENSE_C 40  // GPIO40 = ADC0

// VMOT sense: GPIO46 = ADC channel 6
#define VMOT_SENSE_PIN 46
#define VMOT_ADC_CHAN  6

// Current sense ADC channels
#define SENSE_A_CHAN 2
#define SENSE_B_CHAN 1
#define SENSE_C_CHAN 0

// Hardware constants
#define SHUNT_RESISTOR 0.003f
#define CURRENT_AMP_GAIN 20.0f
#define ADC_VREF 3.3f
#define ADC_MAX 4095.0f

// VMOT voltage divider: R59(50k) + R60(50k) top, R61(5.1k) bottom
#define DIVIDER_TOP   (50.0f + 50.0f)
#define DIVIDER_BOT   5.1f
#define DIVIDER_RATIO (DIVIDER_BOT / (DIVIDER_TOP + DIVIDER_BOT))

// PWM config
#define PWM_FREQ 20000
#define DEAD_TIME_NS 100

// --- Globals ---
static uint16_t pwm_wrap;
static uint16_t dead_cycles;
static float offset_a, offset_b, offset_c;  // ADC voltage offsets
static float vmot = 12.0f;                  // measured at startup
static float v_set = 0.0f;                  // commanded voltage across A-B
static bool loop_mode = false;

// --- PID current control ---
static bool pid_enabled = false;
static float i_target = 0.0f;
static float pid_kp = 0.1f;
static float pid_ki = 0.0f;
static float pid_kd = 0.0f;
static float pid_integral = 0.0f;
static float pid_prev_error = 0.0f;
static unsigned long pid_last_us = 0;
static float pid_output_limit = 4.0f;
#define PID_INTERVAL_US 50  // 20 kHz loop rate

// --- ADC oversampling & EMA filter ---
static int adc_oversample = 4;
static float ema_alpha = 0.3f;

static float filt_a = 0.0f, filt_b = 0.0f, filt_c = 0.0f;

// --- Non-blocking serial ---
static char cmd_buf[64];
static uint8_t cmd_len = 0;

// --- PWM setup ---
// Each half-bridge uses one PWM slice:
//   Channel A = high-side (normal polarity)
//   Channel B = low-side (inverted polarity)
// With dead time: high-side duty is reduced, low-side duty is reduced,
// creating a gap where both are off.

struct HalfBridge {
    uint slice;
    uint ah_pin;
    uint al_pin;
};

static HalfBridge bridges[3];

static void init_half_bridge(HalfBridge *hb, uint ah_pin, uint al_pin) {
    hb->ah_pin = ah_pin;
    hb->al_pin = al_pin;

    gpio_set_function(ah_pin, GPIO_FUNC_PWM);
    gpio_set_function(al_pin, GPIO_FUNC_PWM);

    hb->slice = pwm_gpio_to_slice_num(ah_pin);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, pwm_wrap);
    // Both channels same polarity — the gate driver hardware inverts
    // the low-side signal to create complementary drive.
    pwm_config_set_output_polarity(&cfg, false, false);
    pwm_init(hb->slice, &cfg, false);  // don't start yet
}

static void set_half_bridge_duty(HalfBridge *hb, float duty) {
    // duty: 0.0 to 1.0
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    uint16_t counts = (uint16_t)(duty * (float)pwm_wrap);

    // High-side: reduce by dead_cycles (turns off earlier)
    uint16_t ah_counts = (counts > dead_cycles) ? (counts - dead_cycles) : 0;
    // Low-side: reduce by dead_cycles (turns on later)
    // Because B is inverted, setting a higher compare value means shorter ON time
    uint16_t al_counts = (counts + dead_cycles < pwm_wrap) ? (counts + dead_cycles) : pwm_wrap;

    pwm_set_both_levels(hb->slice, ah_counts, al_counts);
}

static void enable_all_pwm(void) {
    // Enable all three slices simultaneously
    uint32_t mask = (1u << bridges[0].slice) |
                    (1u << bridges[1].slice) |
                    (1u << bridges[2].slice);
    pwm_set_mask_enabled(mask);
}

// --- ADC helpers ---
static uint16_t read_adc(uint channel) {
    adc_select_input(channel);
    return adc_read();
}

static uint16_t read_adc_oversampled(uint channel) {
    uint32_t sum = 0;
    adc_select_input(channel);
    for (int i = 0; i < adc_oversample; i++)
        sum += adc_read();
    return (uint16_t)(sum / adc_oversample);
}

static float adc_to_voltage(uint16_t raw) {
    return (float)raw * ADC_VREF / ADC_MAX;
}

static float raw_to_current(uint16_t raw, float offset_v) {
    float v = adc_to_voltage(raw);
    return (v - offset_v) / (SHUNT_RESISTOR * CURRENT_AMP_GAIN);
}

// --- Filtered current sampling ---
static float sample_current_a(void) {
    uint16_t raw = read_adc_oversampled(SENSE_A_CHAN);
    float i = raw_to_current(raw, offset_a);
    filt_a += ema_alpha * (i - filt_a);
    return filt_a;
}

static void sample_currents(float *ia, float *ib, float *ic) {
    uint16_t raw_a = read_adc_oversampled(SENSE_A_CHAN);
    uint16_t raw_b = read_adc_oversampled(SENSE_B_CHAN);
    uint16_t raw_c = read_adc_oversampled(SENSE_C_CHAN);

    float i_a = raw_to_current(raw_a, offset_a);
    float i_b = raw_to_current(raw_b, offset_b);
    float i_c = raw_to_current(raw_c, offset_c);

    filt_a += ema_alpha * (i_a - filt_a);
    filt_b += ema_alpha * (i_b - filt_b);
    filt_c += ema_alpha * (i_c - filt_c);

    *ia = filt_a;
    *ib = filt_b;
    *ic = filt_c;
}

static float read_vmot(void) {
    uint16_t raw = read_adc(VMOT_ADC_CHAN);
    float adc_v = adc_to_voltage(raw);
    return adc_v / DIVIDER_RATIO;
}

// --- Offset calibration ---
static void calibrate_offsets(void) {
    const int N = 1000;
    uint32_t sum_a = 0, sum_b = 0, sum_c = 0;

    for (int i = 0; i < N; i++) {
        sum_a += read_adc(SENSE_A_CHAN);
        sum_b += read_adc(SENSE_B_CHAN);
        sum_c += read_adc(SENSE_C_CHAN);
    }

    offset_a = adc_to_voltage(sum_a / N);
    offset_b = adc_to_voltage(sum_b / N);
    offset_c = adc_to_voltage(sum_c / N);
}

// --- Motor drive ---
// Apply DC voltage across A-B winding. C floats at midpoint.
static void set_voltage(float v) {
    v_set = v;

    if (vmot < 1.0f) {
        // No supply voltage — set all to 50%
        for (int i = 0; i < 3; i++)
            set_half_bridge_duty(&bridges[i], 0.5f);
        return;
    }

    float half_duty = v / vmot / 2.0f;

    // Clamp
    if (half_duty > 0.45f) half_duty = 0.45f;
    if (half_duty < -0.45f) half_duty = -0.45f;

    set_half_bridge_duty(&bridges[0], 0.5f + half_duty);  // Phase A
    set_half_bridge_duty(&bridges[1], 0.5f - half_duty);  // Phase B
    set_half_bridge_duty(&bridges[2], 0.5f);               // Phase C (float)
}

// --- Print helpers ---
static void print_reading(void) {
    uint16_t raw_a = read_adc_oversampled(SENSE_A_CHAN);
    uint16_t raw_b = read_adc_oversampled(SENSE_B_CHAN);
    uint16_t raw_c = read_adc_oversampled(SENSE_C_CHAN);
    float vmot_now = read_vmot();

    float i_a = raw_to_current(raw_a, offset_a);
    float i_b = raw_to_current(raw_b, offset_b);
    float i_c = raw_to_current(raw_c, offset_c);

    Serial.print("VMOT=");
    Serial.print(vmot_now, 2);
    Serial.print("  raw_a=");
    Serial.print(raw_a);
    Serial.print(" raw_b=");
    Serial.print(raw_b);
    Serial.print(" raw_c=");
    Serial.print(raw_c);
    Serial.print("  I_a=");
    Serial.print(i_a, 4);
    Serial.print(" I_b=");
    Serial.print(i_b, 4);
    Serial.print(" I_c=");
    Serial.println(i_c, 4);
}

static void do_step_response(float voltage) {
    const unsigned long BASELINE_MS = 100;
    const unsigned long STEP_MS = 500;
    const unsigned long TAIL_MS = 100;
    const unsigned long TOTAL_MS = BASELINE_MS + STEP_MS + TAIL_MS;

    Serial.println("t_ms,V_set,I_a,I_b,I_c");

    unsigned long t0 = micros();
    unsigned long deadline = t0 + TOTAL_MS * 1000UL;

    // Start at 0V (baseline)
    set_voltage(0.0f);

    while (micros() < deadline) {
        unsigned long now = micros();
        unsigned long elapsed = now - t0;
        float t_ms = (float)elapsed / 1000.0f;

        // 0–100ms: baseline (0V), 100–600ms: step, 600–700ms: tail (0V)
        float v_now;
        if (elapsed >= BASELINE_MS * 1000UL && elapsed < (BASELINE_MS + STEP_MS) * 1000UL) {
            set_voltage(voltage);
            v_now = voltage;
        } else {
            set_voltage(0.0f);
            v_now = 0.0f;
        }

        float i_a, i_b, i_c;
        sample_currents(&i_a, &i_b, &i_c);

        Serial.print(t_ms, 1);
        Serial.print(',');
        Serial.print(v_now, 3);
        Serial.print(',');
        Serial.print(i_a, 4);
        Serial.print(',');
        Serial.print(i_b, 4);
        Serial.print(',');
        Serial.println(i_c, 4);

        delayMicroseconds(500);
    }

    set_voltage(0.0f);
    Serial.println("DONE");
}

static void do_sweep(void) {
    Serial.println("V_set,I_a,I_b,I_c");

    for (float v = 0.0f; v <= 2.05f; v += 0.1f) {
        set_voltage(v);
        delay(200);  // let current settle

        float i_a, i_b, i_c;
        sample_currents(&i_a, &i_b, &i_c);

        Serial.print(v, 1);
        Serial.print(',');
        Serial.print(i_a, 4);
        Serial.print(',');
        Serial.print(i_b, 4);
        Serial.print(',');
        Serial.println(i_c, 4);
    }

    set_voltage(0.0f);
    Serial.println("DONE");
}

// --- PID update ---
static void pid_update(void) {
    float i_a = sample_current_a();

    float error = i_target - i_a;
    float dt = PID_INTERVAL_US * 1e-6f;

    pid_integral += error * dt;
    // Anti-windup: clamp so Ki * integral stays within output limit
    float integral_limit = pid_output_limit / pid_ki;
    if (pid_integral > integral_limit) pid_integral = integral_limit;
    if (pid_integral < -integral_limit) pid_integral = -integral_limit;

    float derivative = (error - pid_prev_error) / dt;
    pid_prev_error = error;

    float output = pid_kp * error + pid_ki * pid_integral + pid_kd * derivative;

    // Clamp output
    if (output > pid_output_limit) output = pid_output_limit;
    if (output < -pid_output_limit) output = -pid_output_limit;

    set_voltage(output);
}

// --- Current step test (PID-based) ---
static void do_current_step(float target_amps) {
    const unsigned long BASELINE_MS = 100;
    const unsigned long STEP_MS = 500;
    const unsigned long TAIL_MS = 100;
    const unsigned long TOTAL_MS = BASELINE_MS + STEP_MS + TAIL_MS;

    Serial.println("t_ms,I_target,I_a,I_b,I_c,V_cmd");

    // Save PID state and enable PID at 0 for baseline
    bool was_enabled = pid_enabled;
    pid_enabled = true;
    i_target = 0.0f;
    pid_integral = 0.0f;
    pid_prev_error = 0.0f;
    pid_last_us = micros();

    unsigned long t0 = micros();
    unsigned long deadline = t0 + TOTAL_MS * 1000UL;

    while (micros() < deadline) {
        unsigned long now = micros();
        unsigned long elapsed = now - t0;
        float t_ms = (float)elapsed / 1000.0f;

        // 0–100ms: baseline (0A), 100–600ms: step, 600–700ms: tail (0A)
        if (elapsed >= BASELINE_MS * 1000UL && elapsed < (BASELINE_MS + STEP_MS) * 1000UL) {
            i_target = target_amps;
        } else {
            i_target = 0.0f;
        }

        // Run PID at fixed rate
        if (now - pid_last_us >= PID_INTERVAL_US) {
            pid_last_us = now;
            pid_update();
        }

        float i_a, i_b, i_c;
        sample_currents(&i_a, &i_b, &i_c);

        Serial.print(t_ms, 1);
        Serial.print(',');
        Serial.print(i_target, 4);
        Serial.print(',');
        Serial.print(i_a, 4);
        Serial.print(',');
        Serial.print(i_b, 4);
        Serial.print(',');
        Serial.print(i_c, 4);
        Serial.print(',');
        Serial.println(v_set, 4);

        delayMicroseconds(500);
    }

    // Return to 0 and restore state
    i_target = 0.0f;
    pid_integral = 0.0f;
    pid_prev_error = 0.0f;
    set_voltage(0.0f);
    pid_enabled = was_enabled;
    Serial.println("DONE");
}

// --- Process serial command ---
static void process_command(const char *line);

// --- Arduino entry points ---
void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println("=== Bare-Metal Current Sense Test ===");

    // --- ADC init ---
    adc_init();
    adc_gpio_init(PIN_SENSE_A);
    adc_gpio_init(PIN_SENSE_B);
    adc_gpio_init(PIN_SENSE_C);
    adc_gpio_init(VMOT_SENSE_PIN);

    // Read VMOT
    vmot = read_vmot();
    Serial.print("VMOT = ");
    Serial.print(vmot, 2);
    Serial.println(" V");

    // --- PWM init ---
    // Calculate wrap value for desired frequency
    uint32_t sys_clk = clock_get_hz(clk_sys);
    pwm_wrap = (uint16_t)(sys_clk / PWM_FREQ - 1);
    dead_cycles = (uint16_t)((uint64_t)sys_clk * DEAD_TIME_NS / 1000000000ULL);

    Serial.print("PWM wrap=");
    Serial.print(pwm_wrap);
    Serial.print("  dead_cycles=");
    Serial.println(dead_cycles);

    init_half_bridge(&bridges[0], PIN_AH, PIN_AL);
    init_half_bridge(&bridges[1], PIN_BH, PIN_BL);
    init_half_bridge(&bridges[2], PIN_CH, PIN_CL);

    // Start at 50% duty (zero voltage across windings)
    for (int i = 0; i < 3; i++)
        set_half_bridge_duty(&bridges[i], 0.5f);

    enable_all_pwm();

    // --- Calibrate current sense offsets ---
    delay(100);  // let PWM settle
    calibrate_offsets();

    // Seed EMA filter state
    filt_a = raw_to_current(read_adc_oversampled(SENSE_A_CHAN), offset_a);
    filt_b = raw_to_current(read_adc_oversampled(SENSE_B_CHAN), offset_b);
    filt_c = raw_to_current(read_adc_oversampled(SENSE_C_CHAN), offset_c);

    Serial.print("Offsets (V): A=");
    Serial.print(offset_a, 4);
    Serial.print(" B=");
    Serial.print(offset_b, 4);
    Serial.print(" C=");
    Serial.println(offset_c, 4);

    Serial.println("Commands: V<v>, C<a>, P<kp>, I<ki>, D<kd>, T<v>, S, R, L, O<n>, E<f>, 0, ?");
    Serial.println("Ready.");
}

void loop() {
    // PID runs at fixed rate regardless of serial activity
    if (pid_enabled) {
        unsigned long now = micros();
        if (now - pid_last_us >= PID_INTERVAL_US) {
            pid_last_us = now;
            pid_update();
        }
    }

    // Handle loop mode: print at ~100Hz until newline received
    if (loop_mode) {
        print_reading();
        delay(10);
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            loop_mode = false;
            cmd_len = 0;
            Serial.println("Loop stopped.");
        }
        return;
    }

    // Non-blocking serial: accumulate chars into cmd_buf
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                process_command(cmd_buf);
                cmd_len = 0;
            }
        } else if (cmd_len < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = c;
        }
    }
}

static void process_command(const char *line) {
    char cmd = line[0];
    const char *arg = line + 1;

    switch (cmd) {
        case '?':
            Serial.println("FW:current_test");
            break;
        case 'V': case 'v': {
            float v = atof(arg);
            pid_enabled = false;
            set_voltage(v);
            Serial.print("Set V=");
            Serial.println(v, 3);
            print_reading();
            break;
        }
        case 'C': case 'c': {
            float a = atof(arg);
            i_target = a;
            pid_integral = 0.0f;
            pid_prev_error = 0.0f;
            pid_last_us = micros();
            pid_enabled = true;
            Serial.print("PID target I=");
            Serial.print(a, 4);
            Serial.println(" A");
            break;
        }
        case 'P': case 'p': {
            pid_kp = atof(arg);
            Serial.print("Kp=");
            Serial.println(pid_kp, 4);
            break;
        }
        case 'I': case 'i': {
            pid_ki = atof(arg);
            Serial.print("Ki=");
            Serial.println(pid_ki, 4);
            break;
        }
        case 'D': case 'd': {
            pid_kd = atof(arg);
            Serial.print("Kd=");
            Serial.println(pid_kd, 4);
            break;
        }
        case 'T': case 't': {
            float val = atof(arg);
            if (pid_enabled) {
                do_current_step(val);
            } else {
                do_step_response(val);
            }
            break;
        }
        case 'S': case 's':
            do_sweep();
            break;
        case 'R': case 'r':
            print_reading();
            break;
        case 'L': case 'l':
            Serial.println("Loop mode (send any key to stop):");
            loop_mode = true;
            break;
        case '0':
            pid_enabled = false;
            i_target = 0.0f;
            pid_integral = 0.0f;
            set_voltage(0.0f);
            Serial.println("All off (50% duty).");
            print_reading();
            break;
        case 'O': case 'o': {
            int val = (int)atof(arg);
            if (val >= 1 && val <= 16) adc_oversample = val;
            Serial.print("ADC_OVERSAMPLE=");
            Serial.println(adc_oversample);
            break;
        }
        case 'E': case 'e': {
            float val = atof(arg);
            if (val >= 0.01f && val <= 1.0f) ema_alpha = val;
            Serial.print("EMA_ALPHA=");
            Serial.println(ema_alpha, 4);
            break;
        }
        default:
            Serial.print("Unknown command: ");
            Serial.println(line);
            Serial.println("Commands: V<v>, C<a>, P<kp>, I<ki>, D<kd>, T<v>, S, R, L, O<n>, E<f>, 0, ?");
            break;
    }
}
