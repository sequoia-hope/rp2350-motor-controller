

#pragma once

/*
 * RP2040/RP2350 ADC engine for SimpleFOC current sensing.
 *
 * DMA ring-buffer mode: no ISR, no CPU involvement.
 * DMA writes continuously to samples[] in round-robin order.
 * Supports up to 8 ADC channels (RP2350B: GPIO 40-47).
 */


#define SIMPLEFOC_RP2040_ADC_RESOLUTION 4096
#ifndef SIMPLEFOC_RP2040_ADC_VDDA
#define SIMPLEFOC_RP2040_ADC_VDDA 3.3f
#endif

// RP2350B has 8 ADC channels, RP2040 has 4
#if defined(TARGET_RP2350)
#define RP2040_ADC_NUM_CHANNELS 8
#else
#define RP2040_ADC_NUM_CHANNELS 4
#endif


class RP2040ADCEngine {

public:
    RP2040ADCEngine();
    void addPin(int pin);

    bool init();
    void start();
    void stop();

    uint16_t getRawChannel(int chan);

    int samples_per_second = 0; // 0 = max speed (~500ksps free-running)
    float adc_conv = (SIMPLEFOC_RP2040_ADC_VDDA / SIMPLEFOC_RP2040_ADC_RESOLUTION);

    bool initialized;
    int channelCount;
    uint readDMAChannel;

    bool channelsEnabled[RP2040_ADC_NUM_CHANNELS];
    int channelSlot[RP2040_ADC_NUM_CHANNELS];  // maps channel index → position in samples[]
    // Ring buffer: DMA writes here continuously, wrapping every channelCount entries.
    // Must be aligned to power-of-2 size for DMA ring mode.
    volatile uint16_t samples[RP2040_ADC_NUM_CHANNELS] __attribute__((aligned(16)));
};

// Global accessor for the ADC engine singleton
RP2040ADCEngine* getADCEngine();
