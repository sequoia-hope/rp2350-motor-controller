
#if defined(TARGET_RP2040) || defined(TARGET_RP2350)


#include "../../hardware_api.h"
#include "./rp2040_mcu.h"
#include "../../../drivers/hardware_specific/rp2040/rp2040_mcu.h"
#include "communication/SimpleFOCDebug.h"

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

// RP2040: ADC pins are GPIO 26-29 (4 channels)
// RP2350B: ADC pins are GPIO 40-47 (8 channels)
#if defined(TARGET_RP2350) && defined(__FIRSTANALOGGPIO) && __FIRSTANALOGGPIO >= 40
  #define RP_ADC_PIN0 40
#else
  #define RP_ADC_PIN0 26
#endif
#define RP_ADC_PIN_LAST (RP_ADC_PIN0 + RP2040_ADC_NUM_CHANNELS - 1)
#define RP_ADC_CHAN(pin) ((pin) - RP_ADC_PIN0)


/* Singleton instance of the ADC engine (pointer to avoid static initializer on RP2350) */
static RP2040ADCEngine *engine;

RP2040ADCEngine* getADCEngine() {
    if (!engine) engine = new RP2040ADCEngine();
    return engine;
}

/* Hardware API implementation */

float _readADCVoltageInline(const int pinA, const void* cs_params) {
    _UNUSED(cs_params);

    RP2040ADCEngine *eng = getADCEngine();
    int chan = RP_ADC_CHAN(pinA);
    if (chan>=0 && chan<RP2040_ADC_NUM_CHANNELS && eng->channelsEnabled[chan]) {
        return eng->samples[eng->channelSlot[chan]] * eng->adc_conv;
    }

    return NAN;
};


void* _configureADCInline(const void *driver_params, const int pinA, const int pinB, const int pinC) {
    _UNUSED(driver_params);

    RP2040ADCEngine *eng = getADCEngine();
    if( _isset(pinA) )
        eng->addPin(pinA);
    if( _isset(pinB) )
        eng->addPin(pinB);
    if( _isset(pinC) )
        eng->addPin(pinC);
    eng->init();
    eng->start();
    return eng;
};


/* ADC engine implementation */


RP2040ADCEngine::RP2040ADCEngine() {
    for (int i = 0; i < RP2040_ADC_NUM_CHANNELS; i++) {
        channelsEnabled[i] = false;
        channelSlot[i] = -1;
    }
    channelCount = 0;
    initialized = false;
};



void RP2040ADCEngine::addPin(int pin) {
    int chan = RP_ADC_CHAN(pin);
    if (chan>=0 && chan<RP2040_ADC_NUM_CHANNELS)
        channelsEnabled[chan] = true;
    else
        SIMPLEFOC_DEBUG("RP2040-CUR: ERR: Not an ADC pin: ", pin);
};




bool RP2040ADCEngine::init() {
    if (initialized)
        return true;

    adc_init();
    int enableMask = 0x00;
    channelCount = 0;
    for (int i = 0; i < RP2040_ADC_NUM_CHANNELS; i++) {
        if (channelsEnabled[i]){
            adc_gpio_init(i+RP_ADC_PIN0);
            enableMask |= (0x01<<i);
            channelSlot[i] = channelCount;
            channelCount++;
        }
    }
    adc_set_round_robin(enableMask);
    adc_fifo_setup(
     true,              // Write each completed conversion to the sample FIFO
     true,              // Enable DMA data request (DREQ)
     1,                 // DREQ asserted when >=1 sample present
     false,             // No ERR bit
     false              // Keep full 12-bit resolution (no 8-bit shift)
    );
    if (samples_per_second<1 || samples_per_second>=500000) {
        samples_per_second = 0;
        adc_set_clkdiv(0);
    }
    else
        adc_set_clkdiv(48000000/samples_per_second);
    SIMPLEFOC_DEBUG("RP2040-CUR: ADC init, channels: ", channelCount);

    // DMA in ring mode: writes continuously to samples[] buffer, wrapping.
    // Ring size must be power-of-2 bytes. channelCount * 2 bytes.
    // For 4 channels: 8 bytes = 2^3, ring_size_bits = 3.
    int ring_size_bits = 0;
    int ring_bytes = channelCount * 2;  // uint16_t per channel
    while ((1 << ring_size_bits) < ring_bytes) ring_size_bits++;

    readDMAChannel = dma_claim_unused_channel(true);
    dma_channel_config cc1 = dma_channel_get_default_config(readDMAChannel);
    channel_config_set_transfer_data_size(&cc1, DMA_SIZE_16);
    channel_config_set_read_increment(&cc1, false);
    channel_config_set_write_increment(&cc1, true);
    channel_config_set_dreq(&cc1, DREQ_ADC);
    channel_config_set_ring(&cc1, true, ring_size_bits);  // wrap write address
    dma_channel_configure(readDMAChannel,
        &cc1,
        samples,        // dest (ring buffer)
        &adc_hw->fifo,  // source
        0xFFFFFFFF,     // run indefinitely (wraps via ring)
        false           // defer start
    );

    SIMPLEFOC_DEBUG("RP2040-CUR: DMA ring init, ring_bits: ", ring_size_bits);

    initialized = true;
    return initialized;
};




void RP2040ADCEngine::start() {
    SIMPLEFOC_DEBUG("RP2040-CUR: ADC engine starting");
    adc_fifo_drain();
    // Set AINSEL to first enabled channel
    for (int i=0;i<RP2040_ADC_NUM_CHANNELS;i++) {
        if (channelsEnabled[i]) {
            adc_select_input(i);
            break;
        }
    }
    dma_start_channel_mask( (1u << readDMAChannel) );
    adc_run(true);
    SIMPLEFOC_DEBUG("RP2040-CUR: ADC engine started");
};




void RP2040ADCEngine::stop() {
    adc_run(false);
    dma_channel_abort(readDMAChannel);
    adc_fifo_drain();
    SIMPLEFOC_DEBUG("RP2040-CUR: ADC engine stopped");
};



uint16_t RP2040ADCEngine::getRawChannel(int chan) {
    if (chan >= 0 && chan < RP2040_ADC_NUM_CHANNELS && channelsEnabled[chan])
        return samples[channelSlot[chan]];
    return 0;
};



#endif
