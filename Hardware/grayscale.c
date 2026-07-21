/* Eight-channel analog grayscale sensor selected through a 74HC4051. */
#include "grayscale.h"

#include "board.h"

#include <stddef.h>
#include <string.h>

#define GRAY_CH_SWITCH_DELAY_CYCLES  320U
#define GRAY_AD0_RISE_TIMEOUT_MS     200U
#define GRAY_ADC_TIMEOUT_LOOPS        32000U

static GrayscaleSnapshot_t gray_state;

static bool Gray_SetAD0(bool high)
{
    if (!high) {
        DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
        DL_GPIO_disableHiZ(GRAY_AD0_IOMUX);
        return true;
    }

    /* Tianmengxing-64P couples PA23/VREF+ to the undriven PA21/VREF- through
     * C44 (1 uF). Release AD0 and raise it through the internal pull-up so no
     * push-pull edge is injected through that capacitor. HIZ1 releases the
     * pad only while the GPIO output latch is high. */
    DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    DL_GPIO_enableHiZ(GRAY_AD0_IOMUX);
    for (uint16_t elapsed = 0U; elapsed < GRAY_AD0_RISE_TIMEOUT_MS; elapsed++) {
        if (DL_GPIO_readPins(GRAY_AD0_PORT, GRAY_AD0_PIN) != 0U) return true;
        delay_ms(1U);
    }

    return false;
}

static bool Gray_SetChannel(uint8_t channel)
{
    if ((channel & 0x02U) != 0U) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    else                         DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    if ((channel & 0x04U) != 0U) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    else                         DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);

    bool ad0_ok = Gray_SetAD0((channel & 0x01U) != 0U);
    delay_cycles(GRAY_CH_SWITCH_DELAY_CYCLES);
    return ad0_ok;
}

static bool Gray_ReadADC(uint16_t *value)
{
    uint32_t timeout = GRAY_ADC_TIMEOUT_LOOPS;

    DL_ADC12_clearInterruptStatus(GRAY_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    if (!DL_ADC12_isConversionsEnabled(GRAY_ADC_INST)) {
        DL_ADC12_enableConversions(GRAY_ADC_INST);
    }
    DL_ADC12_startConversion(GRAY_ADC_INST);
    while ((DL_ADC12_getRawInterruptStatus(GRAY_ADC_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U) &&
           (timeout > 0U)) {
        timeout--;
    }

    if (timeout == 0U) {
        gray_state.last_adc_status = DL_ADC12_getStatus(GRAY_ADC_INST);
        gray_state.last_adc_flags = DL_ADC12_getRawInterruptStatus(GRAY_ADC_INST,
            DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED |
            DL_ADC12_INTERRUPT_OVERFLOW |
            DL_ADC12_INTERRUPT_UNDERFLOW |
            DL_ADC12_INTERRUPT_TRIG_OVF);
        gray_state.adc_powered = DL_ADC12_isPowerEnabled(GRAY_ADC_INST);
        gray_state.adc_enabled = DL_ADC12_isConversionsEnabled(GRAY_ADC_INST);
        gray_state.adc_started = DL_ADC12_isConversionStarted(GRAY_ADC_INST);
        DL_ADC12_stopConversion(GRAY_ADC_INST);
        return false;
    }

    *value = DL_ADC12_getMemResult(GRAY_ADC_INST, GRAY_ADC_MEM_IDX);
    return true;
}

static bool Gray_SampleChannel(uint8_t channel, uint16_t *value)
{
    uint32_t sum = 0U;

    if (!Gray_SetChannel(channel)) return false;
    for (uint8_t i = 0U; i < GRAYSCALE_SAMPLES_PER_CHANNEL; i++) {
        uint16_t sample;
        if (!Gray_ReadADC(&sample)) return false;
        sum += sample;
    }
    *value = (uint16_t)(sum / GRAYSCALE_SAMPLES_PER_CHANNEL);
    return true;
}

void Grayscale_Init(void)
{
    memset(&gray_state, 0, sizeof(gray_state));

    /* Enable the input buffer so the open-drain rise check reads the pad,
     * then add HIZ1 to make a high output latch release the pin. */
    DL_GPIO_initDigitalInputFeatures(GRAY_AD0_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GRAY_AD0_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY_AD1_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY_AD2_IOMUX);
    DL_GPIO_clearPins(GRAY_AD0_PORT,
        GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN);
    DL_GPIO_enableOutput(GRAY_AD0_PORT,
        GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN);

    /* PA25 stays disconnected from the digital pad and is sampled as A0_2. */
}

bool Grayscale_Scan(void)
{
    /* Read all even channels first, then all odd channels, so PA23/C44 only
     * needs one slow low-to-high transition per complete eight-channel scan. */
    static const uint8_t scan_order[GRAYSCALE_CHANNEL_COUNT] = {
        0U, 2U, 4U, 6U, 1U, 3U, 5U, 7U
    };
    bool scan_ok = true;

    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        uint8_t channel = scan_order[i];
        uint16_t value;
        if (Gray_SampleChannel(channel, &value)) {
            gray_state.adc[channel] = value;
        } else {
            scan_ok = false;
        }
    }

    gray_state.scan_count++;
    gray_state.last_scan_ok = scan_ok;
    if (!scan_ok) gray_state.error_count++;
    return scan_ok;
}

void Grayscale_GetSnapshot(GrayscaleSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = gray_state;
}
