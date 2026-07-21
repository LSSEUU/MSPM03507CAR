/* Eight-channel analog grayscale sensor selected through a 74HC4051. */
#include "grayscale.h"

#define GRAY_CH_SWITCH_DELAY_CYCLES  320U
#define GRAY_AD0_RISE_TIMEOUT_MS     200U
#define GRAY_ADC_TIMEOUT_LOOPS        32000U

Grayscale_t Gray;

static bool Gray_SetAD0(bool high)
{
    if (!high) {
        DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
        DL_GPIO_disableHiZ(GRAY_AD0_IOMUX);
        return true;
    }

    /* Tianmengxing-64P couples PA23/VREF+ to the undriven PA21/VREF- through
     * C44 (1 uF).  Release AD0 and raise it through the internal pull-up so no
     * push-pull edge is injected through that capacitor. */
    /* HIZ1 releases the pad only while the GPIO output latch is high. */
    DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    DL_GPIO_enableHiZ(GRAY_AD0_IOMUX);
    for (uint16_t elapsed = 0U; elapsed < GRAY_AD0_RISE_TIMEOUT_MS; elapsed++) {
        if (DL_GPIO_readPins(GRAY_AD0_PORT, GRAY_AD0_PIN) != 0U) return true;
        delay_ms(1U);
    }

    return false;
}

static bool Gray_SetChannel(uint8_t ch)
{
    if (ch & 0x02U) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    else            DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    if (ch & 0x04U) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    else            DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    bool ad0_ok = Gray_SetAD0((ch & 0x01U) != 0U);
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
        Gray.adc_status = DL_ADC12_getStatus(GRAY_ADC_INST);
        Gray.adc_raw_flags = DL_ADC12_getRawInterruptStatus(GRAY_ADC_INST,
            DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED |
            DL_ADC12_INTERRUPT_OVERFLOW |
            DL_ADC12_INTERRUPT_UNDERFLOW |
            DL_ADC12_INTERRUPT_TRIG_OVF);
        Gray.adc_powered = DL_ADC12_isPowerEnabled(GRAY_ADC_INST) ? 1U : 0U;
        Gray.adc_enabled = DL_ADC12_isConversionsEnabled(GRAY_ADC_INST) ? 1U : 0U;
        Gray.adc_started = DL_ADC12_isConversionStarted(GRAY_ADC_INST) ? 1U : 0U;
        DL_ADC12_stopConversion(GRAY_ADC_INST);
        return false;
    }

    *value = DL_ADC12_getMemResult(GRAY_ADC_INST, GRAY_ADC_MEM_IDX);
    return true;
}

static bool Gray_SampleChannel(uint8_t ch, uint16_t *value)
{
    uint32_t sum = 0U;
    if (!Gray_SetChannel(ch)) return false;

    for (uint8_t i = 0U; i < GRAY_SAMPLE_COUNT; i++) {
        uint16_t sample;
        if (!Gray_ReadADC(&sample)) return false;
        sum += sample;
    }
    *value = (uint16_t)(sum / GRAY_SAMPLE_COUNT);
    return true;
}

void Grayscale_Init(void)
{
    Gray.raw_state = Gray.filtered_state = 0U;
    Gray.position = 0;
    Gray.min_bits = GRAY_MIN_LINE_BITS;
    Gray.scan_count = 0U;
    Gray.error_count = 0U;
    Gray.adc_status = 0U;
    Gray.adc_raw_flags = 0U;
    Gray.adc_powered = 0U;
    Gray.adc_enabled = 0U;
    Gray.adc_started = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        Gray.debounce_cnt[i] = 0U;
        Gray.analog[i] = 0U;
    }

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
    DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN);
    DL_GPIO_enableOutput(GRAY_AD0_PORT, GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN);

    /* PA25 stays disconnected from the digital pad and is sampled as A0_2. */
}

void Grayscale_Update(void)
{
    /* Read all even channels first, then all odd channels, so PA23/C44 only
     * needs one slow low-to-high transition per complete eight-channel scan. */
    static const uint8_t scan_order[GRAYSCALE_CHANNEL_COUNT] = {
        0U, 2U, 4U, 6U, 1U, 3U, 5U, 7U
    };

    bool scan_ok = true;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        uint8_t ch = scan_order[i];
        uint16_t value;
        if (Gray_SampleChannel(ch, &value)) Gray.analog[ch] = value;
        else scan_ok = false;
    }
    Gray.scan_count++;
    if (!scan_ok) Gray.error_count++;
    Gray.raw_state = 0U;
    Gray.filtered_state = 0U;
    Gray.position = GRAY_LINE_LOST;
}

void Grayscale_GetSnapshot(GrayscaleSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = Gray;
}

void Grayscale_SetMinBits(uint8_t bits)
{
    if (bits > 8U) bits = 8U;
    if (bits < 1U) bits = 1U;
    Gray.min_bits = bits;
}
