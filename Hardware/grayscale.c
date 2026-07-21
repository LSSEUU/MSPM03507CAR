/* Eight-channel digital grayscale sensor selected through a 74HC4051. */
#include "grayscale.h"

#define GRAY_CH_SWITCH_DELAY_CYCLES  320U
#define GRAY_AD0_RISE_TIMEOUT_MS     200U

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

static bool Gray_SampleChannel(uint8_t ch)
{
    uint8_t low_count = 0U;
    if (!Gray_SetChannel(ch)) {
        Gray.error_count++;
        return false;
    }
    for (uint8_t i = 0U; i < GRAY_SAMPLE_COUNT; i++) {
        if (DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN) == 0U) low_count++;
    }
    return (low_count > (GRAY_SAMPLE_COUNT / 2U));
}

static uint8_t Gray_ScanAll(void)
{
    uint8_t mask = 0U;
    /* Read all even channels first, then all odd channels, so PA23/C44 only
     * needs one slow low-to-high transition per complete eight-channel scan. */
    static const uint8_t scan_order[GRAYSCALE_CHANNEL_COUNT] = {
        0U, 2U, 4U, 6U, 1U, 3U, 5U, 7U
    };
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        uint8_t ch = scan_order[i];
        if (Gray_SampleChannel(ch)) mask |= (uint8_t)(1U << ch);
    }
    return mask;
}

static uint8_t Gray_CountBits(uint8_t v)
{
    uint8_t n = 0U;
    while (v) { n++; v &= (uint8_t)(v - 1U); }
    return n;
}

void Grayscale_Init(void)
{
    Gray.raw_state = Gray.filtered_state = 0U;
    Gray.position = 0;
    Gray.min_bits = GRAY_MIN_LINE_BITS;
    Gray.scan_count = 0U;
    Gray.error_count = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++)
        Gray.debounce_cnt[i] = 0U;

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

    /* The installed module provides a thresholded digital OUT signal. */
    DL_GPIO_initDigitalInputFeatures(GRAY_OUT_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

void Grayscale_Update(void)
{
    uint8_t raw = Gray_ScanAll();
    Gray.scan_count++;
    Gray.raw_state = raw;

    if (Gray_CountBits(raw) < Gray.min_bits) raw = 0U;

    for (uint8_t ch = 0U; ch < GRAYSCALE_CHANNEL_COUNT; ch++) {
        uint8_t mask       = (uint8_t)(1U << ch);
        uint8_t raw_bit    = raw & mask;
        uint8_t stable_bit = Gray.filtered_state & mask;
        if (raw_bit == stable_bit) {
            Gray.debounce_cnt[ch] = 0U;
        } else if (++Gray.debounce_cnt[ch] >= GRAY_DEBOUNCE_COUNT) {
            if (raw_bit) Gray.filtered_state |= mask;
            else         Gray.filtered_state &= (uint8_t)(~mask);
            Gray.debounce_cnt[ch] = 0U;
        }
    }

    int32_t wsum = 0;
    uint8_t cnt = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        if (Gray.filtered_state & ((uint8_t)(1U << i))) {
            wsum += (int32_t)i;
            cnt++;
        }
    }
    Gray.position = (cnt > 0U) ? (wsum * GRAY_POSITION_SCALE) / (int32_t)cnt
                               : GRAY_LINE_LOST;
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
