/*
 * grayscale.c  —  八路灰度传感器 (数字模式 + 最小黑点过滤)
 */
#include "grayscale.h"

#define GRAY_CH_SWITCH_DELAY  320U

Grayscale_t Gray;

static void Gray_SetChannel(uint8_t ch)
{
    if (ch & 0x01U) DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    else            DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
    if (ch & 0x02U) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    else            DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    if (ch & 0x04U) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    else            DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    for (volatile uint32_t i = 0U; i < GRAY_CH_SWITCH_DELAY; i++) { __NOP(); }
}

static bool Gray_ReadOut(void)
{
    return (DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN) == 0U);
}

static bool Gray_SampleChannel(uint8_t ch)
{
    uint8_t low_count = 0U;
    Gray_SetChannel(ch);
    for (uint8_t i = 0U; i < GRAY_SAMPLE_COUNT; i++)
        if (Gray_ReadOut()) low_count++;
    return (low_count > (GRAY_SAMPLE_COUNT / 2U));
}

static uint8_t Gray_ScanAll(void)
{
    uint8_t mask = 0U;
    for (uint8_t ch = 0U; ch < GRAYSCALE_CHANNEL_COUNT; ch++)
        if (Gray_SampleChannel(ch)) mask |= (uint8_t)(1U << ch);
    return mask;
}

static uint8_t Gray_CountBits(uint8_t v)
{
    uint8_t n = 0U;
    while (v) { n++; v &= v - 1U; }
    return n;
}

void Grayscale_Init(void)
{
    Gray.raw_state = Gray.filtered_state = 0U;
    Gray.position = 0;
    Gray.min_bits = 3U;
    Gray.scan_count = 0U;
    Gray.error_count = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++)
        Gray.debounce_cnt[i] = 0U;

    DL_GPIO_initDigitalOutput(GRAY_AD0_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY_AD1_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY_AD2_IOMUX);
    DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN);
    DL_GPIO_enableOutput(GRAY_AD0_PORT, GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN);

    DL_GPIO_initDigitalInputFeatures(GRAY_OUT_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

void Grayscale_Update(void)
{
    uint8_t raw = Gray_ScanAll();
    Gray.scan_count++;

    /* Preserve the electrical scan before confidence filtering for M1 diagnostics. */
    Gray.raw_state = raw;

    /* 黑点太少 → 当作没有线 (过滤地板杂讯) */
    if (Gray_CountBits(raw) < Gray.min_bits)
        raw = 0U;

    for (uint8_t ch = 0U; ch < GRAYSCALE_CHANNEL_COUNT; ch++) {
        uint8_t mask       = (uint8_t)(1U << ch);
        uint8_t raw_bit    = raw                    & mask;
        uint8_t stable_bit = Gray.filtered_state    & mask;
        if (raw_bit == stable_bit) {
            Gray.debounce_cnt[ch] = 0U;
        } else {
            Gray.debounce_cnt[ch]++;
            if (Gray.debounce_cnt[ch] >= GRAY_DEBOUNCE_COUNT) {
                if (raw_bit) Gray.filtered_state |= mask;
                else          Gray.filtered_state &= (uint8_t)(~mask);
                Gray.debounce_cnt[ch] = 0U;
            }
        }
    }

    int32_t wsum = 0; uint8_t cnt = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        if (Gray.filtered_state & ((uint8_t)(1U << i))) {
            wsum += (int32_t)i; cnt++;
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
