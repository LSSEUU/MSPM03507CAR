/* Eight-channel digital grayscale sensor selected through a CD4051. */
#include "grayscale.h"

#include "board.h"
#include "system_time.h"

#include <stddef.h>
#include <string.h>

/* Vendor examples require 50-100 us after changing the address. */
#define GRAY_CH_SWITCH_DELAY_CYCLES 1600U
#define GRAY_AD0_RISE_TIMEOUT_MS     200U

static GrayscaleSnapshot_t gray_state;

typedef enum {
    GRAY_ASYNC_IDLE = 0,
    GRAY_ASYNC_SELECT,
    GRAY_ASYNC_WAIT_AD0,
    GRAY_ASYNC_SAMPLE
} GrayAsyncPhase_t;

static const uint8_t gray_scan_order[GRAYSCALE_CHANNEL_COUNT] = {
    0U, 2U, 4U, 6U, 1U, 3U, 5U, 7U
};
static GrayAsyncPhase_t gray_async_phase;
static uint8_t gray_async_index;
static uint32_t gray_async_deadline_ms;
static bool gray_async_ok;

static bool Gray_SetAD0(bool high)
{
    if (!high) {
        DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
        DL_GPIO_disableHiZ(GRAY_AD0_IOMUX);
        return true;
    }

    /* Tianmengxing-64P couples PA23/VREF+ to the undriven PA21/VREF- through
     * C44 (1 uF).  Release AD0 and raise it through the internal pull-up. */
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
    bool ad0_ok;
    if ((channel & 0x02U) != 0U) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    else                         DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    if ((channel & 0x04U) != 0U) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    else                         DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    ad0_ok = Gray_SetAD0((channel & 0x01U) != 0U);
    delay_cycles(GRAY_CH_SWITCH_DELAY_CYCLES);
    return ad0_ok;
}

static uint8_t Gray_ReadDigital(void)
{
    return (DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN) != 0U) ? 1U : 0U;
}

static void Gray_SelectChannelAsync(uint8_t channel, uint32_t now_ms)
{
    if ((channel & 0x02U) != 0U) DL_GPIO_setPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    else                         DL_GPIO_clearPins(GRAY_AD1_PORT, GRAY_AD1_PIN);
    if ((channel & 0x04U) != 0U) DL_GPIO_setPins(GRAY_AD2_PORT, GRAY_AD2_PIN);
    else                         DL_GPIO_clearPins(GRAY_AD2_PORT, GRAY_AD2_PIN);

    if ((channel & 0x01U) == 0U) {
        DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
        DL_GPIO_disableHiZ(GRAY_AD0_IOMUX);
        gray_async_phase = GRAY_ASYNC_SAMPLE;
    } else {
        DL_GPIO_setPins(GRAY_AD0_PORT, GRAY_AD0_PIN);
        DL_GPIO_enableHiZ(GRAY_AD0_IOMUX);
        if (DL_GPIO_readPins(GRAY_AD0_PORT, GRAY_AD0_PIN) != 0U) {
            gray_async_phase = GRAY_ASYNC_SAMPLE;
        } else {
            gray_async_deadline_ms = now_ms + GRAY_AD0_RISE_TIMEOUT_MS;
            gray_async_phase = GRAY_ASYNC_WAIT_AD0;
        }
    }
    delay_cycles(GRAY_CH_SWITCH_DELAY_CYCLES);
}

static void Gray_FinishAsyncScan(uint32_t now_ms)
{
    uint8_t high_mask = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        if (gray_state.level[i] != 0U) high_mask |= (uint8_t)(1U << i);
    }
    gray_state.high_mask = high_mask;
    gray_state.scan_count++;
    gray_state.last_update_ms = now_ms;
    gray_state.last_scan_ok = gray_async_ok;
    if (!gray_async_ok) gray_state.error_count++;
    gray_async_phase = GRAY_ASYNC_IDLE;
}

void Grayscale_Init(void)
{
    memset(&gray_state, 0, sizeof(gray_state));
    gray_async_phase = GRAY_ASYNC_IDLE;
    gray_async_index = 0U;
    gray_async_deadline_ms = 0U;
    gray_async_ok = false;

    DL_GPIO_initDigitalInputFeatures(GRAY_AD0_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GRAY_AD0_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY_AD1_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY_AD2_IOMUX);
    DL_GPIO_initDigitalInputFeatures(GRAY_OUT_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GRAY_AD0_PORT,
        GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN);
    DL_GPIO_enableOutput(GRAY_AD0_PORT,
        GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN);
}

bool Grayscale_Scan(void)
{
    bool scan_ok = true;
    uint8_t high_mask = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        uint8_t channel = gray_scan_order[i];
        if (!Gray_SetChannel(channel)) {
            scan_ok = false;
            continue;
        }
        gray_state.level[channel] = Gray_ReadDigital();
        if (gray_state.level[channel] != 0U)
            high_mask |= (uint8_t)(1U << channel);
    }
    gray_state.high_mask = high_mask;
    gray_state.scan_count++;
    gray_state.last_update_ms = SystemTime_Now();
    gray_state.last_scan_ok = scan_ok;
    if (!scan_ok) gray_state.error_count++;
    return scan_ok;
}

bool Grayscale_RequestScan(void)
{
    if (gray_async_phase != GRAY_ASYNC_IDLE) return false;
    gray_async_index = 0U;
    gray_async_ok = true;
    gray_async_phase = GRAY_ASYNC_SELECT;
    return true;
}

void Grayscale_Task(uint32_t now_ms)
{
    uint8_t channel;
    if (gray_async_phase == GRAY_ASYNC_IDLE) return;
    channel = gray_scan_order[gray_async_index];

    if (gray_async_phase == GRAY_ASYNC_SELECT) {
        Gray_SelectChannelAsync(channel, now_ms);
        if (gray_async_phase != GRAY_ASYNC_SAMPLE) return;
    }
    if (gray_async_phase == GRAY_ASYNC_WAIT_AD0) {
        if (DL_GPIO_readPins(GRAY_AD0_PORT, GRAY_AD0_PIN) != 0U) {
            delay_cycles(GRAY_CH_SWITCH_DELAY_CYCLES);
            gray_async_phase = GRAY_ASYNC_SAMPLE;
        } else if (SystemTime_Reached(now_ms, gray_async_deadline_ms)) {
            gray_async_ok = false;
            Gray_FinishAsyncScan(now_ms);
            return;
        } else {
            return;
        }
    }

    gray_state.level[channel] = Gray_ReadDigital();
    gray_async_index++;
    if (gray_async_index >= GRAYSCALE_CHANNEL_COUNT) {
        Gray_FinishAsyncScan(now_ms);
    } else {
        gray_async_phase = GRAY_ASYNC_SELECT;
    }
}

bool Grayscale_IsScanPending(void)
{
    return (gray_async_phase != GRAY_ASYNC_IDLE);
}

void Grayscale_GetSnapshot(GrayscaleSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = gray_state;
}
