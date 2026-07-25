/* Eight-channel analog grayscale sensor selected through a 74HC4051. */
#include "grayscale.h"

#include "board.h"
#include "system_time.h"

#include <stddef.h>
#include <string.h>

#define GRAY_CH_SWITCH_DELAY_CYCLES  320U
#define GRAY_AD0_RISE_TIMEOUT_MS     200U
#define GRAY_ADC_TIMEOUT_LOOPS        32000U

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

    /* Tianmengxing-64P couples PA23/VREF+ to undriven PA21/VREF- through
     * C44 (1 uF). Release AD0 and raise it through the internal pull-up so
     * no push-pull edge is injected through that capacitor. */
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
        gray_state.last_adc_flags = DL_ADC12_getRawInterruptStatus(
            GRAY_ADC_INST,
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
    gray_state.last_adc_status = DL_ADC12_getStatus(GRAY_ADC_INST);
    gray_state.last_adc_flags = DL_ADC12_getRawInterruptStatus(GRAY_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED |
        DL_ADC12_INTERRUPT_OVERFLOW |
        DL_ADC12_INTERRUPT_UNDERFLOW |
        DL_ADC12_INTERRUPT_TRIG_OVF);
    gray_state.adc_powered = DL_ADC12_isPowerEnabled(GRAY_ADC_INST);
    gray_state.adc_enabled = DL_ADC12_isConversionsEnabled(GRAY_ADC_INST);
    gray_state.adc_started = DL_ADC12_isConversionStarted(GRAY_ADC_INST);
    return true;
}

static bool Gray_SampleSelectedChannel(uint16_t *value)
{
    uint32_t sum = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_SAMPLES_PER_CHANNEL; i++) {
        uint16_t sample;
        if (!Gray_ReadADC(&sample)) return false;
        sum += sample;
    }
    *value = (uint16_t)(sum / GRAYSCALE_SAMPLES_PER_CHANNEL);
    return true;
}

static bool Gray_SampleChannel(uint8_t channel, uint16_t *value)
{
    if (!Gray_SetChannel(channel)) return false;
    return Gray_SampleSelectedChannel(value);
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

    /* Enable PA23 input readback for the open-drain rise check. PA25 remains
     * disconnected from the digital pad and is sampled only as ADC0 A0_2. */
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
}

bool Grayscale_Scan(void)
{
    bool scan_ok = true;

    /* Read evens first and odds second so PA23/C44 makes only one slow
     * low-to-high transition per complete scan. */
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        uint8_t channel = gray_scan_order[i];
        uint16_t value;
        if (Gray_SampleChannel(channel, &value)) {
            gray_state.adc[channel] = value;
        } else {
            scan_ok = false;
        }
    }

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
    uint16_t value;

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

    if (!Gray_SampleSelectedChannel(&value)) {
        gray_async_ok = false;
        Gray_FinishAsyncScan(now_ms);
        return;
    }
    gray_state.adc[channel] = value;
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
