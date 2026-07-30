/* OUT.8IR.B eight-channel infrared sensor over software I2C. */
#include "grayscale.h"

#include "board.h"
#include "grayscale_config.h"
#include "system_time.h"

#include <stddef.h>
#include <string.h>

#define GRAY_I2C_ADDRESS_7BIT          0x30U
#define GRAY_I2C_WRITE_ADDRESS \
    ((uint8_t)(GRAY_I2C_ADDRESS_7BIT << 1U))
#define GRAY_I2C_READ_ADDRESS \
    ((uint8_t)((GRAY_I2C_ADDRESS_7BIT << 1U) | 1U))
#define GRAY_I2C_DATA_REGISTER         0x50U
#define GRAY_I2C_DELAY_CYCLES           160U
#define GRAY_I2C_SCL_TIMEOUT_LOOPS     3200U
#define GRAY_I2C_RECOVERY_CLOCKS          9U

static GrayscaleSnapshot_t gray_state;
static uint32_t gray_startup_deadline_ms;
static uint32_t gray_last_attempt_ms;
static bool gray_has_attempted;
static bool gray_scan_pending;
static bool gray_bus_timeout;
static bool gray_bus_stuck;

static void Gray_I2CDelay(void)
{
    delay_cycles(GRAY_I2C_DELAY_CYCLES);
}

static void Gray_SDALow(void)
{
    DL_GPIO_clearPins(GRAY_I2C_SDA_PORT, GRAY_I2C_SDA_PIN);
    DL_GPIO_enableOutput(GRAY_I2C_SDA_PORT, GRAY_I2C_SDA_PIN);
}

static void Gray_SDARelease(void)
{
    DL_GPIO_disableOutput(GRAY_I2C_SDA_PORT, GRAY_I2C_SDA_PIN);
}

static void Gray_SCLLow(void)
{
    DL_GPIO_clearPins(GRAY_I2C_SCL_PORT, GRAY_I2C_SCL_PIN);
    DL_GPIO_enableOutput(GRAY_I2C_SCL_PORT, GRAY_I2C_SCL_PIN);
}

static void Gray_SCLRelease(void)
{
    DL_GPIO_disableOutput(GRAY_I2C_SCL_PORT, GRAY_I2C_SCL_PIN);
}

static bool Gray_SCLReleaseAndWait(void)
{
    uint32_t timeout = GRAY_I2C_SCL_TIMEOUT_LOOPS;
    Gray_SCLRelease();
    while (((DL_GPIO_readPins(GRAY_I2C_SCL_PORT, GRAY_I2C_SCL_PIN) &
             GRAY_I2C_SCL_PIN) == 0U) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        gray_bus_timeout = true;
        return false;
    }
    return true;
}

static bool Gray_I2CStart(void)
{
    Gray_SDARelease();
    if (!Gray_SCLReleaseAndWait()) return false;
    Gray_I2CDelay();
    if ((DL_GPIO_readPins(GRAY_I2C_SDA_PORT, GRAY_I2C_SDA_PIN) &
         GRAY_I2C_SDA_PIN) == 0U) {
        gray_bus_stuck = true;
        return false;
    }
    Gray_SDALow();
    Gray_I2CDelay();
    Gray_SCLLow();
    Gray_I2CDelay();
    return true;
}

static bool Gray_I2CStop(void)
{
    Gray_SDALow();
    Gray_I2CDelay();
    if (!Gray_SCLReleaseAndWait()) {
        Gray_SDARelease();
        return false;
    }
    Gray_I2CDelay();
    Gray_SDARelease();
    Gray_I2CDelay();
    return true;
}

static bool Gray_I2CWriteByte(uint8_t value)
{
    uint8_t bit;
    bool ack;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) Gray_SDARelease();
        else                       Gray_SDALow();
        Gray_I2CDelay();
        if (!Gray_SCLReleaseAndWait()) return false;
        Gray_I2CDelay();
        Gray_SCLLow();
        Gray_I2CDelay();
        value <<= 1U;
    }

    Gray_SDARelease();
    Gray_I2CDelay();
    if (!Gray_SCLReleaseAndWait()) return false;
    Gray_I2CDelay();
    ack = ((DL_GPIO_readPins(GRAY_I2C_SDA_PORT, GRAY_I2C_SDA_PIN) &
            GRAY_I2C_SDA_PIN) == 0U);
    Gray_SCLLow();
    Gray_I2CDelay();
    return ack;
}

static bool Gray_I2CReadByteNack(uint8_t *value)
{
    uint8_t bit;
    uint8_t result = 0U;

    Gray_SDARelease();
    for (bit = 0U; bit < 8U; bit++) {
        Gray_I2CDelay();
        if (!Gray_SCLReleaseAndWait()) return false;
        Gray_I2CDelay();
        result <<= 1U;
        if ((DL_GPIO_readPins(GRAY_I2C_SDA_PORT, GRAY_I2C_SDA_PIN) &
             GRAY_I2C_SDA_PIN) != 0U) result |= 1U;
        Gray_SCLLow();
        Gray_I2CDelay();
    }

    /* A single-byte read ends with a NACK, so SDA remains released. */
    Gray_SDARelease();
    if (!Gray_SCLReleaseAndWait()) return false;
    Gray_I2CDelay();
    Gray_SCLLow();
    Gray_I2CDelay();
    *value = result;
    return true;
}

static bool Gray_I2CRecover(bool record_recovery)
{
    uint8_t pulse;

    if (record_recovery) gray_state.recovery_count++;
    Gray_SDARelease();
    for (pulse = 0U; pulse < GRAY_I2C_RECOVERY_CLOCKS; pulse++) {
        if (!Gray_SCLReleaseAndWait()) return false;
        if ((DL_GPIO_readPins(GRAY_I2C_SDA_PORT, GRAY_I2C_SDA_PIN) &
             GRAY_I2C_SDA_PIN) != 0U) break;
        Gray_SCLLow();
        Gray_I2CDelay();
        if (!Gray_SCLReleaseAndWait()) return false;
        Gray_I2CDelay();
    }
    return Gray_I2CStop();
}

static bool Gray_ReadRawMask(uint8_t *raw_mask,
                             GrayscaleI2CStatus_t *status)
{
    gray_bus_timeout = false;
    gray_bus_stuck = false;

    if (!Gray_I2CStart()) {
        *status = gray_bus_stuck ? GRAYSCALE_I2C_STATUS_BUS_STUCK :
                                  GRAYSCALE_I2C_STATUS_SCL_TIMEOUT;
        return false;
    }
    if (!Gray_I2CWriteByte(GRAY_I2C_WRITE_ADDRESS)) {
        *status = gray_bus_timeout ? GRAYSCALE_I2C_STATUS_SCL_TIMEOUT :
            GRAYSCALE_I2C_STATUS_NACK_WRITE_ADDRESS;
        (void)Gray_I2CStop();
        return false;
    }
    if (!Gray_I2CWriteByte(GRAY_I2C_DATA_REGISTER)) {
        *status = gray_bus_timeout ? GRAYSCALE_I2C_STATUS_SCL_TIMEOUT :
            GRAYSCALE_I2C_STATUS_NACK_REGISTER;
        (void)Gray_I2CStop();
        return false;
    }
    if (!Gray_I2CStart()) {
        *status = gray_bus_stuck ? GRAYSCALE_I2C_STATUS_BUS_STUCK :
                                  GRAYSCALE_I2C_STATUS_SCL_TIMEOUT;
        (void)Gray_I2CStop();
        return false;
    }
    if (!Gray_I2CWriteByte(GRAY_I2C_READ_ADDRESS)) {
        *status = gray_bus_timeout ? GRAYSCALE_I2C_STATUS_SCL_TIMEOUT :
            GRAYSCALE_I2C_STATUS_NACK_READ_ADDRESS;
        (void)Gray_I2CStop();
        return false;
    }
    if (!Gray_I2CReadByteNack(raw_mask) || !Gray_I2CStop()) {
        *status = GRAYSCALE_I2C_STATUS_SCL_TIMEOUT;
        return false;
    }

    *status = GRAYSCALE_I2C_STATUS_OK;
    return true;
}

static void Gray_ConvertMask(uint8_t raw_mask)
{
    uint8_t channel;
    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; channel++) {
        uint8_t bit = (GRAYSCALE_IR1_ON_RIGHT != 0U) ?
            (uint8_t)(7U - channel) : channel;
        gray_state.adc[channel] = ((raw_mask & (uint8_t)(1U << bit)) != 0U) ?
            GRAYSCALE_PROXY_LINE_ADC : GRAYSCALE_PROXY_BACKGROUND_ADC;
    }
}

static void Gray_UpdateStartupReady(uint32_t now_ms)
{
    if (!gray_state.startup_ready &&
        SystemTime_Reached(now_ms, gray_startup_deadline_ms)) {
        gray_state.startup_ready = true;
    }
}

static bool Gray_CanStart(uint32_t now_ms)
{
    Gray_UpdateStartupReady(now_ms);
    if (!gray_state.startup_ready || gray_scan_pending) return false;
    if (gray_has_attempted && !SystemTime_Reached(now_ms,
            gray_last_attempt_ms + GRAYSCALE_I2C_POLL_PERIOD_MS)) return false;
    return true;
}

static bool Gray_RunTransaction(uint32_t now_ms)
{
    uint8_t raw_mask = 0U;
    GrayscaleI2CStatus_t status = GRAYSCALE_I2C_STATUS_OK;
    bool ok;

    gray_has_attempted = true;
    gray_last_attempt_ms = now_ms;
    gray_state.adc_started = true;
    ok = Gray_ReadRawMask(&raw_mask, &status);
    gray_state.scan_count++;
    gray_state.last_scan_ok = ok;
    gray_state.last_i2c_status = status;
    gray_state.last_adc_status = (uint32_t)status;
    gray_state.adc_started = false;

    if (ok) {
        gray_state.raw_mask = raw_mask;
        gray_state.last_adc_flags = raw_mask;
        gray_state.last_update_ms = now_ms;
        gray_state.consecutive_error_count = 0U;
        Gray_ConvertMask(raw_mask);
        return true;
    }

    gray_state.error_count++;
    gray_state.consecutive_error_count++;
    if ((status == GRAYSCALE_I2C_STATUS_NACK_WRITE_ADDRESS) ||
        (status == GRAYSCALE_I2C_STATUS_NACK_REGISTER) ||
        (status == GRAYSCALE_I2C_STATUS_NACK_READ_ADDRESS)) {
        gray_state.nack_count++;
    } else {
        gray_state.bus_error_count++;
    }
    (void)Gray_I2CRecover(true);
    return false;
}

void Grayscale_Init(void)
{
    uint8_t channel;
    uint32_t now_ms = SystemTime_Now();

    memset(&gray_state, 0, sizeof(gray_state));
    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; channel++)
        gray_state.adc[channel] = GRAYSCALE_PROXY_BACKGROUND_ADC;
    gray_state.last_i2c_status = GRAYSCALE_I2C_STATUS_STARTUP;
    gray_state.last_adc_status = GRAYSCALE_I2C_STATUS_STARTUP;
    gray_state.adc_powered = true;
    gray_state.adc_enabled = true;
    gray_startup_deadline_ms = now_ms + GRAYSCALE_STARTUP_GRACE_MS;
    gray_last_attempt_ms = now_ms;
    gray_has_attempted = false;
    gray_scan_pending = false;

    DL_GPIO_initDigitalInput(GRAY_I2C_SDA_IOMUX);
    DL_GPIO_initDigitalInput(GRAY_I2C_SCL_IOMUX);
    DL_GPIO_clearPins(GRAY_I2C_SDA_PORT, GRAY_I2C_SDA_PIN);
    DL_GPIO_clearPins(GRAY_I2C_SCL_PORT, GRAY_I2C_SCL_PIN);
    Gray_SDARelease();
    Gray_SCLRelease();
    (void)Gray_I2CRecover(false);
}

bool Grayscale_Scan(void)
{
    uint32_t now_ms = SystemTime_Now();
    if (!Gray_CanStart(now_ms)) return false;
    return Gray_RunTransaction(now_ms);
}

bool Grayscale_RequestScan(void)
{
    uint32_t now_ms = SystemTime_Now();
    if (!Gray_CanStart(now_ms)) return false;
    gray_scan_pending = true;
    gray_state.adc_started = true;
    return true;
}

void Grayscale_Task(uint32_t now_ms)
{
    if (!gray_scan_pending) {
        Gray_UpdateStartupReady(now_ms);
        return;
    }
    gray_scan_pending = false;
    (void)Gray_RunTransaction(now_ms);
}

bool Grayscale_IsScanPending(void)
{
    return gray_scan_pending;
}

void Grayscale_GetSnapshot(GrayscaleSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = gray_state;
}
