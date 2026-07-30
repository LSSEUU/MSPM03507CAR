#include "board.h"
#include "grayscale.h"
#include "grayscale_config.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

GPIO_Regs mock_gpioa;

static uint32_t mock_now_ms;
static bool force_sda_low;
static bool force_scl_low;
static uint8_t sda_reads[32];
static size_t sda_read_count;
static size_t sda_read_index;
static uint32_t scl_low_count;

void delay_cycles(uint32_t cycles) { (void)cycles; }
void DL_GPIO_initDigitalInput(uint32_t iomux) { (void)iomux; }
void DL_GPIO_clearPins(GPIO_Regs *port, uint32_t pins)
{ (void)port; (void)pins; }
void DL_GPIO_disableOutput(GPIO_Regs *port, uint32_t pins)
{ (void)port; (void)pins; }
void DL_GPIO_enableOutput(GPIO_Regs *port, uint32_t pins)
{
    (void)port;
    if ((pins & GRAY_I2C_SCL_PIN) != 0U) scl_low_count++;
}

uint32_t DL_GPIO_readPins(GPIO_Regs *port, uint32_t pins)
{
    (void)port;
    if ((pins & GRAY_I2C_SCL_PIN) != 0U)
        return force_scl_low ? 0U : GRAY_I2C_SCL_PIN;
    if ((pins & GRAY_I2C_SDA_PIN) != 0U) {
        bool high;
        if (force_sda_low) return 0U;
        high = (sda_read_index < sda_read_count) ?
            (sda_reads[sda_read_index++] != 0U) : true;
        return high ? GRAY_I2C_SDA_PIN : 0U;
    }
    return 0U;
}

uint32_t SystemTime_Now(void) { return mock_now_ms; }
bool SystemTime_Reached(uint32_t now_ms, uint32_t deadline_ms)
{ return (int32_t)(now_ms - deadline_ms) >= 0; }

static void queue_reset(void)
{
    sda_read_count = 0U;
    sda_read_index = 0U;
}

static void queue_bit(bool high)
{
    assert(sda_read_count < sizeof(sda_reads));
    sda_reads[sda_read_count++] = high ? 1U : 0U;
}

static void queue_success(uint8_t raw_mask)
{
    int bit;
    queue_reset();
    queue_bit(true);   /* initial START sees an idle SDA */
    queue_bit(false);  /* 0x60 ACK */
    queue_bit(false);  /* register 0x50 ACK */
    queue_bit(true);   /* repeated START sees released SDA */
    queue_bit(false);  /* 0x61 ACK */
    for (bit = 7; bit >= 0; bit--)
        queue_bit(((raw_mask >> (uint32_t)bit) & 1U) != 0U);
}

static void queue_write_address_nack(void)
{
    queue_reset();
    queue_bit(true);
    queue_bit(true);
}

static void request_and_run(uint32_t now_ms)
{
    mock_now_ms = now_ms;
    assert(Grayscale_RequestScan());
    assert(Grayscale_IsScanPending());
    Grayscale_Task(now_ms);
    assert(!Grayscale_IsScanPending());
}

int main(void)
{
    GrayscaleSnapshot_t snapshot;
    uint32_t channel;
    uint32_t recovery_start;
    uint32_t scl_start;

    mock_now_ms = 0U;
    Grayscale_Init();
    Grayscale_GetSnapshot(&snapshot);
    assert(!snapshot.startup_ready);
    assert(snapshot.error_count == 0U);
    assert(snapshot.recovery_count == 0U);
    for (channel = 0U; channel < GRAYSCALE_CHANNEL_COUNT; channel++)
        assert(snapshot.adc[channel] == GRAYSCALE_PROXY_BACKGROUND_ADC);

    mock_now_ms = GRAYSCALE_STARTUP_GRACE_MS - 1U;
    assert(!Grayscale_RequestScan());
    Grayscale_GetSnapshot(&snapshot);
    assert(snapshot.scan_count == 0U && snapshot.error_count == 0U);

    queue_success(0x80U);
    request_and_run(GRAYSCALE_STARTUP_GRACE_MS);
    Grayscale_GetSnapshot(&snapshot);
    assert(snapshot.startup_ready && snapshot.last_scan_ok);
    assert(snapshot.raw_mask == 0x80U);
    assert(snapshot.scan_count == 1U && snapshot.last_update_ms == 3000U);
#if GRAYSCALE_IR1_ON_RIGHT
    assert(snapshot.conditioned_mask == 0x01U);
    assert(snapshot.adc[0] == GRAYSCALE_PROXY_LINE_ADC);
    assert(snapshot.adc[7] == GRAYSCALE_PROXY_BACKGROUND_ADC);
#else
    assert(snapshot.conditioned_mask == 0x80U);
    assert(snapshot.adc[0] == GRAYSCALE_PROXY_BACKGROUND_ADC);
    assert(snapshot.adc[7] == GRAYSCALE_PROXY_LINE_ADC);
#endif

    mock_now_ms = 3019U;
    assert(!Grayscale_RequestScan());

    queue_write_address_nack();
    request_and_run(3020U);
    Grayscale_GetSnapshot(&snapshot);
    assert(!snapshot.last_scan_ok);
    assert(snapshot.last_i2c_status ==
           GRAYSCALE_I2C_STATUS_NACK_WRITE_ADDRESS);
    assert(snapshot.scan_count == 2U && snapshot.error_count == 1U);
    assert(snapshot.nack_count == 1U && snapshot.bus_error_count == 0U);
    assert(snapshot.recovery_count == 1U);
    assert(snapshot.consecutive_error_count == 1U);
    assert(snapshot.last_update_ms == 3000U);

    queue_success(0x01U);
    request_and_run(3040U);
    Grayscale_GetSnapshot(&snapshot);
    assert(snapshot.last_scan_ok && snapshot.consecutive_error_count == 0U);
    assert(snapshot.error_count == 1U && snapshot.last_update_ms == 3040U);

    recovery_start = snapshot.recovery_count;
    scl_start = scl_low_count;
    force_sda_low = true;
    request_and_run(3060U);
    force_sda_low = false;
    Grayscale_GetSnapshot(&snapshot);
    assert(snapshot.last_i2c_status == GRAYSCALE_I2C_STATUS_BUS_STUCK);
    assert(snapshot.bus_error_count == 1U);
    assert(snapshot.recovery_count == recovery_start + 1U);
    assert(scl_low_count == scl_start + 9U);

    force_scl_low = true;
    request_and_run(3080U);
    force_scl_low = false;
    Grayscale_GetSnapshot(&snapshot);
    assert(snapshot.last_i2c_status == GRAYSCALE_I2C_STATUS_SCL_TIMEOUT);
    assert(snapshot.bus_error_count == 2U);
    assert(snapshot.last_update_ms == 3040U);

    queue_success(0x18U);
    request_and_run(3100U);
    Grayscale_GetSnapshot(&snapshot);
    assert(snapshot.last_scan_ok && snapshot.raw_mask == 0x18U);
    assert(snapshot.conditioned_mask == 0x18U);
    assert(snapshot.consecutive_error_count == 0U);
    assert(snapshot.error_count == 3U);
    return 0;
}
