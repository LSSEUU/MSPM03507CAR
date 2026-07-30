#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include <stdbool.h>
#include <stdint.h>

#define GRAYSCALE_CHANNEL_COUNT       8U

typedef enum {
    GRAYSCALE_I2C_STATUS_OK = 0,
    GRAYSCALE_I2C_STATUS_STARTUP,
    GRAYSCALE_I2C_STATUS_BUS_STUCK,
    GRAYSCALE_I2C_STATUS_SCL_TIMEOUT,
    GRAYSCALE_I2C_STATUS_NACK_WRITE_ADDRESS,
    GRAYSCALE_I2C_STATUS_NACK_REGISTER,
    GRAYSCALE_I2C_STATUS_NACK_READ_ADDRESS
} GrayscaleI2CStatus_t;

typedef struct {
    uint16_t adc[GRAYSCALE_CHANNEL_COUNT];
    uint32_t scan_count;
    uint32_t last_update_ms;
    uint32_t error_count;
    uint32_t last_adc_status;
    uint32_t last_adc_flags;
    uint32_t nack_count;
    uint32_t bus_error_count;
    uint32_t recovery_count;
    uint32_t consecutive_error_count;
    uint8_t raw_mask;
    GrayscaleI2CStatus_t last_i2c_status;
    bool last_scan_ok;
    bool startup_ready;
    /* Legacy M6 flags retained for wire compatibility. They now describe
     * the software-I2C transport rather than an ADC peripheral. */
    bool adc_powered;
    bool adc_enabled;
    bool adc_started;
} GrayscaleSnapshot_t;

void Grayscale_Init(void);
bool Grayscale_Scan(void);
bool Grayscale_RequestScan(void);
void Grayscale_Task(uint32_t now_ms);
bool Grayscale_IsScanPending(void);
void Grayscale_GetSnapshot(GrayscaleSnapshot_t *snapshot);

#endif
