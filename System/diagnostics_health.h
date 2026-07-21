#ifndef DIAGNOSTICS_HEALTH_H
#define DIAGNOSTICS_HEALTH_H

#include <stdint.h>

typedef enum {
    DIAG_STATUS_OK = 0,
    DIAG_STATUS_DEGRADED,
    DIAG_STATUS_FAULT
} DiagStatus_t;

typedef enum {
    DIAG_MOTOR = 0,
    DIAG_ENCODER,
    DIAG_GYRO,
    DIAG_GRAYSCALE,
    DIAG_OLED,
    DIAG_LED,
    DIAG_COMMUNICATION,
    DIAG_PERIPHERAL_COUNT
} DiagPeripheral_t;

typedef struct {
    DiagStatus_t status;
    uint32_t sample_count;
    uint32_t error_count;
    const char *detail;
} DiagReport_t;

const char *Diag_StatusName(DiagStatus_t status);
const char *Diag_PeripheralName(DiagPeripheral_t peripheral);
DiagStatus_t Diag_WorstStatus(const DiagReport_t *reports, uint8_t count);

#endif
