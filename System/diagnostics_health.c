#include "diagnostics_health.h"

const char *Diag_StatusName(DiagStatus_t status)
{
    switch (status) {
        case DIAG_STATUS_OK: return "OK";
        case DIAG_STATUS_DEGRADED: return "DEGRADED";
        case DIAG_STATUS_FAULT: return "FAULT";
        default: return "FAULT";
    }
}

const char *Diag_PeripheralName(DiagPeripheral_t peripheral)
{
    static const char *const names[DIAG_PERIPHERAL_COUNT] = {
        "MOTOR", "ENCODER", "GYRO", "GRAYSCALE", "OLED", "LED", "COMM"
    };
    if ((uint8_t)peripheral >= (uint8_t)DIAG_PERIPHERAL_COUNT) return "UNKNOWN";
    return names[(uint8_t)peripheral];
}

DiagStatus_t Diag_WorstStatus(const DiagReport_t *reports, uint8_t count)
{
    DiagStatus_t worst = DIAG_STATUS_OK;
    if (reports == 0) return DIAG_STATUS_FAULT;
    for (uint8_t i = 0U; i < count; i++)
        if (reports[i].status > worst) worst = reports[i].status;
    return worst;
}
