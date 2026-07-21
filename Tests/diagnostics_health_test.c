#include "diagnostics_health.h"
#include <assert.h>
#include <string.h>

int main(void)
{
    DiagReport_t reports[3] = {
        {DIAG_STATUS_OK, 1U, 0U, "ok"},
        {DIAG_STATUS_DEGRADED, 2U, 0U, "manual"},
        {DIAG_STATUS_FAULT, 0U, 1U, "missing"}
    };
    assert(strcmp(Diag_StatusName(DIAG_STATUS_OK), "OK") == 0);
    assert(strcmp(Diag_StatusName(DIAG_STATUS_DEGRADED), "DEGRADED") == 0);
    assert(strcmp(Diag_StatusName(DIAG_STATUS_FAULT), "FAULT") == 0);
    assert(strcmp(Diag_PeripheralName(DIAG_GYRO), "GYRO") == 0);
    assert(Diag_WorstStatus(reports, 1U) == DIAG_STATUS_OK);
    assert(Diag_WorstStatus(reports, 2U) == DIAG_STATUS_DEGRADED);
    assert(Diag_WorstStatus(reports, 3U) == DIAG_STATUS_FAULT);
    assert(Diag_WorstStatus(0, 3U) == DIAG_STATUS_FAULT);
    return 0;
}
