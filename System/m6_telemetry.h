#ifndef M6_TELEMETRY_H
#define M6_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool enabled;
    uint16_t period_ms;
    uint32_t captured;
    uint32_t transmitted;
    uint32_t dropped;
    uint8_t queued;
} M6TelemetryStats_t;

void M6Telemetry_Init(void);
void M6Telemetry_SetLogging(bool enabled, uint16_t period_ms);
void M6Telemetry_StopFromISR(void);
void M6Telemetry_Capture20ms(uint32_t now_ms);
void M6Telemetry_Service(void);
void M6Telemetry_GetStats(M6TelemetryStats_t *stats);

#endif
