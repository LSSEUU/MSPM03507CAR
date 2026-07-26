#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "diagnostics_health.h"
#include "control_params.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DIAGNOSTICS_ACTION_STRAIGHT = 0,
    DIAGNOSTICS_ACTION_TURN_POSITIVE,
    DIAGNOSTICS_ACTION_TURN_NEGATIVE,
    DIAGNOSTICS_ACTION_LINE
} DiagnosticsAction_t;

void Diagnostics_Init(void);
void Diagnostics_RunBootSelfTest(void);
void Diagnostics_Poll(void);
void Diagnostics_ServiceBackground(void);
void Diagnostics_Task100ms(void);
void Diagnostics_SetLegacyTextEnabled(bool enabled);
uint32_t Diagnostics_Get100msDropped(void);
void Diagnostics_PrintHelp(void);
void Diagnostics_PrintReport(void);
const DiagReport_t *Diagnostics_GetReports(void);
bool Diagnostics_ActionArm(DiagnosticsAction_t action,
                           float straight_speed_mm_s,
                           uint32_t straight_duration_ms,
                           uint32_t now_ms);
bool Diagnostics_ActionConfirm(uint32_t now_ms);
bool Diagnostics_ActionCancel(uint32_t now_ms);
bool Diagnostics_ActionIsIdle(void);
bool Diagnostics_WheelSpeedDiagnosticActive(void);
void Diagnostics_OnControlParamsCommitted(const ControlParamsCommit_t *commit);

#ifdef M2_FAULT_INJECTION_TEST
bool Diagnostics_FaultInjectionStopWatchdog(void);
#endif

#endif
