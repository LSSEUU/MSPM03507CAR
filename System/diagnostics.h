#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "diagnostics_health.h"

void Diagnostics_Init(void);
void Diagnostics_RunBootSelfTest(void);
void Diagnostics_Poll(void);
void Diagnostics_PrintHelp(void);
void Diagnostics_PrintReport(void);
const DiagReport_t *Diagnostics_GetReports(void);

#endif
