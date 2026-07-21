#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

void Watchdog_Init(void);
void Watchdog_Service(void);
bool Watchdog_WasResetCause(uint32_t reset_cause);

#endif
