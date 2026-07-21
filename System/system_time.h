#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <stdbool.h>
#include <stdint.h>

void SystemTime_Init(void);
uint32_t SystemTime_Now(void);
bool SystemTime_Reached(uint32_t now_ms, uint32_t deadline_ms);
void SystemTime_TickISR(void);

#endif
