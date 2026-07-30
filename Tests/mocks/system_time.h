#ifndef TEST_SYSTEM_TIME_H
#define TEST_SYSTEM_TIME_H

#include <stdbool.h>
#include <stdint.h>

uint32_t SystemTime_Now(void);
bool SystemTime_Reached(uint32_t now_ms, uint32_t deadline_ms);

#endif
