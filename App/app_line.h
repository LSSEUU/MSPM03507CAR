#ifndef APP_LINE_H
#define APP_LINE_H

#include "grayscale.h"
#include "line_follow.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_LINE_IDLE = 0,
    APP_LINE_ARMED,
    APP_LINE_RUNNING
} AppLineState_t;

typedef struct {
    AppLineState_t state;
    uint32_t deadline_ms;
    uint32_t started_ms;
    LineFollowSnapshot_t control;
} AppLineSnapshot_t;

void AppLine_Init(void);
bool AppLine_Arm(uint32_t now_ms);
bool AppLine_ConfirmStart(uint32_t now_ms);
bool AppLine_Stop(uint32_t now_ms);
void AppLine_ProcessGrayscale(const GrayscaleSnapshot_t *gray);
void AppLine_Task20ms(uint32_t now_ms);
void AppLine_GetSnapshot(AppLineSnapshot_t *snapshot);

#endif
