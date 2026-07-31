#ifndef APP_H4_BALL_H
#define APP_H4_BALL_H

#include "ball_cascade.h"
#include "lever_actuator.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_H4_BALL_IDLE = 0,
    APP_H4_BALL_ACTIVE,
    APP_H4_BALL_RETURNING,
    APP_H4_BALL_COMPLETE,
    APP_H4_BALL_FAULT
} AppH4BallState_t;

typedef enum {
    APP_H4_BALL_FAULT_NONE = 0,
    APP_H4_BALL_FAULT_VISION,
    APP_H4_BALL_FAULT_EDGE,
    APP_H4_BALL_FAULT_ACTUATOR,
    APP_H4_BALL_FAULT_CONFIG,
    APP_H4_BALL_FAULT_EXTERNAL_STOP,
    APP_H4_BALL_FAULT_EMERGENCY
} AppH4BallFault_t;

typedef struct {
    uint32_t time_ms;
    uint32_t vision_age_ms;
    int32_t target_mm_x10;
    int32_t position_mm_x10;
    int32_t velocity_mm_s;
    int32_t target_velocity_mm_s;
    int32_t velocity_error_mm_s;
    int32_t raw_tilt_mdeg;
    int32_t tilt_mdeg;
    int32_t actuator_counts;
    uint32_t actuator_fault;
    LeverActuatorResult_t actuator_result;
    AppH4BallState_t state;
    AppH4BallFault_t fault;
    uint8_t vision_sequence;
    bool position_saturated;
    bool tilt_saturated;
} AppH4BallSnapshot_t;

void AppH4Ball_Init(uint32_t now_ms);
bool AppH4Ball_CheckReady(uint32_t now_ms);
bool AppH4Ball_Start(uint32_t now_ms);
void AppH4Ball_Task20ms(uint32_t now_ms);
bool AppH4Ball_Finish(void);
void AppH4Ball_Stop(bool emergency);
void AppH4Ball_EmergencyStop(void);
bool AppH4Ball_IsActive(void);
bool AppH4Ball_IsBusy(void);
bool AppH4Ball_IsComplete(void);
bool AppH4Ball_HasFault(void);
AppH4BallFault_t AppH4Ball_Fault(void);
void AppH4Ball_GetSnapshot(AppH4BallSnapshot_t *snapshot);

#endif
