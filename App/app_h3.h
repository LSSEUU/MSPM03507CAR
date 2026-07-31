#ifndef APP_H3_H
#define APP_H3_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_H3_WAIT_READY = 0,
    APP_H3_RUN_PLUS,
    APP_H3_HOLD_PLUS,
    APP_H3_RUN_MINUS,
    APP_H3_HOLD_MINUS,
    APP_H3_RETURNING,
    APP_H3_FAULT,
    APP_H3_CENTER_HOLD
} AppH3State_t;

typedef enum {
    APP_H3_FAULT_NONE = 0,
    APP_H3_FAULT_NOT_READY,
    APP_H3_FAULT_START_REJECTED,
    APP_H3_FAULT_VISION,
    APP_H3_FAULT_EDGE,
    APP_H3_FAULT_DIRECTION,
    APP_H3_FAULT_ACTUATOR,
    APP_H3_FAULT_TIMEOUT,
    APP_H3_FAULT_USER_STOP,
    APP_H3_FAULT_EMERGENCY
} AppH3Fault_t;

typedef struct {
    uint32_t time_ms;
    uint32_t started_ms;
    uint32_t settled_ms;
    uint32_t ready_since_ms;
    uint32_t transitions;
    int32_t target_mm_x10;
    int32_t x_mm_x10;
    int32_t vx_mm_s;
    int32_t raw_pd_mdeg;
    int32_t forced_tilt_mdeg;
    int32_t tilt_mdeg;
    int32_t actuator_counts;
    int32_t actuator_target_counts;
    uint32_t actuator_fault;
    uint32_t vision_age_ms;
    AppH3State_t state;
    AppH3Fault_t fault;
    uint8_t vision_seq;
    bool ready;
    bool active;
    bool score_time_met;
    bool actuator_moving;
    bool actuator_pending;
    bool direction_confirmed;
    bool launch_withdrawing;
} AppH3Snapshot_t;

void AppH3_Init(uint32_t now_ms);
void AppH3_Task20ms(uint32_t now_ms);
void AppH3_Task100ms(uint32_t now_ms);
bool AppH3_Start(uint32_t now_ms);
bool AppH3_StartCenterHold(uint32_t now_ms);
void AppH3_RequestStop(uint32_t now_ms);
void AppH3_EmergencyStop(void);
bool AppH3_IsActive(void);
bool AppH3_IsDevelopmentDisplay(void);
void AppH3_GetSnapshot(AppH3Snapshot_t *snapshot);

#endif
