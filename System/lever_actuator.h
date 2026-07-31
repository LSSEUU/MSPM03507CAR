#ifndef LEVER_ACTUATOR_H
#define LEVER_ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LEVER_ACTUATOR_OK = 0,
    LEVER_ACTUATOR_NOT_CALIBRATED,
    LEVER_ACTUATOR_SESSION_INVALID,
    LEVER_ACTUATOR_FEEDBACK_STALE,
    LEVER_ACTUATOR_CONFIG_CHANGED,
    LEVER_ACTUATOR_OUT_OF_RANGE,
    LEVER_ACTUATOR_MOTOR_STATE,
    LEVER_ACTUATOR_DIRECTION_FAULT,
    LEVER_ACTUATOR_BUSY,
    LEVER_ACTUATOR_IO
} LeverActuatorResult_t;

typedef enum {
    LEVER_ACTUATOR_BOOT_DISABLED = 0,
    LEVER_ACTUATOR_BOOT_WAIT_SETTLE,
    LEVER_ACTUATOR_BOOT_WAIT_REFERENCE,
    LEVER_ACTUATOR_BOOT_WAIT_ENABLE,
    LEVER_ACTUATOR_BOOT_MOVING_HORIZONTAL,
    LEVER_ACTUATOR_BOOT_HOLDING_HORIZONTAL,
    LEVER_ACTUATOR_BOOT_FAULT
} LeverActuatorBootState_t;

typedef struct {
    uint32_t time_ms;
    uint32_t driver_config_fingerprint;
    uint32_t fault_latched;
    uint32_t encoder_age_ms;
    uint32_t motor_status_age_ms;
    uint32_t motor_position_age_ms;
    int32_t current_counts;
    int32_t target_counts;
    int32_t target_angle_mdeg;
    int64_t motor_position_raw;
    LeverActuatorBootState_t boot_state;
    bool calibration_valid;
    bool session_zero_valid;
    bool armed;
    bool moving;
    bool boot_homed;
} LeverActuatorSnapshot_t;

void LeverActuator_Init(uint32_t now_ms);
void LeverActuator_Task(uint32_t now_ms);
LeverActuatorResult_t LeverActuator_ArmHorizontal(void);
LeverActuatorResult_t LeverActuator_SetTilt(int32_t angle_mdeg);
void LeverActuator_RequestStop(void);
void LeverActuator_GetSnapshot(LeverActuatorSnapshot_t *snapshot);

#endif
