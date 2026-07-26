#include "safety.h"

#include "motor.h"
#include "reset_diag.h"
#include "system_state.h"
#include "system_time.h"
#include "vehicle_config.h"

#include <stddef.h>
#include <string.h>

#ifndef SAFETY_HOST_TEST
#include "ti_msp_dl_config.h"
#endif

typedef struct {
    bool valid;
    uint32_t last_update_ms;
} SensorHealth_t;

static SafetySnapshot_t safety;
static SensorHealth_t sensors[SAFETY_SENSOR_COUNT];
static uint32_t sensor_requirement_started_ms;

static uint32_t Safety_EnterCritical(void)
{
#ifdef SAFETY_HOST_TEST
    return 0U;
#else
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#endif
}

static void Safety_ExitCritical(uint32_t key)
{
#ifdef SAFETY_HOST_TEST
    (void)key;
#else
    if (key == 0U) __enable_irq();
#endif
}

static uint32_t Safety_SensorTimeout(SafetySensor_t sensor)
{
    static const uint16_t timeouts[SAFETY_SENSOR_COUNT] = {
        GYRO_TIMEOUT_MS,
        GRAYSCALE_TIMEOUT_MS
    };
    return timeouts[sensor];
}

static uint32_t Safety_SensorFault(SafetySensor_t sensor)
{
    return (sensor == SAFETY_SENSOR_GYRO) ?
        SAFETY_FAULT_GYRO_TIMEOUT : SAFETY_FAULT_GRAYSCALE_TIMEOUT;
}

void Safety_Init(uint32_t now_ms)
{
    memset(&safety, 0, sizeof(safety));
    memset(sensors, 0, sizeof(sensors));
    sensor_requirement_started_ms = now_ms;
    Motor_Stop();
}

void Safety_NotifySensor(SafetySensor_t sensor, uint32_t now_ms)
{
    if ((uint32_t)sensor >= (uint32_t)SAFETY_SENSOR_COUNT) return;
    sensors[sensor].valid = true;
    sensors[sensor].last_update_ms = now_ms;
}

void Safety_BeginAction(uint32_t now_ms, uint32_t timeout_ms,
                        uint32_t required_sensor_mask)
{
    uint32_t key = Safety_EnterCritical();
    safety.action_active = true;
    safety.action_deadline_ms = now_ms + timeout_ms;
    safety.required_sensor_mask = required_sensor_mask;
    sensor_requirement_started_ms = now_ms;
    Safety_ExitCritical(key);
}

void Safety_EndAction(void)
{
    Safety_StopMotor();
    safety.action_active = false;
    safety.action_deadline_ms = 0U;
    safety.required_sensor_mask = 0U;
}

void Safety_RaiseFault(uint32_t fault_bits, uint32_t now_ms)
{
    uint32_t key;
    if (fault_bits == 0U) return;
    key = Safety_EnterCritical();
    if (safety.fault_bits == 0U) safety.first_fault_ms = now_ms;
    safety.fault_bits |= fault_bits;
    safety.last_fault_ms = now_ms;
    safety.fault_count++;
    safety.action_active = false;
    safety.required_sensor_mask = 0U;
    SystemState_EnterFault();
    Safety_StopMotor();
    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_SAFETY_FAULT);
    Safety_ExitCritical(key);
}

void Safety_EmergencyStop(uint32_t now_ms)
{
    Safety_RaiseFault(SAFETY_FAULT_EMERGENCY_STOP, now_ms);
}

void Safety_SetControlContext(int16_t target_left_mm_s,
                              int16_t target_right_mm_s,
                              int16_t measured_left_mm_s,
                              int16_t measured_right_mm_s)
{
    uint32_t key = Safety_EnterCritical();
    safety.target_speed_left_mm_s = target_left_mm_s;
    safety.target_speed_right_mm_s = target_right_mm_s;
    safety.measured_speed_left_mm_s = measured_left_mm_s;
    safety.measured_speed_right_mm_s = measured_right_mm_s;
    Safety_ExitCritical(key);
}

void Safety_Evaluate(uint32_t now_ms)
{
    if (safety.fault_bits != 0U) {
        Safety_StopMotor();
        return;
    }

    if (safety.action_active &&
        SystemTime_Reached(now_ms, safety.action_deadline_ms)) {
        Safety_RaiseFault(SAFETY_FAULT_ACTION_TIMEOUT, now_ms);
        return;
    }

    if (SystemState_Get() != SYSTEM_STATE_RUN) return;

    for (uint32_t i = 0U; i < (uint32_t)SAFETY_SENSOR_COUNT; i++) {
        uint32_t bit = SAFETY_SENSOR_BIT(i);
        uint32_t reference_ms;
        if ((safety.required_sensor_mask & bit) == 0U) continue;
        reference_ms = sensors[i].valid ? sensors[i].last_update_ms :
                                         sensor_requirement_started_ms;
        if ((uint32_t)(now_ms - reference_ms) >=
            Safety_SensorTimeout((SafetySensor_t)i)) {
            Safety_RaiseFault(Safety_SensorFault((SafetySensor_t)i), now_ms);
            return;
        }
    }
}

bool Safety_ClearFaults(uint32_t now_ms)
{
    uint32_t key = Safety_EnterCritical();
    if (SystemState_Get() != SYSTEM_STATE_FAULT) {
        Safety_ExitCritical(key);
        return false;
    }
    Motor_Stop();
    memset(&safety, 0, sizeof(safety));
    sensor_requirement_started_ms = now_ms;
    bool cleared = SystemState_ClearFault();
    Safety_ExitCritical(key);
    return cleared;
}

bool Safety_RequestMotor(int16_t motor_a, int16_t motor_b)
{
    uint32_t key = Safety_EnterCritical();
    safety.requested_motor_a = motor_a;
    safety.requested_motor_b = motor_b;
    if ((safety.fault_bits != 0U) || !safety.action_active ||
        (SystemState_Get() != SYSTEM_STATE_RUN)) {
        Safety_StopMotor();
        Safety_ExitCritical(key);
        return false;
    }
    safety.applied_motor_a = motor_a;
    safety.applied_motor_b = motor_b;
    Motor_SetPWM(motor_a, motor_b);
    Safety_ExitCritical(key);
    return true;
}

void Safety_StopMotor(void)
{
    safety.applied_motor_a = 0;
    safety.applied_motor_b = 0;
    Motor_Stop();
}

bool Safety_HasFault(void)
{
    return (safety.fault_bits != 0U);
}

void Safety_GetSnapshot(SafetySnapshot_t *snapshot)
{
    uint32_t key;
    if (snapshot == NULL) return;
    key = Safety_EnterCritical();
    *snapshot = safety;
    Safety_ExitCritical(key);
}
