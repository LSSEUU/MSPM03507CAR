#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SAFETY_SENSOR_GYRO = 0,
    SAFETY_SENSOR_GRAYSCALE,
    SAFETY_SENSOR_COUNT
} SafetySensor_t;

#define SAFETY_SENSOR_BIT(sensor) (1UL << (uint32_t)(sensor))

typedef enum {
    SAFETY_FAULT_NONE                = 0U,
    SAFETY_FAULT_EMERGENCY_STOP      = (1UL << 0),
    SAFETY_FAULT_GYRO_TIMEOUT        = (1UL << 1),
    SAFETY_FAULT_GRAYSCALE_TIMEOUT   = (1UL << 2),
    SAFETY_FAULT_ACTION_TIMEOUT      = (1UL << 3),
    SAFETY_FAULT_SCHEDULER_DEADLINE  = (1UL << 4),
    SAFETY_FAULT_WATCHDOG_RESET      = (1UL << 5),
    SAFETY_FAULT_ENCODER_INVALID     = (1UL << 6),
    SAFETY_FAULT_WHEEL_STALL         = (1UL << 7)
} SafetyFault_t;

typedef struct {
    uint32_t fault_bits;
    uint32_t first_fault_ms;
    uint32_t last_fault_ms;
    uint32_t fault_count;
    uint32_t required_sensor_mask;
    uint32_t action_deadline_ms;
    int16_t requested_motor_a;
    int16_t requested_motor_b;
    int16_t applied_motor_a;
    int16_t applied_motor_b;
    int16_t target_speed_left_mm_s;
    int16_t target_speed_right_mm_s;
    int16_t measured_speed_left_mm_s;
    int16_t measured_speed_right_mm_s;
    bool action_active;
} SafetySnapshot_t;

void Safety_Init(uint32_t now_ms);
void Safety_NotifySensor(SafetySensor_t sensor, uint32_t now_ms);
void Safety_BeginAction(uint32_t now_ms, uint32_t timeout_ms,
                        uint32_t required_sensor_mask);
void Safety_EndAction(void);
void Safety_Evaluate(uint32_t now_ms);
void Safety_RaiseFault(uint32_t fault_bits, uint32_t now_ms);
void Safety_EmergencyStop(uint32_t now_ms);
void Safety_SetControlContext(int16_t target_left_mm_s,
                              int16_t target_right_mm_s,
                              int16_t measured_left_mm_s,
                              int16_t measured_right_mm_s);
bool Safety_ClearFaults(uint32_t now_ms);
bool Safety_RequestMotor(int16_t motor_a, int16_t motor_b);
void Safety_StopMotor(void);
bool Safety_HasFault(void);
void Safety_GetSnapshot(SafetySnapshot_t *snapshot);

#endif
