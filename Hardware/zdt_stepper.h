#ifndef ZDT_STEPPER_H
#define ZDT_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

#define ZDT_STEPPER_ADDRESS                         (0x01U)
#define ZDT_STEPPER_CHECK_BYTE                      (0x6BU)
#define ZDT_STEPPER_MAX_SPEED_RPM                   (3000U)
#define ZDT_STEPPER_POSITION_COUNTS_PER_REVOLUTION  (65536ULL)

#define ZDT_STEPPER_MOTOR_ENABLED            (1U << 0)
#define ZDT_STEPPER_MOTOR_POSITION_REACHED   (1U << 1)
#define ZDT_STEPPER_MOTOR_STALLED            (1U << 2)
#define ZDT_STEPPER_MOTOR_STALL_PROTECTION   (1U << 3)
#define ZDT_STEPPER_MOTOR_LEFT_LIMIT         (1U << 4)
#define ZDT_STEPPER_MOTOR_RIGHT_LIMIT        (1U << 5)
#define ZDT_STEPPER_MOTOR_POWER_LOSS         (1U << 7)

#define ZDT_STEPPER_HOME_ENCODER_READY       (1U << 0)
#define ZDT_STEPPER_HOME_CALIBRATION_READY   (1U << 1)
#define ZDT_STEPPER_HOME_RUNNING             (1U << 2)
#define ZDT_STEPPER_HOME_FAILED              (1U << 3)
#define ZDT_STEPPER_HOME_OVER_TEMPERATURE    (1U << 4)
#define ZDT_STEPPER_HOME_OVER_CURRENT        (1U << 5)

typedef enum {
    ZDT_STEPPER_OK = 0,
    ZDT_STEPPER_INVALID_ARGUMENT,
    ZDT_STEPPER_TX_FULL
} ZdtStepperResult_t;

typedef enum {
    ZDT_STEPPER_DIRECTION_CW = 0,
    ZDT_STEPPER_DIRECTION_CCW = 1
} ZdtStepperDirection_t;

typedef enum {
    ZDT_STEPPER_POSITION_RELATIVE_LAST_TARGET = 0,
    ZDT_STEPPER_POSITION_ABSOLUTE_ZERO = 1,
    ZDT_STEPPER_POSITION_RELATIVE_CURRENT = 2
} ZdtStepperPositionMode_t;

typedef enum {
    ZDT_STEPPER_HOME_NEAREST_SINGLE_TURN = 0,
    ZDT_STEPPER_HOME_DIRECTION_SINGLE_TURN = 1,
    ZDT_STEPPER_HOME_SENSORLESS = 2,
    ZDT_STEPPER_HOME_LIMIT_SWITCH = 3,
    ZDT_STEPPER_HOME_ABSOLUTE_ZERO = 4,
    ZDT_STEPPER_HOME_POWER_LOSS_POSITION = 5
} ZdtStepperHomeMode_t;

typedef enum {
    ZDT_STEPPER_RESPONSE_NONE = 0,
    ZDT_STEPPER_RESPONSE_ACCEPTED,
    ZDT_STEPPER_RESPONSE_AT_ORIGIN,
    ZDT_STEPPER_RESPONSE_LIMIT_ACTIVE,
    ZDT_STEPPER_RESPONSE_COMPLETED,
    ZDT_STEPPER_RESPONSE_PARAMETER_OR_STATE_ERROR,
    ZDT_STEPPER_RESPONSE_FORMAT_ERROR,
    ZDT_STEPPER_RESPONSE_DATA
} ZdtStepperResponse_t;

typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t tx_overflow;
    uint32_t rx_overflow;
    uint32_t hardware_errors;
    uint32_t invalid_frames;
    uint32_t checksum_errors;
    uint32_t discarded_rx_bytes;
    uint32_t preempted_frames;
    uint16_t tx_queued;
    uint16_t rx_queued;
    uint16_t tx_high_water;
    uint16_t rx_high_water;
} ZdtStepperStats_t;

typedef struct {
    uint32_t response_sequence;
    uint8_t last_function;
    ZdtStepperResponse_t last_response;
    /* Emm speed response: sign plus magnitude in whole RPM. */
    bool speed_valid;
    bool speed_negative;
    uint16_t speed_rpm;
    /* Emm position response: sign plus magnitude, 65536 counts/revolution. */
    bool position_valid;
    bool position_negative;
    uint32_t position_raw;
    bool motor_status_valid;
    uint8_t motor_status;
    bool homing_status_valid;
    uint8_t homing_status;
    ZdtStepperStats_t stats;
} ZdtStepperSnapshot_t;

void ZdtStepper_Init(void);
void ZdtStepper_Poll(void);
void ZdtStepper_GetSnapshot(ZdtStepperSnapshot_t *snapshot);

ZdtStepperResult_t ZdtStepper_Enable(bool enabled);
ZdtStepperResult_t ZdtStepper_Stop(void);
/* Emergency operations preempt normal frames at the next frame boundary. */
ZdtStepperResult_t ZdtStepper_EmergencyStop(bool disable_after_stop);
/* speed_rpm: 0..3000; acceleration: 0..255 (0 starts directly). */
ZdtStepperResult_t ZdtStepper_RunVelocity(ZdtStepperDirection_t direction,
                                          uint16_t speed_rpm,
                                          uint8_t acceleration);
ZdtStepperResult_t ZdtStepper_MovePosition(
    ZdtStepperDirection_t direction, uint16_t speed_rpm,
    uint8_t acceleration, uint32_t pulses,
    ZdtStepperPositionMode_t mode);
ZdtStepperResult_t ZdtStepper_ZeroCurrentPosition(void);
ZdtStepperResult_t ZdtStepper_ClearProtection(void);
ZdtStepperResult_t ZdtStepper_StartHoming(ZdtStepperHomeMode_t mode);
ZdtStepperResult_t ZdtStepper_AbortHoming(void);
ZdtStepperResult_t ZdtStepper_ReadSpeed(void);
ZdtStepperResult_t ZdtStepper_ReadPosition(void);
ZdtStepperResult_t ZdtStepper_ReadMotorStatus(void);
ZdtStepperResult_t ZdtStepper_ReadHomingStatus(void);

#endif
