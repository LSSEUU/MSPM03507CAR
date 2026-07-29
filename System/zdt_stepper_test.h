#ifndef ZDT_STEPPER_TEST_H
#define ZDT_STEPPER_TEST_H

#include "zdt_stepper.h"

#include <stdbool.h>
#include <stdint.h>

#define ZDT_STEPPER_REFRESH_SPEED        (1U << 0)
#define ZDT_STEPPER_REFRESH_POSITION     (1U << 1)
#define ZDT_STEPPER_REFRESH_MOTOR        (1U << 2)
#define ZDT_STEPPER_REFRESH_HOMING       (1U << 3)
#define ZDT_STEPPER_REFRESH_ALL          (0x0FU)

/* WRONG_STATE detail bits for rejected motion commands. */
#define ZDT_STEPPER_NOT_READY_NO_STATUS          (1UL << 0)
#define ZDT_STEPPER_NOT_READY_STATUS_STALE       (1UL << 1)
#define ZDT_STEPPER_NOT_READY_DISABLED           (1UL << 2)
#define ZDT_STEPPER_NOT_READY_STALLED            (1UL << 3)
#define ZDT_STEPPER_NOT_READY_STALL_PROTECTION   (1UL << 4)
#define ZDT_STEPPER_NOT_READY_LEFT_LIMIT         (1UL << 5)
#define ZDT_STEPPER_NOT_READY_RIGHT_LIMIT        (1UL << 6)
#define ZDT_STEPPER_NOT_READY_POWER_LOSS         (1UL << 7)
#define ZDT_STEPPER_NOT_READY_INTERNAL            (1UL << 8)

typedef enum {
    ZDT_TEST_COMMAND_ENABLE = 1,
    ZDT_TEST_COMMAND_DISABLE = 2,
    ZDT_TEST_COMMAND_STOP = 3,
    ZDT_TEST_COMMAND_VELOCITY = 4,
    ZDT_TEST_COMMAND_POSITION = 5
} ZdtStepperTestCommand_t;

typedef enum {
    ZDT_TEST_STATE_IDLE = 0,
    ZDT_TEST_STATE_WAITING_ACK = 1,
    ZDT_TEST_STATE_RUNNING = 2,
    ZDT_TEST_STATE_COMPLETED = 3,
    ZDT_TEST_STATE_ERROR = 4
} ZdtStepperTestState_t;

typedef enum {
    ZDT_TEST_KIND_NONE = 0,
    ZDT_TEST_KIND_VELOCITY = 1,
    ZDT_TEST_KIND_POSITION = 2
} ZdtStepperTestKind_t;

typedef enum {
    ZDT_TEST_STOP_NONE = 0,
    ZDT_TEST_STOP_USER = 1,
    ZDT_TEST_STOP_DISABLED = 2,
    ZDT_TEST_STOP_DURATION = 3,
    ZDT_TEST_STOP_POSITION_REACHED = 4,
    ZDT_TEST_STOP_TIMEOUT = 5,
    ZDT_TEST_STOP_COMMUNICATION = 6,
    ZDT_TEST_STOP_DEVICE_REJECTED = 7,
    ZDT_TEST_STOP_STARTUP = 8
} ZdtStepperTestStopReason_t;

typedef struct {
    uint32_t command_id;
    ZdtStepperTestCommand_t command;
    bool confirmed;
    ZdtStepperDirection_t direction;
    uint8_t acceleration;
    uint16_t speed_rpm;
    uint16_t duration_or_timeout_ms;
    uint32_t pulses;
    ZdtStepperPositionMode_t position_mode;
} ZdtStepperTestRequest_t;

typedef struct {
    ZdtStepperSnapshot_t device;
    uint32_t time_ms;
    uint32_t response_age_ms;
    uint16_t valid_flags;
    int16_t speed_rpm;
    int64_t position_raw;
    uint32_t command_id;
    uint32_t test_id;
    uint32_t remaining_ms;
    ZdtStepperTestState_t state;
    ZdtStepperTestKind_t kind;
    ZdtStepperDirection_t direction;
    uint8_t acceleration;
    uint16_t requested_speed_rpm;
    ZdtStepperPositionMode_t position_mode;
    ZdtStepperTestStopReason_t stop_reason;
    uint32_t pulses;
} ZdtStepperTestSnapshot_t;

void ZdtStepperTest_Init(uint32_t now_ms);
void ZdtStepperTest_Poll(uint32_t now_ms);
void ZdtStepperTest_RequestRefresh(uint8_t mask);
ZdtStepperResult_t ZdtStepperTest_Command(
    const ZdtStepperTestRequest_t *request, uint32_t now_ms);
uint32_t ZdtStepperTest_GetLastRejectDetail(void);
void ZdtStepperTest_GetSnapshot(ZdtStepperTestSnapshot_t *snapshot,
                                uint32_t now_ms);

#endif
