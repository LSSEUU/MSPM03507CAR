#include "motor.h"
#include "reset_diag.h"
#include "safety.h"
#include "scheduler.h"
#include "system_state.h"
#include "vehicle_config.h"

#include <assert.h>
#include <stdint.h>

static int16_t motor_a;
static int16_t motor_b;
static uint32_t motor_stop_count;
static ResetDiagCheckpoint_t reset_checkpoint;

void ResetDiag_Checkpoint(ResetDiagCheckpoint_t checkpoint)
{
    reset_checkpoint = checkpoint;
}

void Motor_Init(void) {}

void Motor_SetPWM(int16_t a, int16_t b)
{
    motor_a = a;
    motor_b = b;
}

void Motor_Stop(void)
{
    motor_a = 0;
    motor_b = 0;
    motor_stop_count++;
}

void Motor_GetStats(MotorStats_t *stats)
{
    (void)stats;
}

static void test_state_machine(void)
{
    SystemState_Init();
    assert(SystemState_Get() == SYSTEM_STATE_BOOT);
    assert(!SystemState_RequestRun());
    assert(SystemState_BeginSelfTest());
    assert(SystemState_CompleteSelfTest(true));
    assert(SystemState_Get() == SYSTEM_STATE_IDLE);
    assert(SystemState_RequestRun());
    assert(SystemState_RequestIdle());
    SystemState_EnterFault();
    assert(!SystemState_RequestRun());
    assert(SystemState_ClearFault());
    assert(SystemState_Get() == SYSTEM_STATE_IDLE);
}

static void test_scheduler(void)
{
    Scheduler_Init();
    for (uint32_t now = 1U; now <= 100U; now++) {
        Scheduler_TickISR(now);
        for (uint32_t task = 0U; task < (uint32_t)SCHEDULER_TASK_COUNT; task++) {
            if (Scheduler_Take((SchedulerTask_t)task))
                Scheduler_Complete((SchedulerTask_t)task);
        }
    }
    assert(Scheduler_TakeMissedMask() == 0U);
    assert(Scheduler_ConsumeHealthyWindow());

    Scheduler_Init();
    Scheduler_TickISR(5U);
    Scheduler_TickISR(10U);
    assert((Scheduler_TakeMissedMask() &
            SCHEDULER_TASK_BIT(SCHEDULER_TASK_5MS)) != 0U);
    assert(!Scheduler_ConsumeHealthyWindow());
}

static void enter_idle(void)
{
    SystemState_Init();
    assert(SystemState_BeginSelfTest());
    assert(SystemState_CompleteSelfTest(true));
}

static void test_action_timeout_and_motor_veto(void)
{
    SafetySnapshot_t snapshot;
    enter_idle();
    Safety_Init(0U);
    assert(!Safety_RequestMotor(100, 100));
    assert(motor_a == 0 && motor_b == 0);

    assert(SystemState_RequestRun());
    Safety_BeginAction(10U, 100U, 0U);
    assert(Safety_RequestMotor(200, -200));
    assert(motor_a == 200 && motor_b == -200);
    Safety_Evaluate(109U);
    assert(!Safety_HasFault());
    Safety_Evaluate(110U);
    Safety_GetSnapshot(&snapshot);
    assert((snapshot.fault_bits & SAFETY_FAULT_ACTION_TIMEOUT) != 0U);
    assert(reset_checkpoint == RESET_DIAG_CHECKPOINT_SAFETY_FAULT);
    assert(SystemState_Get() == SYSTEM_STATE_FAULT);
    assert(motor_a == 0 && motor_b == 0);
    assert(Safety_ClearFaults(111U));
    assert(SystemState_Get() == SYSTEM_STATE_IDLE);
}

static void test_sensor_timeout_and_emergency_stop(void)
{
    SafetySnapshot_t snapshot;
    enter_idle();
    Safety_Init(0U);
    assert(SystemState_RequestRun());
    Safety_BeginAction(1000U, 1000U, SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO));
    Safety_NotifySensor(SAFETY_SENSOR_GYRO, 1000U);
    assert(Safety_RequestMotor(150, 150));
    Safety_Evaluate(1249U);
    assert(!Safety_HasFault());
    Safety_Evaluate(1250U);
    Safety_GetSnapshot(&snapshot);
    assert((snapshot.fault_bits & SAFETY_FAULT_GYRO_TIMEOUT) != 0U);
    assert(motor_a == 0 && motor_b == 0);

    assert(Safety_ClearFaults(1251U));
    assert(SystemState_RequestRun());
    Safety_BeginAction(1251U, 1000U, 0U);
    assert(Safety_RequestMotor(100, 100));
    Safety_EmergencyStop(1260U);
    Safety_GetSnapshot(&snapshot);
    assert((snapshot.fault_bits & SAFETY_FAULT_EMERGENCY_STOP) != 0U);
    assert(snapshot.first_fault_ms == 1260U);
    assert(motor_a == 0 && motor_b == 0);
}

static void test_continuous_action_keeps_sensor_watchdog(void)
{
    SafetySnapshot_t snapshot;
    enter_idle();
    Safety_Init(0U);
    assert(SystemState_RequestRun());
    Safety_BeginContinuousAction(1000U,
        SAFETY_SENSOR_BIT(SAFETY_SENSOR_GRAYSCALE));
    Safety_NotifySensor(SAFETY_SENSOR_GRAYSCALE, 1000U);
    Safety_Evaluate(1299U);
    Safety_GetSnapshot(&snapshot);
    assert(snapshot.action_active);
    assert(snapshot.action_deadline_ms == 0U);
    assert(!Safety_HasFault());
    Safety_NotifySensor(SAFETY_SENSOR_GRAYSCALE, 100000U);
    Safety_Evaluate(100000U);
    assert(!Safety_HasFault());
    Safety_Evaluate(100000U + GRAYSCALE_TIMEOUT_MS);
    Safety_GetSnapshot(&snapshot);
    assert((snapshot.fault_bits & SAFETY_FAULT_GRAYSCALE_TIMEOUT) != 0U);
}

int main(void)
{
    test_state_machine();
    test_scheduler();
    test_action_timeout_and_motor_veto();
    test_sensor_timeout_and_emergency_stop();
    test_continuous_action_keeps_sensor_watchdog();
    assert(motor_stop_count > 0U);
    return 0;
}
