#include "app_remote.h"
#include "diagnostics.h"
#include "heading.h"
#include "ir_receiver.h"
#include "remote_config.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "wheel_speed.h"

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define TEST_QUEUE_SIZE 32U

static IrNecEvent_t test_queue[TEST_QUEUE_SIZE];
static uint8_t test_head;
static uint8_t test_tail;
static uint32_t test_now;
static bool test_fault;
static SystemState_t test_system_state;
static bool test_diag_idle;
static bool test_diag_armed;
static bool test_wheel_enabled;
static float test_left_target;
static float test_right_target;
static uint32_t test_safety_timeout;
static uint32_t test_required_sensor_mask;
static uint32_t test_arm_calls;
static uint32_t test_confirm_calls;
static uint32_t test_cancel_calls;
static DiagnosticsAction_t test_last_action;
static HeadingSnapshot_t test_heading;
static uint32_t test_heading_start_calls;
static uint32_t test_heading_stop_calls;

static void inject(IrNecEventType_t type, uint8_t address, uint8_t command,
                   uint32_t timestamp_ms)
{
    IrNecEvent_t event = { type, address, command, 0U, timestamp_ms };
    test_queue[test_head++ % TEST_QUEUE_SIZE] = event;
}

static void reset_fixture(void)
{
    test_head = 0U;
    test_tail = 0U;
    test_now = 0U;
    test_fault = false;
    test_system_state = SYSTEM_STATE_IDLE;
    test_diag_idle = true;
    test_diag_armed = false;
    test_wheel_enabled = false;
    test_left_target = 0.0f;
    test_right_target = 0.0f;
    test_safety_timeout = 0U;
    test_required_sensor_mask = 0U;
    test_arm_calls = 0U;
    test_confirm_calls = 0U;
    test_cancel_calls = 0U;
    memset(&test_heading, 0, sizeof(test_heading));
    test_heading_start_calls = 0U;
    test_heading_stop_calls = 0U;
    AppRemote_Init();
}

void IrReceiver_Init(void) {}
bool IrReceiver_Read(IrNecEvent_t *event)
{
    if (test_tail == test_head) return false;
    *event = test_queue[test_tail++ % TEST_QUEUE_SIZE];
    return true;
}
void IrReceiver_Service(uint32_t now_ms) { (void)now_ms; }
void IrReceiver_GetStats(IrReceiverStats_t *stats) { memset(stats, 0, sizeof(*stats)); }

uint32_t SystemTime_Now(void) { return test_now; }
bool SystemTime_Reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

SystemState_t SystemState_Get(void) { return test_system_state; }
bool SystemState_RequestRun(void)
{
    if (test_system_state != SYSTEM_STATE_IDLE) return false;
    test_system_state = SYSTEM_STATE_RUN;
    return true;
}
bool SystemState_RequestIdle(void)
{
    if (test_system_state == SYSTEM_STATE_FAULT) return false;
    test_system_state = SYSTEM_STATE_IDLE;
    return true;
}

bool Safety_HasFault(void) { return test_fault; }
void Safety_BeginAction(uint32_t now_ms, uint32_t timeout_ms,
                        uint32_t required_sensor_mask)
{
    (void)now_ms;
    test_safety_timeout = timeout_ms;
    test_required_sensor_mask = required_sensor_mask;
}
void Safety_EndAction(void)
{
    test_safety_timeout = 0U;
    test_required_sensor_mask = 0U;
}

void WheelSpeed_Enable(void) { test_wheel_enabled = true; }
void WheelSpeed_Disable(void)
{
    test_wheel_enabled = false; test_left_target = 0.0f; test_right_target = 0.0f;
}
void WheelSpeed_SetTarget(float left_mm_s, float right_mm_s)
{
    test_left_target = left_mm_s; test_right_target = right_mm_s;
}

bool Heading_StartHold(float base_speed_mm_s)
{
    test_heading_start_calls++;
    test_heading.mode = HEADING_MODE_HOLD;
    test_heading.left_target_mm_s = base_speed_mm_s + 10.0f;
    test_heading.right_target_mm_s = base_speed_mm_s - 10.0f;
    return true;
}
bool Heading_SetHoldSpeed(float base_speed_mm_s)
{
    if (test_heading.mode != HEADING_MODE_HOLD) return false;
    test_heading.left_target_mm_s = base_speed_mm_s + 10.0f;
    test_heading.right_target_mm_s = base_speed_mm_s - 10.0f;
    return true;
}
void Heading_Stop(void)
{
    test_heading_stop_calls++;
    test_heading.mode = HEADING_MODE_IDLE;
}
void Heading_GetSnapshot(HeadingSnapshot_t *snapshot)
{
    *snapshot = test_heading;
}

bool Diagnostics_ActionIsIdle(void) { return test_diag_idle; }
bool Diagnostics_ActionArm(DiagnosticsAction_t action,
                           float speed, uint32_t duration, uint32_t now_ms)
{
    (void)speed; (void)duration; (void)now_ms;
    if (!test_diag_idle) return false;
    test_arm_calls++;
    test_last_action = action;
    test_diag_idle = false;
    test_diag_armed = true;
    return true;
}
bool Diagnostics_ActionConfirm(uint32_t now_ms)
{
    (void)now_ms;
    if (!test_diag_armed) return false;
    test_confirm_calls++;
    test_diag_armed = false;
    test_system_state = SYSTEM_STATE_RUN;
    return true;
}
bool Diagnostics_ActionCancel(uint32_t now_ms)
{
    (void)now_ms;
    if (test_diag_idle) return false;
    test_cancel_calls++;
    test_diag_idle = true;
    test_diag_armed = false;
    test_system_state = SYSTEM_STATE_IDLE;
    return true;
}

void debug_printf(const char *format, ...) { (void)format; }
void debug_println(const char *text) { (void)text; }
int debug_try_write(const char *data, size_t length)
{
    (void)data;
    return (length > 0U) ? 1 : 0;
}

static void test_four_directions_and_heartbeat(void)
{
    static const uint8_t commands[] = {
        IR_REMOTE_KEY_UP, IR_REMOTE_KEY_DOWN,
        IR_REMOTE_KEY_LEFT, IR_REMOTE_KEY_RIGHT
    };
    static const float left[] = { 160.0f, -160.0f, -160.0f, 160.0f };
    static const float right[] = { 160.0f, -160.0f, 160.0f, -160.0f };
    uint32_t i;
    for (i = 0U; i < 4U; i++) {
        AppRemoteSnapshot_t snapshot;
        reset_fixture();
        inject(IR_NEC_EVENT_FULL, 0x00U, commands[i], 10U);
        AppRemote_Poll();
        AppRemote_GetSnapshot(&snapshot);
        assert(snapshot.state == APP_REMOTE_MANUAL);
        assert(test_wheel_enabled);
        assert(test_left_target == left[i]);
        assert(test_right_target == right[i]);
        assert(test_safety_timeout == 500U);
    }

    reset_fixture();
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_UP, 10U);
    AppRemote_Poll();
    AppRemote_Task20ms(259U);
    assert(test_wheel_enabled);
    inject(IR_NEC_EVENT_REPEAT, 0x00U, IR_REMOTE_KEY_UP, 260U);
    AppRemote_Poll();
    AppRemote_Task20ms(509U);
    assert(test_wheel_enabled);
    AppRemote_Task20ms(510U);
    assert(!test_wheel_enabled);
    assert(test_system_state == SYSTEM_STATE_IDLE);
}

static void test_speed_demo_confirmation_and_cancel(void)
{
    AppRemoteSnapshot_t snapshot;
    reset_fixture();
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_STAR, 1U);
    AppRemote_Poll();
    AppRemote_GetSnapshot(&snapshot);
    assert(snapshot.speed_mm_s == 100U);
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_HASH, 2U);
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_HASH, 3U);
    AppRemote_Poll();
    AppRemote_GetSnapshot(&snapshot);
    assert(snapshot.speed_mm_s == 220U);

    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_1, 10U);
    AppRemote_Poll();
    AppRemote_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_REMOTE_PENDING_DEMO);
    assert(test_arm_calls == 1U);
    assert(test_last_action == DIAGNOSTICS_ACTION_STRAIGHT);
    inject(IR_NEC_EVENT_REPEAT, 0x00U, IR_REMOTE_KEY_OK, 20U);
    AppRemote_Poll();
    assert(test_confirm_calls == 0U);
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_OK, 30U);
    AppRemote_Poll();
    assert(test_confirm_calls == 1U);
    AppRemote_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_REMOTE_DEMO_RUNNING);
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_0, 40U);
    AppRemote_Poll();
    assert(test_cancel_calls == 1U);
    assert(test_system_state == SYSTEM_STATE_IDLE);
}

static void test_conflicts_capture_fault_and_address_filter(void)
{
    AppRemoteSnapshot_t snapshot;
    reset_fixture();
    inject(IR_NEC_EVENT_FULL, 0x01U, IR_REMOTE_KEY_UP, 1U);
    AppRemote_Poll();
    AppRemote_GetSnapshot(&snapshot);
    assert(snapshot.address_rejects == 1U);
    assert(!test_wheel_enabled);

    test_fault = true;
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_UP, 2U);
    AppRemote_Poll();
    assert(!test_wheel_enabled);
    test_fault = false;
    assert(AppRemote_StartCapture(10U));
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_UP, 11U);
    AppRemote_Poll();
    AppRemote_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_REMOTE_CAPTURE);
    assert(snapshot.captured_keys == 1U);
    assert(!test_wheel_enabled);
    AppRemote_Task20ms(30010U);
    AppRemote_GetSnapshot(&snapshot);
    assert(snapshot.state == APP_REMOTE_IDLE);
}

static void test_optional_heading_hold_only_for_straight_keys(void)
{
    AppRemoteConfig_t config;
    AppRemoteSnapshot_t snapshot;
    reset_fixture();
    AppRemote_GetDefaultConfig(&config);
    config.heading_hold = true;
    assert(AppRemote_ApplyConfig(&config));
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_UP, 10U);
    AppRemote_Poll();
    AppRemote_GetSnapshot(&snapshot);
    assert(snapshot.heading_hold_active);
    assert(test_heading_start_calls == 1U);
    assert(test_left_target == 170.0f);
    assert(test_right_target == 150.0f);
    assert(test_required_sensor_mask ==
           SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO));

    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_RIGHT, 20U);
    AppRemote_Poll();
    AppRemote_GetSnapshot(&snapshot);
    assert(!snapshot.heading_hold_active);
    assert(test_heading_stop_calls == 1U);
    assert(test_left_target == 160.0f);
    assert(test_right_target == -160.0f);
    assert(test_required_sensor_mask == 0U);
}

static void test_production_keymap_gate(void)
{
    AppRemoteSnapshot_t snapshot;
    reset_fixture();
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_UP, 1U);
    AppRemote_Poll();
    AppRemote_GetSnapshot(&snapshot);
    assert(!snapshot.actions_enabled);
    assert(snapshot.state == APP_REMOTE_IDLE);
    assert(!test_wheel_enabled);
    assert(AppRemote_StartCapture(10U));
    inject(IR_NEC_EVENT_FULL, 0x00U, IR_REMOTE_KEY_UP, 11U);
    AppRemote_Poll();
    AppRemote_GetSnapshot(&snapshot);
    assert(snapshot.captured_keys == 1U);
}

int main(void)
{
    if (IR_REMOTE_KEYMAP_VERIFIED != 0) {
        test_four_directions_and_heartbeat();
        test_speed_demo_confirmation_and_cancel();
        test_conflicts_capture_fault_and_address_filter();
        test_optional_heading_hold_only_for_straight_keys();
    } else {
        test_production_keymap_gate();
    }
    return 0;
}
