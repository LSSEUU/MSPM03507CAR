#include "app_remote.h"

#include "app_manual_drive.h"
#include "diagnostics.h"
#include "heading.h"
#include "ir_receiver.h"
#include "remote_config.h"
#include "safety.h"
#include "system_state.h"
#include "system_time.h"
#include "uart_debug.h"
#include "wheel_speed.h"

#include <string.h>

static AppRemoteSnapshot_t remote;
static AppRemoteConfig_t active_config;
static uint8_t remote_notice_flags;
static bool legacy_text_enabled;

#define APP_REMOTE_NOTICE_HEARTBEAT_STOP (1U << 0)
#define APP_REMOTE_NOTICE_DEMO_TIMEOUT   (1U << 1)
#define APP_REMOTE_NOTICE_CAPTURE_END    (1U << 2)

static const char k_notice_heartbeat_stop[] =
    "IR manual heartbeat expired; normal brake to IDLE.\r\n";
static const char k_notice_demo_timeout[] =
    "IR demo confirmation timed out.\r\n";
static const char k_notice_capture_end[] =
    "IR key capture ended after 30 s.\r\n";

static bool AppRemote_IsFinite(float value)
{
    return (value == value) &&
           (value <= 3.402823466e+38F) &&
           (value >= -3.402823466e+38F);
}

void AppRemote_GetDefaultConfig(AppRemoteConfig_t *config)
{
    if (config == NULL) return;
    config->speed_low_mm_s = (float)IR_REMOTE_SPEED_LOW_MM_S;
    config->speed_medium_mm_s = (float)IR_REMOTE_SPEED_DEFAULT_MM_S;
    config->speed_high_mm_s = (float)IR_REMOTE_SPEED_HIGH_MM_S;
    config->heading_hold = false;
}

void AppRemote_GetConfig(AppRemoteConfig_t *config)
{
    if (config != NULL) *config = active_config;
}

bool AppRemote_ApplyConfig(const AppRemoteConfig_t *config)
{
    if ((config == NULL) || (remote.state != APP_REMOTE_IDLE) ||
        !AppManualDrive_IsIdle() ||
        !AppRemote_IsFinite(config->speed_low_mm_s) ||
        !AppRemote_IsFinite(config->speed_medium_mm_s) ||
        !AppRemote_IsFinite(config->speed_high_mm_s) ||
        (config->speed_low_mm_s < 50.0f) ||
        (config->speed_high_mm_s > 400.0f) ||
        (config->speed_low_mm_s > config->speed_medium_mm_s) ||
        (config->speed_medium_mm_s > config->speed_high_mm_s)) return false;
    active_config = *config;
    remote.speed_mm_s = (uint16_t)(active_config.speed_medium_mm_s + 0.5f);
    remote.heading_hold_enabled = active_config.heading_hold;
    remote.heading_hold_active = false;
    return true;
}

static bool AppRemote_IsDirection(uint8_t command,
                                  AppRemoteDirection_t *direction)
{
    if (command == IR_REMOTE_KEY_UP) *direction = APP_REMOTE_DIRECTION_FORWARD;
    else if (command == IR_REMOTE_KEY_DOWN) *direction = APP_REMOTE_DIRECTION_REVERSE;
    else if (command == IR_REMOTE_KEY_LEFT) *direction = APP_REMOTE_DIRECTION_LEFT;
    else if (command == IR_REMOTE_KEY_RIGHT) *direction = APP_REMOTE_DIRECTION_RIGHT;
    else return false;
    return true;
}

static void AppRemote_Targets(AppRemoteDirection_t direction,
                              float *left, float *right)
{
    float speed = (float)remote.speed_mm_s;
    *left = 0.0f;
    *right = 0.0f;
    if (direction == APP_REMOTE_DIRECTION_FORWARD) {
        *left = speed; *right = speed;
    } else if (direction == APP_REMOTE_DIRECTION_REVERSE) {
        *left = -speed; *right = -speed;
    } else if (direction == APP_REMOTE_DIRECTION_LEFT) {
        *left = -speed; *right = speed;
    } else if (direction == APP_REMOTE_DIRECTION_RIGHT) {
        *left = speed; *right = -speed;
    }
}

static void AppRemote_StopManual(uint32_t now_ms)
{
    if (remote.heading_hold_active) Heading_Stop();
    (void)AppManualDrive_Stop(APP_MANUAL_SOURCE_IR,
                              APP_MANUAL_STOP_RELEASE, now_ms);
    remote.state = APP_REMOTE_IDLE;
    remote.direction = APP_REMOTE_DIRECTION_NONE;
    remote.heading_hold_active = false;
}

static bool AppRemote_Drive(AppRemoteDirection_t direction, uint32_t now_ms)
{
    float left;
    float right;
    AppManualDriveSnapshot_t manual;
    if (remote.state == APP_REMOTE_PENDING_DEMO) {
        (void)Diagnostics_ActionCancel(now_ms);
        remote.state = APP_REMOTE_IDLE;
    }
    if (remote.state == APP_REMOTE_IDLE) {
        if (Safety_HasFault()) {
            remote.action_rejects++;
            return false;
        }
    } else if (remote.state != APP_REMOTE_MANUAL) {
        remote.action_rejects++;
        return false;
    }

    AppRemote_Targets(direction, &left, &right);
    if (!AppManualDrive_Submit(APP_MANUAL_SOURCE_IR,
            (int16_t)left, (int16_t)right, IR_REMOTE_HEARTBEAT_MS, 0U,
            0U, now_ms)) {
        remote.action_rejects++;
        return false;
    }
    remote.state = APP_REMOTE_MANUAL;
    if (remote.heading_hold_enabled &&
        ((direction == APP_REMOTE_DIRECTION_FORWARD) ||
         (direction == APP_REMOTE_DIRECTION_REVERSE))) {
        float hold_speed = (direction == APP_REMOTE_DIRECTION_FORWARD) ?
            (float)remote.speed_mm_s : -(float)remote.speed_mm_s;
        bool same_straight_direction = remote.heading_hold_active &&
            (remote.direction == direction);
        if (remote.heading_hold_active && !same_straight_direction) {
            Heading_Stop();
            remote.heading_hold_active = false;
        }
        if (same_straight_direction)
            remote.heading_hold_active = Heading_SetHoldSpeed(hold_speed);
        else
            remote.heading_hold_active = Heading_StartHold(hold_speed);
        if (remote.heading_hold_active) {
            HeadingSnapshot_t heading;
            Heading_GetSnapshot(&heading);
            left = heading.left_target_mm_s;
            right = heading.right_target_mm_s;
        }
    } else if (remote.heading_hold_active) {
        Heading_Stop();
        remote.heading_hold_active = false;
    }
    if (!AppManualDrive_Submit(APP_MANUAL_SOURCE_IR,
            (int16_t)left, (int16_t)right, IR_REMOTE_HEARTBEAT_MS,
            remote.heading_hold_active ?
                SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO) : 0U,
            0U, now_ms)) {
        AppManualDrive_GetSnapshot(&manual);
        if (!manual.active || (manual.source != APP_MANUAL_SOURCE_IR)) {
            remote.state = APP_REMOTE_IDLE;
            remote.heading_hold_active = false;
        }
        remote.action_rejects++;
        return false;
    }
    remote.direction = direction;
    remote.last_drive_event_ms = now_ms;
    return true;
}

static void AppRemote_AdjustSpeed(bool increase)
{
    if ((remote.state != APP_REMOTE_IDLE) || Safety_HasFault() ||
        !AppManualDrive_IsIdle() ||
        (SystemState_Get() != SYSTEM_STATE_IDLE) ||
        !Diagnostics_ActionIsIdle()) {
        remote.action_rejects++;
        return;
    }
    if (increase) {
        if ((float)remote.speed_mm_s < active_config.speed_medium_mm_s)
            remote.speed_mm_s = (uint16_t)(
                active_config.speed_medium_mm_s + 0.5f);
        else remote.speed_mm_s = (uint16_t)(
            active_config.speed_high_mm_s + 0.5f);
    } else {
        if ((float)remote.speed_mm_s > active_config.speed_medium_mm_s)
            remote.speed_mm_s = (uint16_t)(
                active_config.speed_medium_mm_s + 0.5f);
        else remote.speed_mm_s = (uint16_t)(
            active_config.speed_low_mm_s + 0.5f);
    }
    debug_printf("IR speed=%u mm/s\r\n", remote.speed_mm_s);
}

static void AppRemote_ArmDemo(uint8_t command, uint32_t now_ms)
{
    DiagnosticsAction_t action;
    if (remote.state != APP_REMOTE_IDLE) {
        remote.action_rejects++;
        return;
    }
    if (command == IR_REMOTE_KEY_1) action = DIAGNOSTICS_ACTION_STRAIGHT;
    else if (command == IR_REMOTE_KEY_2) action = DIAGNOSTICS_ACTION_TURN_POSITIVE;
    else if (command == IR_REMOTE_KEY_3) action = DIAGNOSTICS_ACTION_TURN_NEGATIVE;
    else action = DIAGNOSTICS_ACTION_LINE;

    if (!Diagnostics_ActionArm(action, (float)remote.speed_mm_s,
            IR_REMOTE_STRAIGHT_DEMO_MS, now_ms)) {
        remote.action_rejects++;
        debug_println("IR demo arm rejected by current safety/precondition state.");
        return;
    }
    remote.state = APP_REMOTE_PENDING_DEMO;
    remote.pending_deadline_ms = now_ms + IR_REMOTE_PENDING_MS;
    debug_println("IR demo armed; press OK within 10 s to confirm.");
}

bool AppRemote_NormalStop(uint32_t now_ms)
{
    bool stopped = false;
    if (Safety_HasFault()) return false;
    if (remote.state == APP_REMOTE_MANUAL) {
        AppRemote_StopManual(now_ms);
        stopped = true;
    }
    if (Diagnostics_ActionCancel(now_ms)) stopped = true;
    if (remote.state == APP_REMOTE_PENDING_DEMO ||
        remote.state == APP_REMOTE_DEMO_RUNNING) stopped = true;
    if (remote.state != APP_REMOTE_CAPTURE) {
        remote.state = APP_REMOTE_IDLE;
        remote.direction = APP_REMOTE_DIRECTION_NONE;
    }
    return stopped;
}

static void AppRemote_HandleFull(const IrNecEvent_t *event)
{
    AppRemoteDirection_t direction;
    uint32_t now_ms = event->timestamp_ms;
    if (event->address != IR_REMOTE_EXPECTED_ADDRESS) {
        remote.address_rejects++;
        return;
    }
    if (AppRemote_IsDirection(event->command, &direction)) {
        (void)AppRemote_Drive(direction, now_ms);
    } else if (event->command == IR_REMOTE_KEY_STAR) {
        AppRemote_AdjustSpeed(false);
    } else if (event->command == IR_REMOTE_KEY_HASH) {
        AppRemote_AdjustSpeed(true);
    } else if (event->command == IR_REMOTE_KEY_OK) {
        if (remote.state == APP_REMOTE_PENDING_DEMO) {
            if (Diagnostics_ActionConfirm(now_ms)) {
                remote.state = APP_REMOTE_DEMO_RUNNING;
                debug_println("IR demo confirmed and running.");
            } else {
                remote.state = APP_REMOTE_IDLE;
                remote.action_rejects++;
            }
        } else {
            (void)AppRemote_NormalStop(now_ms);
        }
    } else if (event->command == IR_REMOTE_KEY_0) {
        (void)AppRemote_NormalStop(now_ms);
    } else if ((event->command == IR_REMOTE_KEY_1) ||
               (event->command == IR_REMOTE_KEY_2) ||
               (event->command == IR_REMOTE_KEY_3) ||
               (event->command == IR_REMOTE_KEY_4)) {
        AppRemote_ArmDemo(event->command, now_ms);
    } else if ((event->command == IR_REMOTE_KEY_5) ||
               (event->command == IR_REMOTE_KEY_6) ||
               (event->command == IR_REMOTE_KEY_7) ||
               (event->command == IR_REMOTE_KEY_8) ||
               (event->command == IR_REMOTE_KEY_9)) {
        /* Reserved in the first release. */
    } else {
        remote.unknown_commands++;
    }
}

void AppRemote_Init(void)
{
    memset(&remote, 0, sizeof(remote));
    remote_notice_flags = 0U;
    legacy_text_enabled = true;
    AppRemote_GetDefaultConfig(&active_config);
    remote.speed_mm_s = (uint16_t)(active_config.speed_medium_mm_s + 0.5f);
    remote.actions_enabled = (IR_REMOTE_KEYMAP_VERIFIED != 0);
    remote.heading_hold_enabled = active_config.heading_hold;
    IrReceiver_Init();
}

void AppRemote_SetLegacyTextEnabled(bool enabled)
{
    remote_notice_flags = 0U;
    legacy_text_enabled = enabled;
}

static void AppRemote_ServiceNotice(void)
{
    if (!legacy_text_enabled) {
        remote_notice_flags = 0U;
        return;
    }
    if ((remote_notice_flags & APP_REMOTE_NOTICE_HEARTBEAT_STOP) != 0U) {
        if (debug_try_write(k_notice_heartbeat_stop,
                sizeof(k_notice_heartbeat_stop) - 1U)) {
            remote_notice_flags &=
                (uint8_t)~APP_REMOTE_NOTICE_HEARTBEAT_STOP;
        }
    } else if ((remote_notice_flags & APP_REMOTE_NOTICE_DEMO_TIMEOUT) != 0U) {
        if (debug_try_write(k_notice_demo_timeout,
                sizeof(k_notice_demo_timeout) - 1U)) {
            remote_notice_flags &=
                (uint8_t)~APP_REMOTE_NOTICE_DEMO_TIMEOUT;
        }
    } else if ((remote_notice_flags & APP_REMOTE_NOTICE_CAPTURE_END) != 0U) {
        if (debug_try_write(k_notice_capture_end,
                sizeof(k_notice_capture_end) - 1U)) {
            remote_notice_flags &=
                (uint8_t)~APP_REMOTE_NOTICE_CAPTURE_END;
        }
    }
}

void AppRemote_Poll(void)
{
    IrNecEvent_t event;
    uint32_t now_ms = SystemTime_Now();
    IrReceiver_Service(now_ms);
    while (IrReceiver_Read(&event)) {
        if (remote.state == APP_REMOTE_CAPTURE) {
            if (event.type == IR_NEC_EVENT_FULL) {
                remote.captured_keys++;
                debug_printf("IR KEY addr=0x%02X cmd=0x%02X raw=0x%08lX\r\n",
                    event.address, event.command, (unsigned long)event.raw);
            }
            continue;
        }
        if (!remote.actions_enabled) continue;
        if (event.type == IR_NEC_EVENT_FULL) {
            AppRemote_HandleFull(&event);
        } else if (event.type == IR_NEC_EVENT_REPEAT) {
            AppRemoteDirection_t direction;
            if ((event.address == IR_REMOTE_EXPECTED_ADDRESS) &&
                AppRemote_IsDirection(event.command, &direction) &&
                (remote.state == APP_REMOTE_MANUAL)) {
                (void)AppRemote_Drive(direction, event.timestamp_ms);
            }
        }
    }
    AppRemote_ServiceNotice();
}

void AppRemote_Task20ms(uint32_t now_ms)
{
    AppManualDriveSnapshot_t manual;
    IrReceiver_Service(now_ms);
    AppManualDrive_GetSnapshot(&manual);
    if ((remote.state == APP_REMOTE_MANUAL) &&
        (!manual.active || (manual.source != APP_MANUAL_SOURCE_IR))) {
        if (remote.heading_hold_active) Heading_Stop();
        remote.state = APP_REMOTE_IDLE;
        remote.direction = APP_REMOTE_DIRECTION_NONE;
        remote.heading_hold_active = false;
    }
    if ((remote.state == APP_REMOTE_MANUAL) &&
        ((uint32_t)(now_ms - remote.last_drive_event_ms) >=
            IR_REMOTE_HEARTBEAT_MS)) {
        AppRemote_StopManual(now_ms);
        remote.heartbeat_stops++;
        if (legacy_text_enabled)
            remote_notice_flags |= APP_REMOTE_NOTICE_HEARTBEAT_STOP;
    } else if ((remote.state == APP_REMOTE_PENDING_DEMO) &&
               SystemTime_Reached(now_ms, remote.pending_deadline_ms)) {
        (void)Diagnostics_ActionCancel(now_ms);
        remote.state = APP_REMOTE_IDLE;
        if (legacy_text_enabled)
            remote_notice_flags |= APP_REMOTE_NOTICE_DEMO_TIMEOUT;
    } else if ((remote.state == APP_REMOTE_DEMO_RUNNING) &&
               Diagnostics_ActionIsIdle() &&
               (SystemState_Get() == SYSTEM_STATE_IDLE)) {
        remote.state = APP_REMOTE_IDLE;
    } else if ((remote.state == APP_REMOTE_CAPTURE) &&
               SystemTime_Reached(now_ms, remote.capture_deadline_ms)) {
        remote.state = APP_REMOTE_IDLE;
        if (legacy_text_enabled)
            remote_notice_flags |= APP_REMOTE_NOTICE_CAPTURE_END;
    }
    if (Safety_HasFault() && (remote.state != APP_REMOTE_CAPTURE)) {
        if (remote.heading_hold_active) Heading_Stop();
        remote.state = APP_REMOTE_IDLE;
        remote.direction = APP_REMOTE_DIRECTION_NONE;
        remote.heading_hold_active = false;
    }
}

bool AppRemote_StartCapture(uint32_t now_ms)
{
    if ((remote.state != APP_REMOTE_IDLE) || Safety_HasFault() ||
        !AppManualDrive_IsIdle() ||
        (SystemState_Get() != SYSTEM_STATE_IDLE) ||
        !Diagnostics_ActionIsIdle()) return false;
    remote.state = APP_REMOTE_CAPTURE;
    remote.capture_deadline_ms = now_ms + IR_REMOTE_CAPTURE_MS;
    remote.captured_keys = 0U;
    return true;
}

void AppRemote_GetSnapshot(AppRemoteSnapshot_t *snapshot)
{
    if (snapshot == 0) return;
    *snapshot = remote;
}
