#include "h_mission_control_profile.h"

#include "heading.h"
#include "line_follow.h"
#include "wheel_speed.h"

#include <stddef.h>
#include <string.h>

/* Exact loaded-chassis controller snapshot verified by the r65 H4 run and
 * the r69 H5 lap.  Mission speeds remain in HMissionConfig so RAM-only host
 * development can still select a lower speed without changing this profile. */
#define H45_WHEEL_KP                         0.70f
#define H45_WHEEL_KI                         2.50f
#define H45_WHEEL_FILTER_ALPHA               0.10f
#define H45_WHEEL_ACCEL_MM_S2              250.0f
#define H4_WHEEL_DECEL_MM_S2               400.0f
#define H5_WHEEL_DECEL_MM_S2               300.0f
#define H45_WHEEL_JERK_MM_S3              1500.0f
#define H45_WHEEL_STATIC_FF_PWM              10.0f
#define H45_WHEEL_VELOCITY_FF_PWM_PER_MM_S    0.75f

#define H45_HEADING_STRAIGHT_KP               3.0f
#define H45_HEADING_STRAIGHT_KD               0.88f
#define H45_HEADING_TURN_KP                   2.0f
#define H45_HEADING_TURN_KD                   0.55f
#define H45_HEADING_RATE_ALPHA                0.35f
#define H45_HEADING_CORRECTION_LIMIT_MM_S   120.0f
#define H45_HEADING_TURN_SPEED_MM_S          150.0f
#define H45_HEADING_SMOOTH_ZONE_DEG            2.0f

#define H45_LINE_SPEED_MIN_MM_S              300.0f
#define H45_LINE_SPEED_MAX_MM_S              400.0f
#define H45_LINE_POSITION_ALPHA                0.35f
#define H45_LINE_DERIVATIVE_ALPHA              0.25f
#define H45_LINE_KP                           55.0f
#define H45_LINE_KD                           10.0f
#define H45_LINE_CORRECTION_LIMIT_MM_S        75.0f
#define H45_LINE_CORRECTION_SLEW_MM_S2       300.0f
#define H45_LINE_CURVE_GAIN                    2.0f
#define H45_LINE_LOST_SEARCH_SPEED_MM_S      105.0f

typedef struct {
    HMissionControlProfileSnapshot_t snapshot;
    WheelSpeedConfig_t saved_wheel;
    HeadingConfig_t saved_heading;
    LineFollowConfig_t saved_line;
} HMissionControlProfileState_t;

static HMissionControlProfileState_t profile;

static void HMissionControlProfile_BuildWheel(WheelSpeedConfig_t *config,
                                               HMissionMode_t mode)
{
    config->kp[WHEEL_SPEED_LEFT] = H45_WHEEL_KP;
    config->ki[WHEEL_SPEED_LEFT] = H45_WHEEL_KI;
    config->kp[WHEEL_SPEED_RIGHT] = H45_WHEEL_KP;
    config->ki[WHEEL_SPEED_RIGHT] = H45_WHEEL_KI;
    config->filter_alpha = H45_WHEEL_FILTER_ALPHA;
    config->accel_mm_s2 = H45_WHEEL_ACCEL_MM_S2;
    config->decel_mm_s2 = (mode == H_MISSION_MODE_H4) ?
        H4_WHEEL_DECEL_MM_S2 : H5_WHEEL_DECEL_MM_S2;
    config->jerk_mm_s3 = H45_WHEEL_JERK_MM_S3;
    config->static_feedforward_pwm[WHEEL_SPEED_LEFT] =
        H45_WHEEL_STATIC_FF_PWM;
    config->velocity_feedforward_pwm_per_mm_s[WHEEL_SPEED_LEFT] =
        H45_WHEEL_VELOCITY_FF_PWM_PER_MM_S;
    config->static_feedforward_pwm[WHEEL_SPEED_RIGHT] =
        H45_WHEEL_STATIC_FF_PWM;
    config->velocity_feedforward_pwm_per_mm_s[WHEEL_SPEED_RIGHT] =
        H45_WHEEL_VELOCITY_FF_PWM_PER_MM_S;
}

static void HMissionControlProfile_BuildHeading(HeadingConfig_t *config)
{
    config->straight_kp = H45_HEADING_STRAIGHT_KP;
    config->straight_kd = H45_HEADING_STRAIGHT_KD;
    config->turn_kp = H45_HEADING_TURN_KP;
    config->turn_kd = H45_HEADING_TURN_KD;
    config->rate_alpha = H45_HEADING_RATE_ALPHA;
    config->correction_limit_mm_s =
        H45_HEADING_CORRECTION_LIMIT_MM_S;
    config->turn_speed_mm_s = H45_HEADING_TURN_SPEED_MM_S;
    config->smooth_zone_deg = H45_HEADING_SMOOTH_ZONE_DEG;
}

static void HMissionControlProfile_BuildLine(LineFollowConfig_t *config)
{
    config->speed_min_mm_s = H45_LINE_SPEED_MIN_MM_S;
    config->speed_max_mm_s = H45_LINE_SPEED_MAX_MM_S;
    config->position_alpha = H45_LINE_POSITION_ALPHA;
    config->derivative_alpha = H45_LINE_DERIVATIVE_ALPHA;
    config->kp = H45_LINE_KP;
    config->kd = H45_LINE_KD;
    config->correction_limit_mm_s = H45_LINE_CORRECTION_LIMIT_MM_S;
    config->correction_slew_mm_s2 = H45_LINE_CORRECTION_SLEW_MM_S2;
    config->curve_gain = H45_LINE_CURVE_GAIN;
    config->lost_search_speed_mm_s =
        H45_LINE_LOST_SEARCH_SPEED_MM_S;
}

void HMissionControlProfile_Init(void)
{
    memset(&profile, 0, sizeof(profile));
}

bool HMissionControlProfile_Activate(HMissionMode_t mode)
{
    WheelSpeedConfig_t wheel;
    HeadingConfig_t heading;
    LineFollowConfig_t line;

    if ((mode != H_MISSION_MODE_H4) && (mode != H_MISSION_MODE_H5))
        return false;
    if (profile.snapshot.active)
        return profile.snapshot.active_mode == mode;
    WheelSpeed_GetConfig(&profile.saved_wheel);
    Heading_GetConfig(&profile.saved_heading);
    LineFollow_GetConfig(&profile.saved_line);
    wheel = profile.saved_wheel;
    heading = profile.saved_heading;
    line = profile.saved_line;
    HMissionControlProfile_BuildWheel(&wheel, mode);
    HMissionControlProfile_BuildHeading(&heading);
    HMissionControlProfile_BuildLine(&line);

    if (!WheelSpeed_ApplyConfig(&wheel, true)) return false;
    if (!Heading_ApplyConfig(&heading)) {
        (void)WheelSpeed_ApplyConfig(&profile.saved_wheel, true);
        return false;
    }
    if (!LineFollow_ApplyConfig(&line)) {
        (void)Heading_ApplyConfig(&profile.saved_heading);
        (void)WheelSpeed_ApplyConfig(&profile.saved_wheel, true);
        return false;
    }
    profile.snapshot.active = true;
    profile.snapshot.active_mode = mode;
    profile.snapshot.activation_count++;
    return true;
}

bool HMissionControlProfile_Restore(void)
{
    bool ok;
    if (!profile.snapshot.active) return true;
    ok = LineFollow_ApplyConfig(&profile.saved_line);
    ok = Heading_ApplyConfig(&profile.saved_heading) && ok;
    ok = WheelSpeed_ApplyConfig(&profile.saved_wheel, true) && ok;
    if (ok) {
        profile.snapshot.active = false;
        profile.snapshot.active_mode = (HMissionMode_t)0;
        profile.snapshot.restore_count++;
    }
    return ok;
}

void HMissionControlProfile_GetSnapshot(
    HMissionControlProfileSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = profile.snapshot;
}
