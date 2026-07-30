#include "h2_control_profile.h"

#include "heading.h"
#include "line_follow.h"
#include "wheel_speed.h"

#include <stddef.h>
#include <string.h>

/* Exact controller snapshot from the r55 lap whose student-selected vehicle
 * point stopped on the A line.  These values intentionally do not enter the
 * parameter store and therefore can never be saved to parameter Flash. */
#define H2_WHEEL_KP                         0.70f
#define H2_WHEEL_KI                         2.50f
#define H2_WHEEL_FILTER_ALPHA               0.10f
#define H2_WHEEL_ACCEL_MM_S2              600.0f
#define H2_WHEEL_DECEL_MM_S2              900.0f
#define H2_WHEEL_JERK_MM_S3             20000.0f
#define H2_WHEEL_STATIC_FF_PWM              10.0f
#define H2_WHEEL_VELOCITY_FF_PWM_PER_MM_S    0.75f

#define H2_HEADING_STRAIGHT_KP               3.0f
#define H2_HEADING_STRAIGHT_KD               0.88f
#define H2_HEADING_TURN_KP                   2.0f
#define H2_HEADING_TURN_KD                   0.55f
#define H2_HEADING_RATE_ALPHA                0.35f
#define H2_HEADING_CORRECTION_LIMIT_MM_S   120.0f
#define H2_HEADING_TURN_SPEED_MM_S          150.0f
#define H2_HEADING_SMOOTH_ZONE_DEG            2.0f

#define H2_LINE_SPEED_MIN_MM_S              300.0f
#define H2_LINE_SPEED_MAX_MM_S              400.0f
#define H2_LINE_POSITION_ALPHA                0.35f
#define H2_LINE_DERIVATIVE_ALPHA              0.25f
#define H2_LINE_KP                           55.0f
#define H2_LINE_KD                           10.0f
#define H2_LINE_CORRECTION_LIMIT_MM_S        75.0f
#define H2_LINE_CORRECTION_SLEW_MM_S2       300.0f
#define H2_LINE_CURVE_GAIN                    2.0f
#define H2_LINE_LOST_SEARCH_SPEED_MM_S      105.0f

typedef struct {
    H2ControlProfileSnapshot_t snapshot;
    WheelSpeedConfig_t saved_wheel;
    HeadingConfig_t saved_heading;
    LineFollowConfig_t saved_line;
} H2ControlProfileState_t;

static H2ControlProfileState_t profile;

static void H2ControlProfile_BuildWheel(WheelSpeedConfig_t *config)
{
    config->kp[WHEEL_SPEED_LEFT] = H2_WHEEL_KP;
    config->ki[WHEEL_SPEED_LEFT] = H2_WHEEL_KI;
    config->kp[WHEEL_SPEED_RIGHT] = H2_WHEEL_KP;
    config->ki[WHEEL_SPEED_RIGHT] = H2_WHEEL_KI;
    config->filter_alpha = H2_WHEEL_FILTER_ALPHA;
    config->accel_mm_s2 = H2_WHEEL_ACCEL_MM_S2;
    config->decel_mm_s2 = H2_WHEEL_DECEL_MM_S2;
    config->jerk_mm_s3 = H2_WHEEL_JERK_MM_S3;
    config->static_feedforward_pwm[WHEEL_SPEED_LEFT] =
        H2_WHEEL_STATIC_FF_PWM;
    config->velocity_feedforward_pwm_per_mm_s[WHEEL_SPEED_LEFT] =
        H2_WHEEL_VELOCITY_FF_PWM_PER_MM_S;
    config->static_feedforward_pwm[WHEEL_SPEED_RIGHT] =
        H2_WHEEL_STATIC_FF_PWM;
    config->velocity_feedforward_pwm_per_mm_s[WHEEL_SPEED_RIGHT] =
        H2_WHEEL_VELOCITY_FF_PWM_PER_MM_S;
}

static void H2ControlProfile_BuildHeading(HeadingConfig_t *config)
{
    config->straight_kp = H2_HEADING_STRAIGHT_KP;
    config->straight_kd = H2_HEADING_STRAIGHT_KD;
    config->turn_kp = H2_HEADING_TURN_KP;
    config->turn_kd = H2_HEADING_TURN_KD;
    config->rate_alpha = H2_HEADING_RATE_ALPHA;
    config->correction_limit_mm_s = H2_HEADING_CORRECTION_LIMIT_MM_S;
    config->turn_speed_mm_s = H2_HEADING_TURN_SPEED_MM_S;
    config->smooth_zone_deg = H2_HEADING_SMOOTH_ZONE_DEG;
}

static void H2ControlProfile_BuildLine(LineFollowConfig_t *config)
{
    config->speed_min_mm_s = H2_LINE_SPEED_MIN_MM_S;
    config->speed_max_mm_s = H2_LINE_SPEED_MAX_MM_S;
    config->position_alpha = H2_LINE_POSITION_ALPHA;
    config->derivative_alpha = H2_LINE_DERIVATIVE_ALPHA;
    config->kp = H2_LINE_KP;
    config->kd = H2_LINE_KD;
    config->correction_limit_mm_s = H2_LINE_CORRECTION_LIMIT_MM_S;
    config->correction_slew_mm_s2 = H2_LINE_CORRECTION_SLEW_MM_S2;
    config->curve_gain = H2_LINE_CURVE_GAIN;
    config->lost_search_speed_mm_s = H2_LINE_LOST_SEARCH_SPEED_MM_S;
}

void H2ControlProfile_Init(void)
{
    memset(&profile, 0, sizeof(profile));
}

bool H2ControlProfile_Activate(void)
{
    WheelSpeedConfig_t wheel;
    HeadingConfig_t heading;
    LineFollowConfig_t line;

    if (profile.snapshot.active) return true;
    WheelSpeed_GetConfig(&profile.saved_wheel);
    Heading_GetConfig(&profile.saved_heading);
    LineFollow_GetConfig(&profile.saved_line);
    wheel = profile.saved_wheel;
    heading = profile.saved_heading;
    line = profile.saved_line;
    H2ControlProfile_BuildWheel(&wheel);
    H2ControlProfile_BuildHeading(&heading);
    H2ControlProfile_BuildLine(&line);

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
    profile.snapshot.activation_count++;
    return true;
}

bool H2ControlProfile_Restore(void)
{
    bool ok;
    if (!profile.snapshot.active) return true;
    ok = LineFollow_ApplyConfig(&profile.saved_line);
    ok = Heading_ApplyConfig(&profile.saved_heading) && ok;
    ok = WheelSpeed_ApplyConfig(&profile.saved_wheel, true) && ok;
    if (ok) {
        profile.snapshot.active = false;
        profile.snapshot.restore_count++;
    }
    return ok;
}

void H2ControlProfile_GetSnapshot(H2ControlProfileSnapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = profile.snapshot;
}
