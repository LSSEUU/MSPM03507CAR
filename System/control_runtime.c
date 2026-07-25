#include "control_runtime.h"

#include "app_remote.h"
#include "diagnostics.h"
#include "heading.h"
#include "line_follow.h"
#include "wheel_speed.h"

bool ControlRuntime_ApplyActive(const ControlParamsCommit_t *commit)
{
    const ControlParams_t *params = ControlParams_Active();
    WheelSpeedConfig_t wheel;
    HeadingConfig_t heading;
    LineFollowConfig_t line;
    AppRemoteConfig_t remote;
    bool ok = true;

    if ((commit != NULL) && commit->diagnostic_boundary) {
        if ((commit->change_mask &
             (CONTROL_PARAMS_CHANGE_WHEEL_LEFT_GAINS |
              CONTROL_PARAMS_CHANGE_WHEEL_DIAG_TARGET)) != 0U)
            WheelSpeed_SetGains(WHEEL_SPEED_LEFT, params->wheel.left_kp,
                                params->wheel.left_ki);
        if ((commit->change_mask &
             (CONTROL_PARAMS_CHANGE_WHEEL_RIGHT_GAINS |
              CONTROL_PARAMS_CHANGE_WHEEL_DIAG_TARGET)) != 0U)
            WheelSpeed_SetGains(WHEEL_SPEED_RIGHT, params->wheel.right_kp,
                                params->wheel.right_ki);
        Diagnostics_OnControlParamsCommitted(commit);
        return true;
    }

    wheel.kp[WHEEL_SPEED_LEFT] = params->wheel.left_kp;
    wheel.ki[WHEEL_SPEED_LEFT] = params->wheel.left_ki;
    wheel.kp[WHEEL_SPEED_RIGHT] = params->wheel.right_kp;
    wheel.ki[WHEEL_SPEED_RIGHT] = params->wheel.right_ki;
    wheel.filter_alpha = params->wheel.filter_alpha;
    wheel.accel_mm_s2 = params->wheel.accel_mm_s2;
    wheel.decel_mm_s2 = params->wheel.decel_mm_s2;
    wheel.jerk_mm_s3 = params->wheel.jerk_mm_s3;
    wheel.static_feedforward_pwm[WHEEL_SPEED_LEFT] = params->wheel.left_ks;
    wheel.velocity_feedforward_pwm_per_mm_s[WHEEL_SPEED_LEFT] =
        params->wheel.left_kv;
    wheel.static_feedforward_pwm[WHEEL_SPEED_RIGHT] = params->wheel.right_ks;
    wheel.velocity_feedforward_pwm_per_mm_s[WHEEL_SPEED_RIGHT] =
        params->wheel.right_kv;

    heading.straight_kp = params->heading.straight_kp;
    heading.straight_kd = params->heading.straight_kd;
    heading.turn_kp = params->heading.turn_kp;
    heading.turn_kd = params->heading.turn_kd;
    heading.rate_alpha = params->heading.rate_alpha;
    heading.correction_limit_mm_s = params->heading.correction_limit_mm_s;
    heading.turn_speed_mm_s = params->heading.turn_speed_mm_s;
    heading.smooth_zone_deg = params->heading.smooth_zone_deg;

    line.speed_min_mm_s = params->line.speed_min_mm_s;
    line.speed_max_mm_s = params->line.speed_max_mm_s;
    line.position_alpha = params->line.position_alpha;
    line.derivative_alpha = params->line.derivative_alpha;
    line.kp = params->line.kp;
    line.kd = params->line.kd;
    line.correction_limit_mm_s = params->line.correction_limit_mm_s;
    line.correction_slew_mm_s2 = params->line.correction_slew_mm_s2;
    line.curve_gain = params->line.curve_gain;
    line.lost_search_speed_mm_s = params->line.lost_search_speed_mm_s;

    remote.speed_low_mm_s = params->remote.speed_low_mm_s;
    remote.speed_medium_mm_s = params->remote.speed_medium_mm_s;
    remote.speed_high_mm_s = params->remote.speed_high_mm_s;
    remote.heading_hold = params->remote.heading_hold != 0U;

    ok = WheelSpeed_ApplyConfig(&wheel, true) && ok;
    ok = Heading_ApplyConfig(&heading) && ok;
    ok = LineFollow_ApplyConfig(&line) && ok;
    ok = AppRemote_ApplyConfig(&remote) && ok;
    Diagnostics_OnControlParamsCommitted(commit);
    return ok;
}
