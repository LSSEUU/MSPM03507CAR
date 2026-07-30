#include "h_mission_control_profile.h"

#include "heading.h"
#include "line_follow.h"
#include "wheel_speed.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

static WheelSpeedConfig_t wheel_config;
static HeadingConfig_t heading_config;
static LineFollowConfig_t line_config;
static bool fail_profile_line_once;
static uint32_t apply_count;

void WheelSpeed_GetConfig(WheelSpeedConfig_t *config)
{
    *config = wheel_config;
}

bool WheelSpeed_ApplyConfig(const WheelSpeedConfig_t *config,
                            bool reset_controllers)
{
    assert(reset_controllers);
    wheel_config = *config;
    apply_count++;
    return true;
}

void Heading_GetConfig(HeadingConfig_t *config)
{
    *config = heading_config;
}

bool Heading_ApplyConfig(const HeadingConfig_t *config)
{
    heading_config = *config;
    apply_count++;
    return true;
}

void LineFollow_GetConfig(LineFollowConfig_t *config)
{
    *config = line_config;
}

bool LineFollow_ApplyConfig(const LineFollowConfig_t *config)
{
    if (fail_profile_line_once &&
        fabsf(config->speed_max_mm_s - 400.0f) < 0.01f) {
        fail_profile_line_once = false;
        return false;
    }
    line_config = *config;
    apply_count++;
    return true;
}

static void set_external_configs(void)
{
    memset(&wheel_config, 0, sizeof(wheel_config));
    memset(&heading_config, 0, sizeof(heading_config));
    memset(&line_config, 0, sizeof(line_config));
    wheel_config.kp[WHEEL_SPEED_LEFT] = 1.1f;
    wheel_config.ki[WHEEL_SPEED_LEFT] = 3.1f;
    wheel_config.kp[WHEEL_SPEED_RIGHT] = 1.2f;
    wheel_config.ki[WHEEL_SPEED_RIGHT] = 3.2f;
    wheel_config.filter_alpha = 0.6f;
    wheel_config.accel_mm_s2 = 600.0f;
    wheel_config.decel_mm_s2 = 900.0f;
    wheel_config.jerk_mm_s3 = 20000.0f;
    wheel_config.static_feedforward_pwm[WHEEL_SPEED_LEFT] = 22.0f;
    wheel_config.velocity_feedforward_pwm_per_mm_s[WHEEL_SPEED_LEFT] = 0.5f;
    wheel_config.static_feedforward_pwm[WHEEL_SPEED_RIGHT] = 23.0f;
    wheel_config.velocity_feedforward_pwm_per_mm_s[WHEEL_SPEED_RIGHT] = 0.6f;
    heading_config.straight_kp = 4.0f;
    heading_config.straight_kd = 1.1f;
    heading_config.turn_kp = 3.0f;
    heading_config.turn_kd = 0.9f;
    heading_config.rate_alpha = 0.5f;
    heading_config.correction_limit_mm_s = 100.0f;
    heading_config.turn_speed_mm_s = 140.0f;
    heading_config.smooth_zone_deg = 4.0f;
    line_config.speed_min_mm_s = 180.0f;
    line_config.speed_max_mm_s = 260.0f;
    line_config.position_alpha = 0.5f;
    line_config.derivative_alpha = 0.4f;
    line_config.kp = 70.0f;
    line_config.kd = 8.0f;
    line_config.correction_limit_mm_s = 80.0f;
    line_config.correction_slew_mm_s2 = 500.0f;
    line_config.curve_gain = 1.0f;
    line_config.lost_search_speed_mm_s = 90.0f;
}

static void assert_h45_profile(HMissionMode_t mode)
{
    assert(fabsf(wheel_config.kp[WHEEL_SPEED_LEFT] - 0.7f) < 0.001f);
    assert(fabsf(wheel_config.ki[WHEEL_SPEED_RIGHT] - 2.5f) < 0.001f);
    assert(fabsf(wheel_config.filter_alpha - 0.1f) < 0.001f);
    assert(fabsf(wheel_config.accel_mm_s2 - 250.0f) < 0.001f);
    assert(fabsf(wheel_config.decel_mm_s2 -
                 (mode == H_MISSION_MODE_H4 ? 400.0f : 300.0f)) < 0.001f);
    assert(fabsf(wheel_config.jerk_mm_s3 - 1500.0f) < 0.001f);
    assert(fabsf(wheel_config.static_feedforward_pwm[WHEEL_SPEED_LEFT] -
                 10.0f) < 0.001f);
    assert(fabsf(wheel_config.velocity_feedforward_pwm_per_mm_s[
                 WHEEL_SPEED_RIGHT] - 0.75f) < 0.001f);
    assert(fabsf(heading_config.straight_kp - 3.0f) < 0.001f);
    assert(fabsf(heading_config.straight_kd - 0.88f) < 0.001f);
    assert(fabsf(heading_config.turn_kp - 2.0f) < 0.001f);
    assert(fabsf(heading_config.turn_kd - 0.55f) < 0.001f);
    assert(fabsf(heading_config.rate_alpha - 0.35f) < 0.001f);
    assert(fabsf(heading_config.correction_limit_mm_s - 120.0f) < 0.001f);
    assert(fabsf(line_config.speed_min_mm_s - 300.0f) < 0.001f);
    assert(fabsf(line_config.speed_max_mm_s - 400.0f) < 0.001f);
    assert(fabsf(line_config.position_alpha - 0.35f) < 0.001f);
    assert(fabsf(line_config.derivative_alpha - 0.25f) < 0.001f);
    assert(fabsf(line_config.kp - 55.0f) < 0.001f);
    assert(fabsf(line_config.kd - 10.0f) < 0.001f);
    assert(fabsf(line_config.correction_limit_mm_s - 75.0f) < 0.001f);
    assert(fabsf(line_config.correction_slew_mm_s2 - 300.0f) < 0.001f);
    assert(fabsf(line_config.curve_gain - 2.0f) < 0.001f);
    assert(fabsf(line_config.lost_search_speed_mm_s - 105.0f) < 0.001f);
}

static void verify_mode(HMissionMode_t mode)
{
    WheelSpeedConfig_t saved_wheel = wheel_config;
    HeadingConfig_t saved_heading = heading_config;
    LineFollowConfig_t saved_line = line_config;
    HMissionControlProfileSnapshot_t snapshot;
    uint32_t count_after_activation;

    assert(HMissionControlProfile_Activate(mode));
    HMissionControlProfile_GetSnapshot(&snapshot);
    assert(snapshot.active);
    assert(snapshot.active_mode == mode);
    assert_h45_profile(mode);
    count_after_activation = apply_count;
    assert(HMissionControlProfile_Activate(mode));
    assert(apply_count == count_after_activation);
    assert(!HMissionControlProfile_Activate(
        mode == H_MISSION_MODE_H4 ? H_MISSION_MODE_H5 : H_MISSION_MODE_H4));
    assert(HMissionControlProfile_Restore());
    HMissionControlProfile_GetSnapshot(&snapshot);
    assert(!snapshot.active);
    assert(memcmp(&wheel_config, &saved_wheel, sizeof(saved_wheel)) == 0);
    assert(memcmp(&heading_config, &saved_heading, sizeof(saved_heading)) == 0);
    assert(memcmp(&line_config, &saved_line, sizeof(saved_line)) == 0);
}

int main(void)
{
    WheelSpeedConfig_t saved_wheel;
    HeadingConfig_t saved_heading;
    LineFollowConfig_t saved_line;
    HMissionControlProfileSnapshot_t snapshot;

    set_external_configs();
    HMissionControlProfile_Init();
    assert(!HMissionControlProfile_Activate(H_MISSION_MODE_H2));
    verify_mode(H_MISSION_MODE_H4);
    verify_mode(H_MISSION_MODE_H5);

    saved_wheel = wheel_config;
    saved_heading = heading_config;
    saved_line = line_config;
    HMissionControlProfile_Init();
    fail_profile_line_once = true;
    assert(!HMissionControlProfile_Activate(H_MISSION_MODE_H4));
    HMissionControlProfile_GetSnapshot(&snapshot);
    assert(!snapshot.active);
    assert(memcmp(&wheel_config, &saved_wheel, sizeof(saved_wheel)) == 0);
    assert(memcmp(&heading_config, &saved_heading, sizeof(saved_heading)) == 0);
    assert(memcmp(&line_config, &saved_line, sizeof(saved_line)) == 0);
    return 0;
}
