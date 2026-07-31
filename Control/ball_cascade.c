#include "ball_cascade.h"

#include <limits.h>
#include <stddef.h>

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int32_t add_saturating_i32(int32_t value, int32_t delta)
{
    int64_t sum = (int64_t)value + delta;
    if (sum > INT32_MAX) return INT32_MAX;
    if (sum < INT32_MIN) return INT32_MIN;
    return (int32_t)sum;
}

static int32_t divide_round_i64(int64_t numerator, int32_t denominator)
{
    int64_t value;
    if (numerator >= 0) {
        value = (numerator + denominator / 2) / denominator;
    } else {
        value = (numerator - denominator / 2) / denominator;
    }
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

static uint32_t bounded_elapsed(uint32_t elapsed_ms)
{
    if (elapsed_ms == 0U) return 20U;
    if (elapsed_ms > 100U) return 100U;
    return elapsed_ms;
}

static bool crossed_zero(int32_t previous, int32_t current)
{
    return ((previous > 0) && (current < 0)) ||
           ((previous < 0) && (current > 0));
}

bool BallCascade_ConfigValid(const BallCascadeConfig_t *config)
{
    if (config == NULL) return false;
    if ((config->direction_sign != 1) &&
        (config->direction_sign != -1)) return false;
    if ((config->velocity_limit_mm_s == 0U) ||
        (config->positive_tilt_limit_mdeg == 0U) ||
        (config->negative_tilt_limit_mdeg == 0U) ||
        (config->tilt_slew_mdeg_per_20ms == 0U)) return false;
    if ((config->position_kp_x1000_per_s == 0U) &&
        (config->position_ki_x1000_per_s2 == 0U)) return false;
    if ((config->speed_kp_x100_mdeg_per_mm_s == 0U) &&
        (config->speed_ki_x100_mdeg_per_mm == 0U)) return false;
    if ((config->position_ki_x1000_per_s2 != 0U) &&
        (config->position_integral_limit_mm_s == 0U)) return false;
    if ((config->speed_ki_x100_mdeg_per_mm != 0U) &&
        (config->speed_integral_limit_mdeg == 0U)) return false;
    return true;
}

void BallCascade_Reset(BallCascadeState_t *state, uint32_t now_ms)
{
    if (state == NULL) return;
    state->position_integral_mm_s_x1000 = 0;
    state->speed_integral_mdeg_x1000 = 0;
    state->previous_position_error_x10 = 0;
    state->previous_velocity_error_mm_s = 0;
    state->target_velocity_mm_s = 0;
    state->velocity_error_mm_s = 0;
    state->raw_tilt_mdeg = 0;
    state->tilt_mdeg = 0;
    state->last_update_ms = now_ms;
    state->initialized = 1U;
    state->position_saturated = 0U;
    state->tilt_saturated = 0U;
}

int32_t BallCascade_PositionLoop(BallCascadeState_t *state,
    int32_t target_mm_x10, int32_t position_mm_x10,
    uint32_t elapsed_ms, const BallCascadeConfig_t *config)
{
    int32_t error_x10;
    int32_t previous_integral;
    int32_t integral_limit_x1000;
    int32_t velocity_limit_x1000;
    int32_t requested_x1000;
    int32_t integral_delta;
    int64_t proportional_x1000;
    int64_t unclamped_x1000;

    if ((state == NULL) || !BallCascade_ConfigValid(config)) return 0;
    elapsed_ms = bounded_elapsed(elapsed_ms);
    error_x10 = target_mm_x10 - position_mm_x10;
    if ((error_x10 >= -(int32_t)config->position_deadband_x10) &&
        (error_x10 <= (int32_t)config->position_deadband_x10))
        error_x10 = 0;

    previous_integral = state->position_integral_mm_s_x1000;
    if ((error_x10 == 0) ||
        crossed_zero(state->previous_position_error_x10, error_x10)) {
        state->position_integral_mm_s_x1000 = 0;
    } else if (config->position_ki_x1000_per_s2 != 0U) {
        int64_t delta = (int64_t)config->position_ki_x1000_per_s2 *
            error_x10 * elapsed_ms;
        state->position_integral_mm_s_x1000 = add_saturating_i32(
            state->position_integral_mm_s_x1000,
            divide_round_i64(delta, 10000));
    }
    integral_limit_x1000 =
        (int32_t)config->position_integral_limit_mm_s * 1000;
    state->position_integral_mm_s_x1000 = clamp_i32(
        state->position_integral_mm_s_x1000, -integral_limit_x1000,
        integral_limit_x1000);

    proportional_x1000 =
        (int64_t)config->position_kp_x1000_per_s * error_x10 / 10;
    unclamped_x1000 = proportional_x1000 +
        state->position_integral_mm_s_x1000;
    velocity_limit_x1000 = (int32_t)config->velocity_limit_mm_s * 1000;
    integral_delta = state->position_integral_mm_s_x1000 -
        previous_integral;
    if (((unclamped_x1000 > velocity_limit_x1000) &&
         (integral_delta > 0)) ||
        ((unclamped_x1000 < -velocity_limit_x1000) &&
         (integral_delta < 0))) {
        state->position_integral_mm_s_x1000 = previous_integral;
        unclamped_x1000 = proportional_x1000 + previous_integral;
    }
    state->position_saturated =
        ((unclamped_x1000 > velocity_limit_x1000) ||
         (unclamped_x1000 < -velocity_limit_x1000)) ? 1U : 0U;
    if (unclamped_x1000 > velocity_limit_x1000)
        requested_x1000 = velocity_limit_x1000;
    else if (unclamped_x1000 < -velocity_limit_x1000)
        requested_x1000 = -velocity_limit_x1000;
    else
        requested_x1000 = (int32_t)unclamped_x1000;

    state->previous_position_error_x10 = error_x10;
    state->target_velocity_mm_s = divide_round_i64(requested_x1000, 1000);
    return state->target_velocity_mm_s;
}

int32_t BallCascade_SpeedLoop(BallCascadeState_t *state,
    int32_t target_velocity_mm_s, int32_t velocity_mm_s,
    uint32_t elapsed_ms, const BallCascadeConfig_t *config)
{
    int32_t error_mm_s;
    int32_t previous_integral;
    int32_t integral_limit_x1000;
    int32_t integral_delta_physical;
    int32_t positive_limit_x1000;
    int32_t negative_limit_x1000;
    int32_t requested_mdeg;
    int32_t slew_mdeg;
    int64_t proportional_mdeg_x1000;
    int64_t requested_mdeg_x1000;

    if ((state == NULL) || !BallCascade_ConfigValid(config)) return 0;
    elapsed_ms = bounded_elapsed(elapsed_ms);
    error_mm_s = target_velocity_mm_s - velocity_mm_s;
    previous_integral = state->speed_integral_mdeg_x1000;
    if ((error_mm_s == 0) ||
        crossed_zero(state->previous_velocity_error_mm_s, error_mm_s)) {
        state->speed_integral_mdeg_x1000 = 0;
    } else if (config->speed_ki_x100_mdeg_per_mm != 0U) {
        int64_t delta = (int64_t)config->speed_ki_x100_mdeg_per_mm *
            error_mm_s * elapsed_ms;
        state->speed_integral_mdeg_x1000 = add_saturating_i32(
            state->speed_integral_mdeg_x1000,
            divide_round_i64(delta, 100));
    }
    integral_limit_x1000 =
        (int32_t)config->speed_integral_limit_mdeg * 1000;
    state->speed_integral_mdeg_x1000 = clamp_i32(
        state->speed_integral_mdeg_x1000, -integral_limit_x1000,
        integral_limit_x1000);

    proportional_mdeg_x1000 =
        (int64_t)config->speed_kp_x100_mdeg_per_mm_s * error_mm_s * 10;
    requested_mdeg_x1000 = (proportional_mdeg_x1000 +
        state->speed_integral_mdeg_x1000) * config->direction_sign;
    positive_limit_x1000 =
        (int32_t)config->positive_tilt_limit_mdeg * 1000;
    negative_limit_x1000 =
        (int32_t)config->negative_tilt_limit_mdeg * 1000;
    integral_delta_physical =
        (state->speed_integral_mdeg_x1000 - previous_integral) *
        config->direction_sign;
    if (((requested_mdeg_x1000 > positive_limit_x1000) &&
         (integral_delta_physical > 0)) ||
        ((requested_mdeg_x1000 < -negative_limit_x1000) &&
         (integral_delta_physical < 0))) {
        state->speed_integral_mdeg_x1000 = previous_integral;
        requested_mdeg_x1000 = (proportional_mdeg_x1000 +
            previous_integral) * config->direction_sign;
    }
    state->tilt_saturated =
        ((requested_mdeg_x1000 > positive_limit_x1000) ||
         (requested_mdeg_x1000 < -negative_limit_x1000)) ? 1U : 0U;
    if (requested_mdeg_x1000 > positive_limit_x1000)
        requested_mdeg_x1000 = positive_limit_x1000;
    else if (requested_mdeg_x1000 < -negative_limit_x1000)
        requested_mdeg_x1000 = -negative_limit_x1000;
    requested_mdeg = divide_round_i64(requested_mdeg_x1000, 1000);
    state->raw_tilt_mdeg = requested_mdeg;

    slew_mdeg = (int32_t)(((int64_t)config->tilt_slew_mdeg_per_20ms *
                            elapsed_ms + 19) / 20);
    requested_mdeg = clamp_i32(requested_mdeg,
        state->tilt_mdeg - slew_mdeg, state->tilt_mdeg + slew_mdeg);
    state->tilt_mdeg = requested_mdeg;
    state->velocity_error_mm_s = error_mm_s;
    state->previous_velocity_error_mm_s = error_mm_s;
    return requested_mdeg;
}

int32_t BallCascade_Update(BallCascadeState_t *state,
    int32_t target_mm_x10, int32_t position_mm_x10,
    int32_t velocity_mm_s, uint32_t now_ms,
    const BallCascadeConfig_t *config)
{
    uint32_t elapsed_ms;
    int32_t target_velocity_mm_s;

    if ((state == NULL) || !BallCascade_ConfigValid(config)) return 0;
    if (state->initialized == 0U) BallCascade_Reset(state, now_ms);
    elapsed_ms = bounded_elapsed(now_ms - state->last_update_ms);
    target_velocity_mm_s = BallCascade_PositionLoop(state,
        target_mm_x10, position_mm_x10, elapsed_ms, config);
    (void)BallCascade_SpeedLoop(state, target_velocity_mm_s,
        velocity_mm_s, elapsed_ms, config);
    state->last_update_ms = now_ms;
    return state->tilt_mdeg;
}
