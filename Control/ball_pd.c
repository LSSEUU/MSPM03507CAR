#include "ball_pd.h"

#include "h3_control_config.h"

#include <stddef.h>

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

void BallPd_Reset(BallPdState_t *state, uint32_t now_ms)
{
    if (state == NULL) return;
    state->tilt_mdeg = 0;
    state->raw_requested_mdeg = 0;
    state->integral_mdeg = 0;
    state->previous_error_x10 = 0;
    state->last_update_ms = now_ms;
    state->initialized = 1U;
    state->breakaway_active = 0U;
}

int32_t BallPd_Update(BallPdState_t *state, int32_t target_mm_x10,
                      int32_t x_mm_x10, int32_t vx_mm_s,
                      int32_t minimum_tilt_mdeg, uint32_t now_ms,
                      const BallPdConfig_t *config)
{
    int32_t error_x10;
    int32_t requested;
    int32_t proportional_derivative;
    int32_t previous_integral;
    int32_t slew;
    int32_t slew_per_20ms;
    uint32_t elapsed_ms;
    uint8_t was_breakaway;

    if (state == NULL || config == NULL) return 0;
    if (minimum_tilt_mdeg < 0) minimum_tilt_mdeg = -minimum_tilt_mdeg;
    if (minimum_tilt_mdeg > config->positive_limit_mdeg &&
        minimum_tilt_mdeg > config->negative_limit_mdeg)
        minimum_tilt_mdeg = config->positive_limit_mdeg >
            config->negative_limit_mdeg ? config->positive_limit_mdeg :
                                          config->negative_limit_mdeg;
    if (state->initialized == 0U) BallPd_Reset(state, now_ms);
    was_breakaway = state->breakaway_active;
    elapsed_ms = now_ms - state->last_update_ms;
    if (elapsed_ms == 0U) elapsed_ms = 20U;
    if (elapsed_ms > 100U) elapsed_ms = 100U;

    error_x10 = target_mm_x10 - x_mm_x10;
    proportional_derivative = H3_DIRECTION_SIGN *
        (((int32_t)config->kp_mdeg_per_mm * error_x10) / 10L -
         (int32_t)config->kd_mdeg_per_mm_s * vx_mm_s);
    previous_integral = state->integral_mdeg;
    if ((error_x10 == 0) ||
        ((state->previous_error_x10 > 0) && (error_x10 < 0)) ||
        ((state->previous_error_x10 < 0) && (error_x10 > 0)) ||
        ((error_x10 >= -H3_INTEGRAL_SETTLE_ERROR_X10) &&
         (error_x10 <= H3_INTEGRAL_SETTLE_ERROR_X10))) {
        state->integral_mdeg = 0;
    } else if ((config->ki_mdeg_per_mm_s > 0U) &&
               (minimum_tilt_mdeg == 0) &&
               (vx_mm_s >= -H3_INTEGRAL_ACTIVE_SPEED_MM_S) &&
               (vx_mm_s <= H3_INTEGRAL_ACTIVE_SPEED_MM_S)) {
        int64_t delta = (int64_t)H3_DIRECTION_SIGN *
            config->ki_mdeg_per_mm_s * error_x10 * (int32_t)elapsed_ms;
        state->integral_mdeg += (int32_t)(delta / 10000L);
        state->integral_mdeg = clamp_i32(
            state->integral_mdeg, -(int32_t)config->integral_limit_mdeg,
            (int32_t)config->integral_limit_mdeg);
    } else if (state->integral_mdeg > 0) {
        state->integral_mdeg -= state->integral_mdeg >
            H3_INTEGRAL_DECAY_MDEG_PER_20MS ?
            H3_INTEGRAL_DECAY_MDEG_PER_20MS : state->integral_mdeg;
    } else if (state->integral_mdeg < 0) {
        int32_t magnitude = -state->integral_mdeg;
        state->integral_mdeg += magnitude >
            H3_INTEGRAL_DECAY_MDEG_PER_20MS ?
            H3_INTEGRAL_DECAY_MDEG_PER_20MS : magnitude;
    }
    requested = proportional_derivative + state->integral_mdeg;
    if (((requested > (int32_t)config->positive_limit_mdeg) &&
         (state->integral_mdeg > previous_integral)) ||
        ((requested < -(int32_t)config->negative_limit_mdeg) &&
         (state->integral_mdeg < previous_integral))) {
        state->integral_mdeg = previous_integral;
        requested = proportional_derivative + state->integral_mdeg;
    }
    requested = clamp_i32(requested, -(int32_t)config->negative_limit_mdeg,
                          (int32_t)config->positive_limit_mdeg);
    state->raw_requested_mdeg = requested;
    state->previous_error_x10 = error_x10;
    if ((minimum_tilt_mdeg > 0) && (error_x10 != 0)) {
        int32_t expected_sign = error_x10 > 0 ? H3_DIRECTION_SIGN :
                                               -H3_DIRECTION_SIGN;
        if ((expected_sign > 0) && (requested < minimum_tilt_mdeg))
            requested = minimum_tilt_mdeg;
        else if ((expected_sign < 0) && (requested > -minimum_tilt_mdeg))
            requested = -minimum_tilt_mdeg;
    }
    requested = clamp_i32(requested, -(int32_t)config->negative_limit_mdeg,
                          (int32_t)config->positive_limit_mdeg);

    /* A command opposing measured ball velocity is active braking.  Let that
     * counter-tilt build faster than ordinary tracking so static-friction
     * ramp-up does not leave the controller coasting toward the target. */
    slew_per_20ms = minimum_tilt_mdeg > 0 ?
        config->breakaway_slew_mdeg_per_20ms :
        config->normal_slew_mdeg_per_20ms;
    if (((int64_t)requested * H3_DIRECTION_SIGN * vx_mm_s) < 0L)
        slew_per_20ms = config->brake_slew_mdeg_per_20ms;
    else if ((minimum_tilt_mdeg == 0) &&
             (((state->tilt_mdeg > 0) &&
               (requested < state->tilt_mdeg)) ||
              ((state->tilt_mdeg < 0) &&
               (requested > state->tilt_mdeg))) &&
             ((was_breakaway != 0U) ||
              (((int64_t)state->tilt_mdeg * H3_DIRECTION_SIGN *
                vx_mm_s) > 0L)))
        /* Keep the brake slew active after the one-shot breakaway flag has
         * cleared while the remaining tilt is still accelerating the ball
         * outward.  Falling back to the normal slew here left a +16 degree
         * launch request applied for roughly 600 ms after breakaway. */
        slew_per_20ms = config->brake_slew_mdeg_per_20ms;

    slew = (int32_t)(((int64_t)slew_per_20ms *
                      elapsed_ms + 19L) / 20L);
    requested = clamp_i32(requested, state->tilt_mdeg - slew,
                          state->tilt_mdeg + slew);
    state->tilt_mdeg = requested;
    state->last_update_ms = now_ms;
    state->breakaway_active = minimum_tilt_mdeg > 0 ? 1U : 0U;
    return requested;
}
