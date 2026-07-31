#include "ball_pd.h"
#include "h3_control_config.h"

#include <assert.h>

int main(void)
{
    BallPdState_t state;
    BallPdConfig_t config = {
        H3_CONTROL_KP_MDEG_PER_MM,
        H3_CONTROL_KD_MDEG_PER_MM_S,
        H3_CONTROL_KI_MDEG_PER_MM_S,
        H3_INTEGRAL_LIMIT_MDEG,
        H3_TILT_SLEW_MDEG_PER_20MS,
        H3_BREAKAWAY_TILT_SLEW_MDEG_PER_20MS,
        H3_BRAKE_TILT_SLEW_MDEG_PER_20MS,
        H3_TILT_LIMIT_MDEG,
        H3_TILT_LIMIT_MDEG
    };
    int32_t output;
    uint32_t now_ms;
    BallPd_Reset(&state, 0U);
    config.positive_limit_mdeg = config.negative_limit_mdeg = 1500U;
    output = BallPd_Update(&state, 500, 0, 0, 0, 20U, &config);
    assert(output == 200 * H3_DIRECTION_SIGN);
    output = BallPd_Update(&state, 500, 0, 0, 0, 40U, &config);
    assert(output == 400 * H3_DIRECTION_SIGN);
    output = BallPd_Update(&state, 500, 0, 100, 0, 60U, &config);
    assert(output == 600 * H3_DIRECTION_SIGN);
    output = BallPd_Update(&state, -500, 0, 0, 0, 80U, &config);
    assert(output == 400 * H3_DIRECTION_SIGN);

    BallPd_Reset(&state, 100U);
    output = BallPd_Update(&state, 500, 0, 100, 0, 120U, &config);
    assert(output == 200 * H3_DIRECTION_SIGN);
    output = BallPd_Update(&state, 500, 0, 1000, 0, 140U, &config);
    assert(output == -1300 * H3_DIRECTION_SIGN);
    /* Derivative braking reverses demand with the faster bounded slew. */

    BallPd_Reset(&state, 200U);
    config.positive_limit_mdeg = config.negative_limit_mdeg = 600U;
    output = BallPd_Update(&state, 5000, 0, 0, 0, 220U, &config);
    assert(output == 200 * H3_DIRECTION_SIGN);
    output = BallPd_Update(&state, 5000, 0, 0, 0, 260U, &config);
    assert(output == 600 * H3_DIRECTION_SIGN);
    output = BallPd_Update(&state, 5000, 0, 0, 0, 280U, &config);
    assert(output == 600 * H3_DIRECTION_SIGN);

    /* The measured installation does not break static friction before
     * 5290 count, about +4.91 degrees from the 4396-count horizontal. */
    BallPd_Reset(&state, 1000U);
    config.positive_limit_mdeg = config.negative_limit_mdeg =
        H3_TILT_LIMIT_MDEG;
    config.ki_mdeg_per_mm_s = 0U;
    for (now_ms = 1020U; now_ms <= 1600U; now_ms += 20U) {
        output = BallPd_Update(&state, 500, 0, 0, 0, now_ms, &config);
    }
    assert(output ==
           H3_CONTROL_KP_MDEG_PER_MM * H3_TARGET_POSITION_MM *
           H3_DIRECTION_SIGN);
    assert(output * H3_DIRECTION_SIGN >
           H3_MEASURED_POSITIVE_BREAKAWAY_MDEG);

    BallPd_Reset(&state, 2000U);
    for (now_ms = 2020U; now_ms <= 3400U; now_ms += 20U) {
        output = BallPd_Update(&state, 500, 0, 0,
                               H3_BREAKAWAY_MAX_MDEG, now_ms, &config);
    }
    assert(output == H3_BREAKAWAY_MAX_MDEG * H3_DIRECTION_SIGN);
    output = BallPd_Update(&state, 500, 10, 0, 0, 3420U, &config);
    assert(output == (H3_BREAKAWAY_MAX_MDEG -
                      H3_BRAKE_TILT_SLEW_MDEG_PER_20MS) *
                     H3_DIRECTION_SIGN);
    output = BallPd_Update(&state, 500, 10, 10, 0, 3440U, &config);
    assert(output == (H3_BREAKAWAY_MAX_MDEG -
                      2 * H3_BRAKE_TILT_SLEW_MDEG_PER_20MS) *
                     H3_DIRECTION_SIGN);

    /* Once the ball is moving toward the target, velocity feedback must be
     * able to request a real counter-tilt and use the faster brake slew. */
    BallPd_Reset(&state, 3100U);
    output = BallPd_Update(&state, 500, 300, 100, 0, 3120U, &config);
    assert(output == (H3_CONTROL_KP_MDEG_PER_MM * 20 -
                      H3_CONTROL_KD_MDEG_PER_MM_S * 100) *
                     H3_DIRECTION_SIGN);

    /* Conditional Ki accumulates only while the ball is nearly stationary,
     * remains bounded, decays during motion, and clears across target. */
    BallPd_Reset(&state, 4000U);
    config.kp_mdeg_per_mm = 0U;
    config.kd_mdeg_per_mm_s = 0U;
    config.ki_mdeg_per_mm_s = 20U;
    config.integral_limit_mdeg = 3000U;
    for (now_ms = 4020U; now_ms <= 9000U; now_ms += 20U)
        output = BallPd_Update(&state, 500, 250, 0, 0, now_ms, &config);
    assert(state.integral_mdeg == 2500 * H3_DIRECTION_SIGN);
    output = BallPd_Update(&state, 500, 250, 10, 0, 9020U, &config);
    assert(state.integral_mdeg == 2400 * H3_DIRECTION_SIGN);
    output = BallPd_Update(&state, 500, 600, 0, 0, 9040U, &config);
    assert(state.integral_mdeg == 0);
    return 0;
}
