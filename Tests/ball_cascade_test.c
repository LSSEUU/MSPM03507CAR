#include "ball_cascade.h"

#include <assert.h>

static BallCascadeConfig_t default_config(void)
{
    BallCascadeConfig_t config = {
        2000U, 0U, 20U, 0U, 100U,
        10000U, 0U, 2000U,
        3000U, 3000U, 500U, 1
    };
    return config;
}

int main(void)
{
    BallCascadeState_t state;
    BallCascadeConfig_t config = default_config();
    int32_t output;
    int32_t velocity;
    uint32_t now_ms;

    assert(BallCascade_ConfigValid(&config));
    config.direction_sign = 0;
    assert(!BallCascade_ConfigValid(&config));
    config.direction_sign = 1;

    BallCascade_Reset(&state, 0U);
    velocity = BallCascade_PositionLoop(&state, 100, 0, 20U, &config);
    assert(velocity == 20);
    output = BallCascade_SpeedLoop(&state, velocity, 0, 20U, &config);
    assert(state.raw_tilt_mdeg == 2000);
    assert(output == 500);
    BallCascade_Reset(&state, 0U);
    output = BallCascade_Update(&state, 100, 0, 0, 20U, &config);
    assert(output == 500);
    output = BallCascade_Update(&state, 100, 0, 0, 40U, &config);
    assert(state.target_velocity_mm_s == 20);
    assert(output == 1000);

    /* The inner speed loop must brake when measured velocity exceeds the
     * outer-loop target, and the output must respect the same slew limit. */
    output = BallCascade_Update(&state, 100, 0, 50, 60U, &config);
    assert(state.velocity_error_mm_s == -30);
    assert(state.raw_tilt_mdeg == -3000);
    assert(output == 500);

    /* Position requests and tilt requests are independently saturated. */
    BallCascade_Reset(&state, 100U);
    output = BallCascade_Update(&state, 2000, 0, 0, 120U, &config);
    assert(state.position_saturated != 0U);
    assert(state.target_velocity_mm_s == 100);
    assert(state.tilt_saturated != 0U);
    assert(state.raw_tilt_mdeg == 3000);
    assert(output == 500);

    /* A one millimetre deadband suppresses position chatter while retaining
     * velocity damping through the inner loop. */
    config.position_deadband_x10 = 10U;
    BallCascade_Reset(&state, 200U);
    output = BallCascade_Update(&state, 0, 8, 12, 220U, &config);
    assert(state.target_velocity_mm_s == 0);
    assert(state.velocity_error_mm_s == -12);
    assert(output == -500);

    /* Integral terms remain bounded and do not wind farther into saturated
     * outputs.  Crossing the target clears both stored biases. */
    config = default_config();
    config.position_kp_x1000_per_s = 1U;
    config.position_ki_x1000_per_s2 = 50000U;
    config.position_integral_limit_mm_s = 20U;
    config.speed_kp_x100_mdeg_per_mm_s = 1U;
    config.speed_ki_x100_mdeg_per_mm = 20000U;
    config.speed_integral_limit_mdeg = 1000U;
    BallCascade_Reset(&state, 1000U);
    for (now_ms = 1020U; now_ms <= 4000U; now_ms += 20U)
        (void)BallCascade_Update(&state, 1000, 0, 0, now_ms, &config);
    assert(state.position_integral_mm_s_x1000 <= 20000);
    assert(state.speed_integral_mdeg_x1000 <= 1000000);
    (void)BallCascade_Update(&state, -1000, 0, 0, 4020U, &config);
    assert(state.position_integral_mm_s_x1000 <= 0);
    assert(state.speed_integral_mdeg_x1000 <= 0);

    /* Installation direction is applied only at the physical tilt output. */
    config = default_config();
    config.direction_sign = -1;
    BallCascade_Reset(&state, 5000U);
    output = BallCascade_Update(&state, 100, 0, 0, 5020U, &config);
    assert(state.target_velocity_mm_s == 20);
    assert(state.raw_tilt_mdeg == -2000);
    assert(output == -500);
    return 0;
}
