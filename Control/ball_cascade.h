#ifndef BALL_CASCADE_H
#define BALL_CASCADE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Position loop:
 *   position error -> requested ball velocity
 * Speed loop:
 *   velocity error -> lever tilt
 *
 * Gains use explicit fixed-point scales so the controller is deterministic
 * on the Cortex-M0+ and does not pull floating-point code into the 20 ms
 * control path.
 */
typedef struct {
    uint32_t position_kp_x1000_per_s;
    uint32_t position_ki_x1000_per_s2;
    uint16_t position_integral_limit_mm_s;
    uint16_t position_deadband_x10;
    uint16_t velocity_limit_mm_s;
    uint32_t speed_kp_x100_mdeg_per_mm_s;
    uint32_t speed_ki_x100_mdeg_per_mm;
    uint16_t speed_integral_limit_mdeg;
    uint16_t positive_tilt_limit_mdeg;
    uint16_t negative_tilt_limit_mdeg;
    uint16_t tilt_slew_mdeg_per_20ms;
    int8_t direction_sign;
} BallCascadeConfig_t;

typedef struct {
    int32_t position_integral_mm_s_x1000;
    int32_t speed_integral_mdeg_x1000;
    int32_t previous_position_error_x10;
    int32_t previous_velocity_error_mm_s;
    int32_t target_velocity_mm_s;
    int32_t velocity_error_mm_s;
    int32_t raw_tilt_mdeg;
    int32_t tilt_mdeg;
    uint32_t last_update_ms;
    uint8_t initialized;
    uint8_t position_saturated;
    uint8_t tilt_saturated;
} BallCascadeState_t;

bool BallCascade_ConfigValid(const BallCascadeConfig_t *config);
void BallCascade_Reset(BallCascadeState_t *state, uint32_t now_ms);

int32_t BallCascade_PositionLoop(BallCascadeState_t *state,
    int32_t target_mm_x10, int32_t position_mm_x10,
    uint32_t elapsed_ms, const BallCascadeConfig_t *config);
int32_t BallCascade_SpeedLoop(BallCascadeState_t *state,
    int32_t target_velocity_mm_s, int32_t velocity_mm_s,
    uint32_t elapsed_ms, const BallCascadeConfig_t *config);

int32_t BallCascade_Update(BallCascadeState_t *state,
    int32_t target_mm_x10, int32_t position_mm_x10,
    int32_t velocity_mm_s, uint32_t now_ms,
    const BallCascadeConfig_t *config);

#endif
