#ifndef BALL_PD_H
#define BALL_PD_H

#include <stdint.h>

typedef struct {
    int32_t tilt_mdeg;
    int32_t raw_requested_mdeg;
    int32_t integral_mdeg;
    int32_t previous_error_x10;
    uint32_t last_update_ms;
    uint8_t initialized;
    uint8_t breakaway_active;
} BallPdState_t;

typedef struct {
    uint16_t kp_mdeg_per_mm;
    uint16_t kd_mdeg_per_mm_s;
    uint16_t ki_mdeg_per_mm_s;
    uint16_t integral_limit_mdeg;
    uint16_t normal_slew_mdeg_per_20ms;
    uint16_t breakaway_slew_mdeg_per_20ms;
    uint16_t brake_slew_mdeg_per_20ms;
    uint16_t positive_limit_mdeg;
    uint16_t negative_limit_mdeg;
} BallPdConfig_t;

void BallPd_Reset(BallPdState_t *state, uint32_t now_ms);
int32_t BallPd_Update(BallPdState_t *state, int32_t target_mm_x10,
                      int32_t x_mm_x10, int32_t vx_mm_s,
                      int32_t minimum_tilt_mdeg, uint32_t now_ms,
                      const BallPdConfig_t *config);

#endif
