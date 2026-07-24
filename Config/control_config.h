#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* M3 chassis calibration measured on-car over 10 wheel revolutions. */
#define WHEEL_DIAMETER_MM                    46.0f
#define ENCODER_A_COUNTS_PER_WHEEL_REV      530.0f
#define ENCODER_B_COUNTS_PER_WHEEL_REV      531.4f
#define ENCODER_A_FORWARD_SIGN                  1
#define ENCODER_B_FORWARD_SIGN                 -1

#define WHEEL_SPEED_FILTER_ALPHA             0.25f
#define WHEEL_SPEED_MAX_TARGET_MM_S          700.0f
#define WHEEL_SPEED_MAX_PHYSICAL_MM_S       1500.0f
#define WHEEL_SPEED_MAX_JUMP_MM_S           1000.0f
#define WHEEL_SPEED_ACCEL_MM_S2              600.0f
#define WHEEL_SPEED_DECEL_MM_S2              900.0f

#define WHEEL_SPEED_PWM_LIMIT                850.0f
#define WHEEL_SPEED_KP_A                       0.70f
#define WHEEL_SPEED_KI_A                       2.50f
#define WHEEL_SPEED_KP_B                       0.70f
#define WHEEL_SPEED_KI_B                       2.50f

#define WHEEL_STALL_TARGET_MIN_MM_S          150.0f
#define WHEEL_STALL_PWM_MIN                  250.0f
#define WHEEL_STALL_TIMEOUT_MS                 500U

#define SPEED_DIAGNOSTIC_DEFAULT_MM_S        300.0f
#define SPEED_DIAGNOSTIC_STEP_MM_S            25.0f
#define SPEED_DIAGNOSTIC_HOLD_MS             5000U
#define SPEED_DIAGNOSTIC_SETTLE_MS           1500U
#define SPEED_DIAGNOSTIC_STOP_TIMEOUT_MS     1500U
#define SPEED_DIAGNOSTIC_ACTION_TIMEOUT_MS   7000U

#define HEADING_CALIBRATION_SAMPLES            20U
#define HEADING_HOLD_KP_MM_S_PER_DEG          3.0f
#define HEADING_HOLD_KD_MM_S_PER_DEG_S        0.80f
#define HEADING_HOLD_MAX_CORRECTION_MM_S     120.0f
#define HEADING_TURN_KP_MM_S_PER_DEG          2.0f
#define HEADING_TURN_KD_MM_S_PER_DEG_S        0.55f
#define HEADING_YAW_RATE_FILTER_ALPHA          0.35f
#define HEADING_TURN_MAX_SPEED_MM_S          150.0f
#define HEADING_TURN_MIN_SPEED_MM_S           35.0f
#define HEADING_TURN_TOLERANCE_DEG             2.0f
#define HEADING_TURN_SETTLE_SAMPLES             3U
#define HEADING_TURN_TIMEOUT_MS               5000U
#define HEADING_ACTION_TIMEOUT_MS             7000U
#define HEADING_STRAIGHT_TEST_SPEED_MM_S      250.0f
#define HEADING_STRAIGHT_TEST_DURATION_MS     5000U

/* M5 fixed digital line-sensor and line-following defaults. Position is
 * normalized to [-1, 1], negative on the left and positive on the right. */
#define LINE_VALID_CONFIDENCE                   0.20f
#define LINE_VALID_ACQUIRE_SAMPLES                 2U
#define LINE_VALID_RELEASE_SAMPLES                 2U
#define LINE_EXPECTED_ACTIVE_SUM                 1.50f

#define LINE_FOLLOW_BASE_SPEED_MM_S             160.0f
#define LINE_FOLLOW_KP_MM_S                     150.0f
#define LINE_FOLLOW_KD_MM_S_PER_UNIT_S            1.0f
#define LINE_FOLLOW_MAX_CORRECTION_MM_S         150.0f
#define LINE_SEARCH_FORWARD_MM_S                 45.0f
#define LINE_SEARCH_TURN_MM_S                   105.0f
#define LINE_LOST_TIMEOUT_MS                    1500U
#define LINE_FOLLOW_ACTION_TIMEOUT_MS         300000U

#endif
