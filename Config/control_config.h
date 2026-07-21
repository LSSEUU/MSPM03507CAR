#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/*
 * M3 chassis calibration.  The encoder value is the decoded x4 count at the
 * wheel shaft.  The current 1040 count value is the initial calibration value
 * inferred from the recorded hand-turn counts; confirm it with a marked exact
 * wheel revolution before accepting physical speed accuracy.
 */
#define WHEEL_DIAMETER_MM                    65.0f
#define ENCODER_A_COUNTS_PER_WHEEL_REV     1040.0f
#define ENCODER_B_COUNTS_PER_WHEEL_REV     1040.0f
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

#endif
