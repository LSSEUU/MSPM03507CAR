#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* MG513XP28 chassis calibration measured over 10 wheel revolutions.
 * Logical left is raw encoder B; logical right is raw encoder A. */
#define WHEEL_DIAMETER_MM                    65.0f
#define ENCODER_LEFT_COUNTS_PER_WHEEL_REV   736.4f
#define ENCODER_RIGHT_COUNTS_PER_WHEEL_REV  732.6f
#define ENCODER_LEFT_FORWARD_SIGN                1
#define ENCODER_RIGHT_FORWARD_SIGN              -1

#define WHEEL_SPEED_FILTER_ALPHA             0.25f
#define WHEEL_SPEED_MAX_TARGET_MM_S          700.0f
#define WHEEL_SPEED_MAX_PHYSICAL_MM_S       1500.0f
#define WHEEL_SPEED_MAX_JUMP_MM_S           1000.0f
#define WHEEL_SPEED_ACCEL_MM_S2              600.0f
#define WHEEL_SPEED_DECEL_MM_S2              900.0f

#define WHEEL_SPEED_PWM_LIMIT                500.0f
/* Keep the V1 parameter registry range stable while the physical output is
 * clamped by WHEEL_SPEED_PWM_LIMIT during initial chassis bring-up. */
#define WHEEL_FEEDFORWARD_PARAM_MAX          850.0f
#define WHEEL_SPEED_KP_LEFT                    0.70f
#define WHEEL_SPEED_KI_LEFT                    2.50f
#define WHEEL_SPEED_KP_RIGHT                   0.70f
#define WHEEL_SPEED_KI_RIGHT                   2.50f

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

/* OUT.8IR.B compatibility values are conditioned to active-high line bits.
 * Position is normalized to [-1, 1], left negative and right positive. */
#define LINE_VALID_CONFIDENCE                   0.20f
#define LINE_VALID_ACQUIRE_SAMPLES                 2U
#define LINE_VALID_RELEASE_SAMPLES                 2U
#define LINE_BRANCH_HINT_SAMPLES                   2U
#define LINE_BRANCH_RELEASE_SAMPLES               10U
#define LINE_BRANCH_RELEASE_MIN_ACTIVE              2U
#define LINE_BRANCH_RELEASE_POSITION              0.20f
#define LINE_EXPECTED_ACTIVE_SUM                 1.50f
/* Ignore weak calibrated reflectance drift in the continuous centroid.
 * Hysteretic bits remain the sole source of line/topology validity. */
#define LINE_ANALOG_STRENGTH_FLOOR                0.08f

/* OUT.8IR.B digital-mask conditioning and forward-only recovery.  These
 * safety bounds do not change the frozen parameter registry. */
#define LINE_DIGITAL_CENTER_DEADBAND               0.15f
#define LINE_DIGITAL_CENTER_CONTROL_SAMPLES             3U
#define LINE_DIGITAL_CENTER_HINT_SAMPLES                5U
#define LINE_DIGITAL_SINGLE_CENTER_POSITION_SCALE      0.50f
#define LINE_DIGITAL_DERIVATIVE_LIMIT               4.0f
#define LINE_DIGITAL_MAX_CORRECTION_SLEW          800.0f
#define LINE_DIGITAL_REACQUIRE_SAMPLES                 3U
#define LINE_DIGITAL_DIRECTION_SAMPLES                 2U
#define LINE_DIGITAL_LOSS_GRACE_MS                   120U
#define LINE_DIGITAL_AMBIGUOUS_BRIDGE_MS             250U
#define LINE_DIGITAL_SEARCH_CORRECTION_RATIO        0.40f

/* Keep the measured straight-line Kp unchanged near the centre, then add
 * proportional authority before a 0.5 m course arc can settle on the two
 * outermost sensors.  Smoothstep avoids a gain discontinuity at either
 * boundary and deliberately leaves parameter schema 1 unchanged. */
#define LINE_CURVE_KP_BOOST_START_POSITION           0.20f
#define LINE_CURVE_KP_BOOST_FULL_POSITION            0.45f
#define LINE_CURVE_KP_BOOST_RATIO                     0.55f

/* JY61P yaw-rate damping blended after the infrared steering controller.
 * The runtime gain intentionally reuses heading.straight.kd; these constants
 * bound and qualify the assist without changing parameter schema 1. */
#define LINE_YAW_ASSIST_RATE_DEADBAND_DEG_S          0.50f
#define LINE_YAW_ASSIST_LIMIT_MM_S                   8.0f
#define LINE_YAW_ASSIST_SLEW_MM_S2                 200.0f
#define LINE_YAW_ASSIST_MIN_BASE_MM_S               80.0f
#define LINE_YAW_ASSIST_GYRO_FRESH_MS                100U
#define LINE_YAW_ASSIST_MAX_DT_MS                    100U
#define LINE_YAW_ASSIST_MIN_INNER_MM_S              50.0f

/* Near the two centre probes, the digital mask alone cannot reveal the
 * vehicle's angle relative to a straight.  Keep a local gyro heading while
 * the line is central, and adapt that reference slowly from the line error so
 * a sub-degree placement error is learned instead of becoming a limit cycle.
 * An outer probe or sustained curve rate releases the local heading at once. */
#define LINE_STRAIGHT_HEADING_POSITION_MAX            0.12f
#define LINE_STRAIGHT_HEADING_OFFSET_DEG_PER_POSITION 12.0f
#define LINE_COURSE_EXIT_OFFSET_DEG_PER_POSITION        6.0f
#define LINE_STRAIGHT_HEADING_REFERENCE_ADAPT_DEG_S    2.5f
#define LINE_STRAIGHT_HEADING_RATE_LIMIT_DEG_S         6.0f
#define LINE_STRAIGHT_HEADING_REACQUIRE_RATE_DEG_S     4.0f
#define LINE_STRAIGHT_HEADING_REACQUIRE_MS           300U

/* H-course curve observer.  Once an outer probe proves a curve direction,
 * keep the 0.5 m-radius differential feed-forward even while the digital
 * line briefly returns to centre.  Infrared position is only a modest trim;
 * the gyro closes the yaw-rate loop and the integrated relative angle arms
 * the transition back to straight control near the end of a semicircle. */
#define LINE_CURVE_RADIUS_MM                           500.0f
#define LINE_CURVE_TARGET_POSITION                       0.10f
#define LINE_CURVE_POSITION_TRIM_MM_S                   10.0f
#define LINE_CURVE_DERIVATIVE_TRIM_MM_S_PER_UNIT_S       6.0f
#define LINE_CURVE_DESIRED_CORRECTION_LIMIT_MM_S        50.0f
#define LINE_CURVE_YAW_ASSIST_LIMIT_MM_S                40.0f
#define LINE_CURVE_CONFIRM_TURN_DEG                      15.0f
#define LINE_CURVE_EXIT_ARM_DEG                         175.0f
#define LINE_CURVE_FORCE_EXIT_DEG                       195.0f
#define LINE_CURVE_EXIT_OPPOSITE_POSITION                 0.20f
#define LINE_CURVE_EXIT_STRAIGHT_GUARD_MS               700U

/* Fixed H-course geometry: AB/CD are 1.5 m straights and BC/DA are
 * clockwise 0.5 m-radius semicircles.  Encoder distance opens a narrow
 * curve-entry window; infrared confirms the edge, with a distance fallback
 * for placement and odometry tolerance. */
#define H_COURSE_STRAIGHT_ARM_MM                       1350.0f
#define H_COURSE_STRAIGHT_FORCE_MM                     1500.0f
#define H_COURSE_CURVE_EVIDENCE_POSITION                  0.18f
#define H_COURSE_CURVE_EVIDENCE_SAMPLES                    3U
/* Start the gyro-owned straight handoff before 180 degrees so the residual
 * curve yaw rate and correction decay into the planned outgoing heading
 * instead of carrying the chassis several degrees past it. */
#define H_COURSE_CURVE_EXIT_DEG                         170.0f
#define H_COURSE_EXIT_CAPTURE_MIN_MM                     100.0f
#define H_COURSE_EXIT_CAPTURE_MAX_MM                    1000.0f
#define H_COURSE_EXIT_CAPTURE_POSITION_MAX                 0.12f
#define H_COURSE_EXIT_CAPTURE_HEADING_MAX_DEG              3.0f
#define H_COURSE_EXIT_CAPTURE_RATE_MAX_DEG_S               6.0f
#define H_COURSE_EXIT_CAPTURE_SAMPLES                       10U
#define H_COURSE_MAX_DT_MS                                100U

#define LINE_FOLLOW_BASE_SPEED_MM_S             160.0f
#define LINE_FOLLOW_KP_MM_S                     150.0f
#define LINE_FOLLOW_KD_MM_S_PER_UNIT_S            1.0f
#define LINE_FOLLOW_MAX_CORRECTION_MM_S         150.0f
#define LINE_SEARCH_FORWARD_MM_S                 45.0f
#define LINE_SEARCH_TURN_MM_S                   105.0f
#define LINE_LOST_TIMEOUT_MS                     600U
#define LINE_FOLLOW_ACTION_TIMEOUT_MS         300000U

#endif
