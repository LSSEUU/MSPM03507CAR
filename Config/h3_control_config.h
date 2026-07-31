#ifndef H3_CONTROL_CONFIG_H
#define H3_CONTROL_CONFIG_H

/* H3 owns K1 while the H2 selector is displayed.  Both defaults are the
 * competition path; host tests may still override them on the command line. */
#ifndef H3_DEVELOPMENT_ENTRY
#define H3_DEVELOPMENT_ENTRY                    1U
#endif
#ifndef H3_SINGLE_TARGET_ONLY
#define H3_SINGLE_TARGET_ONLY                   0U
#endif

/* r77 hardware evidence: target +50 mm with a negative requested tilt moved
 * the ball to -66.1 mm.  Positive local tilt is therefore the +x direction
 * for the current lever/camera installation. */
#define H3_DIRECTION_SIGN                        1

#define H3_TARGET_POSITION_MM                   50L
#define H3_PASS_POSITION_TOLERANCE_MM           10L
#define H3_START_POSITION_LIMIT_MM              35L
#define H3_SETTLE_POSITION_LIMIT_MM              7L
#define H3_SETTLE_SPEED_LIMIT_MM_S              20L
#define H3_SETTLE_DWELL_MS                     250UL
#define H3_STAGE_REVERSAL_GRACE_MS             1200UL
#define H3_MINUS_TARGET_RAMP_MS                1500UL
#define H3_READY_DWELL_MS                      300UL
#define H3_RUN_TIMEOUT_MS                     6500UL
#define H3_SCORE_TIME_LIMIT_MS                5000UL

#define H3_MEASURED_POSITIVE_BREAKAWAY_MDEG   4911L
#define H3_BREAKAWAY_START_MDEG               8000L
#define H3_BREAKAWAY_MAX_MDEG                18000L
#define H3_PLUS_BREAKAWAY_MAX_MDEG           15000L
#define H3_TUNE_POSITIVE_MAX_MDEG             30000U
#define H3_TUNE_NEGATIVE_MAX_MDEG             19000U
#define H3_BREAKAWAY_TIMEOUT_MS               3000UL
#define H3_CONTROL_KP_MDEG_PER_MM              105L
#define H3_CONTROL_KD_MDEG_PER_MM_S             36L
#define H3_CONTROL_KI_MDEG_PER_MM_S              0L
#define H3_INTEGRAL_LIMIT_MDEG                 3000L
#define H3_INTEGRAL_ACTIVE_SPEED_MM_S             5L
#define H3_INTEGRAL_SETTLE_ERROR_X10             50L
#define H3_INTEGRAL_DECAY_MDEG_PER_20MS         100L
#define H3_TILT_LIMIT_MDEG                   18000L
#define H3_TILT_GUARD_LIMIT_MDEG               600L
#define H3_TILT_GUARD_MS                       200UL
#define H3_TILT_SLEW_MDEG_PER_20MS             200L
#define H3_BREAKAWAY_TILT_SLEW_MDEG_PER_20MS  1000L
#define H3_BRAKE_TILT_SLEW_MDEG_PER_20MS      1500L
#define H3_LAUNCH_PULSE_FEEDBACK_MDEG          8000L
#define H3_LAUNCH_RETRY_STEP_MDEG              1000L
#define H3_LAUNCH_MAX_PULSES                       2U
#define H3_LAUNCH_REARM_LEVEL_COUNTS             256L
#define H3_LAUNCH_REARM_SPEED_MM_S                10L
/* r150 repeatedly reached +25..35 mm but the encoder-confirmed 6.5 degree
 * relaunch pulse could not overcome ball friction.  Positive relaunch uses
 * the already validated 8 degree launch limit; negative remains unchanged. */
#define H3_PLUS_RELAUNCH_TILT_MDEG               8000L
#define H3_MINUS_RELAUNCH_TILT_MDEG              6500L
#define H3_RELAUNCH_ERROR_X10                     100L
#define H3_RELAUNCH_STILL_SPEED_MM_S                5L
#define H3_RELAUNCH_MOVING_SPEED_MM_S               8L
#define H3_RELAUNCH_DWELL_MS                      100UL
#define H3_PLUS_TOPUP_TRIGGER_X10                  150L
#define H3_PLUS_TOPUP_MAX_SPEED_MM_S                 5L
#define H3_PLUS_TOPUP_RECOVERY_SPEED_MM_S           30L
#define H3_PLUS_TOPUP_TILT_MDEG                   8000L
#define H3_PLUS_TOPUP_BRAKE_TRIGGER_X10            250L
#define H3_PLUS_HARD_BRAKE_PREDICT_MS              500UL
#define H3_PLUS_HARD_BRAKE_TRIGGER_X10              540L
#define H3_PLUS_HARD_BRAKE_ARM_SPEED_MM_S              1L
#define H3_PLUS_HARD_BRAKE_RELEASE_MM_S               20L
#define H3_PLUS_PREPASS_BRAKE_TILT_MDEG              4000L
#define H3_PLUS_HARD_BRAKE_TILT_MDEG               10000L
/* Hardware runs with identical parameters released the -8 degree pass brake
 * anywhere from 160 to 461 ms after crossing +40 mm.  That timing spread
 * produced either a short -26 mm reversal or a -93 mm overshoot.  Preserve
 * the measured successful r151 dwell: only RUN_MINUS uses this fixed hold;
 * RUN_PLUS retains the original velocity release so the scored pass is not
 * changed before +40 mm. */
#define H3_PLUS_TO_MINUS_BRAKE_HOLD_MS               300UL
#define H3_PLUS_TO_MINUS_BRAKE_DELAY_MS               50UL
#define H3_MINUS_HARD_BRAKE_PREDICT_MS            500UL
#define H3_MINUS_HARD_BRAKE_TRIGGER_X10             400L
#define H3_MINUS_HARD_BRAKE_ARM_SPEED_MM_S             1L
#define H3_MINUS_HARD_BRAKE_RELEASE_MM_S             10L
#define H3_MINUS_HARD_BRAKE_TILT_MDEG              4000L
/* The +40 pass brake can be anywhere from about -6 to -9 degrees when the
 * negative counter-tilt is accepted.  Normalize the commanded reversal
 * around the measured successful -7 degree encoder point. */
#define H3_MINUS_BRAKE_ENCODER_REFERENCE_MDEG       7000L
#define H3_MINUS_BRAKE_ADAPTIVE_MIN_MDEG            1500L
#define H3_MINUS_BRAKE_ADAPTIVE_MAX_MDEG            6000L
/* The camera velocity trails the observed position reversal.  Hardware r131
 * reached its physical minimum at -46.1 mm while the filtered velocity still
 * reported -57 mm/s, so waiting for x <= -50 held +5 degrees indefinitely
 * and launched the ball back to the positive edge.  A 75 ms projection
 * requests the measured mechanical return before that hidden reversal. */
#define H3_MINUS_BRAKE_RETURN_PREDICT_MS              75UL

#define H3_DIRECTION_CONFIRM_POSITION_X10       10L
#define H3_DIRECTION_REVERSE_POSITION_X10       20L
#define H3_DIRECTION_REVERSE_SPEED_MM_S          15L
#define H3_BALL_EDGE_LIMIT_MM                    58L
#define H3_BALL_EDGE_PREDICT_MS                 200UL

#define H3_LEVER_POSITIVE_HARD_LIMIT_MDEG     32000L
#define H3_LEVER_NEGATIVE_HARD_LIMIT_MDEG     20500L
#define H3_LEVER_VELOCITY_RPM                     3U
#define H3_LEVER_VELOCITY_ACCELERATION             0U
#define H3_LEVER_BREAKAWAY_VELOCITY_RPM             5U
#define H3_PLUS_LEVER_BREAKAWAY_VELOCITY_RPM        7U
#define H3_LEVER_BREAKAWAY_VELOCITY_ACCELERATION    0U
#define H3_LEVER_START_DEADBAND_COUNTS           128L
#define H3_LEVER_BRAKE_MARGIN_COUNTS              256L
#define H3_LEVER_REVERSE_REARM_COUNTS             384L
#define H3_LEVER_BRAKE_HOLD_MS                    100UL
#define H3_LEVER_BRAKE_SETTLE_COUNTS                 8L
#define H3_LEVER_BRAKE_SETTLE_SAMPLES                2U
#define H3_LEVER_COMPLETION_TOLERANCE_COUNTS     32L
#define H3_LEVER_COMPLETION_SAMPLES               2U
#define H3_LEVER_PROGRESS_MIN_COUNTS               8L
#define H3_LEVER_POLARITY_PROBE_COUNTS              8L
#define H3_LEVER_POLARITY_MAX_ATTEMPTS               2U
#define H3_LEVER_PROGRESS_TIMEOUT_MS             600UL
#define H3_LEVER_DIRECTION_FAULT_COUNTS         384L
#define H3_LEVER_FEEDBACK_DISAGREEMENT_COUNTS   768L
#define H3_LEVER_FEEDBACK_HANDOFF_MS             150UL
#define H3_LEVER_RETURN_HOLD_TIMEOUT_MS         2000UL
#define H3_LEVER_RETURN_FAST_VELOCITY_RPM           5U
#define H3_LEVER_RETURN_VELOCITY_RPM                3U
#define H3_LEVER_RETURN_VELOCITY_ACCELERATION       0U
#define H3_LEVER_RETURN_SLOW_ZONE_COUNTS          1024L
#define H3_LEVER_WITHDRAW_FAST_VELOCITY_RPM         5U
#define H3_LEVER_WITHDRAW_VELOCITY_RPM              2U
#define H3_LEVER_WITHDRAW_VELOCITY_ACCELERATION     0U
#define H3_LEVER_RETURN_SPEED_RPM                  1U
#define H3_LEVER_RETURN_ACCELERATION               0U

#define H3_DEBUG_PERIOD_MS                       50UL

_Static_assert((H3_DIRECTION_SIGN == 1) || (H3_DIRECTION_SIGN == -1),
               "H3 direction sign must be +/-1");
_Static_assert(H3_PASS_POSITION_TOLERANCE_MM > 0L &&
               H3_PASS_POSITION_TOLERANCE_MM < H3_TARGET_POSITION_MM,
               "H3 intermediate pass window is invalid");
_Static_assert(H3_SCORE_TIME_LIMIT_MS < H3_RUN_TIMEOUT_MS,
               "H3 mission timeout must exceed the five-second score limit");
_Static_assert(H3_STAGE_REVERSAL_GRACE_MS < H3_BREAKAWAY_TIMEOUT_MS,
               "H3 pass-through reversal grace must remain bounded");
_Static_assert(H3_STAGE_REVERSAL_GRACE_MS < H3_MINUS_TARGET_RAMP_MS &&
               H3_MINUS_TARGET_RAMP_MS < H3_SCORE_TIME_LIMIT_MS,
               "H3 negative target trajectory is invalid");
_Static_assert((H3_LEVER_POSITIVE_HARD_LIMIT_MDEG -
                H3_TUNE_POSITIVE_MAX_MDEG) >= 200L &&
               (H3_LEVER_NEGATIVE_HARD_LIMIT_MDEG -
                H3_TUNE_NEGATIVE_MAX_MDEG) >= 200L,
               "H3 tune range needs margin inside local hard limits");
_Static_assert((H3_TILT_LIMIT_MDEG -
                H3_MEASURED_POSITIVE_BREAKAWAY_MDEG) >= 200L,
               "H3 tilt cannot reach the measured ball breakaway point");
_Static_assert(H3_BREAKAWAY_START_MDEG >
               H3_MEASURED_POSITIVE_BREAKAWAY_MDEG,
               "H3 breakaway kick must exceed the measured static point");
_Static_assert(H3_BREAKAWAY_START_MDEG < H3_BREAKAWAY_MAX_MDEG &&
               H3_BREAKAWAY_MAX_MDEG <= H3_TILT_LIMIT_MDEG,
               "H3 automatic breakaway range is invalid");
_Static_assert(H3_BREAKAWAY_START_MDEG < H3_PLUS_BREAKAWAY_MAX_MDEG &&
               H3_PLUS_BREAKAWAY_MAX_MDEG <= H3_BREAKAWAY_MAX_MDEG,
               "H3 positive breakaway cap is invalid");
_Static_assert(H3_LAUNCH_PULSE_FEEDBACK_MDEG >
               H3_MEASURED_POSITIVE_BREAKAWAY_MDEG &&
               H3_LAUNCH_MAX_PULSES > 0U &&
               (H3_LAUNCH_PULSE_FEEDBACK_MDEG +
                H3_LAUNCH_RETRY_STEP_MDEG *
                (H3_LAUNCH_MAX_PULSES - 1U)) <=
               H3_BREAKAWAY_MAX_MDEG,
               "H3 launch pulse feedback threshold is invalid");
_Static_assert(H3_LAUNCH_REARM_LEVEL_COUNTS >=
               H3_LEVER_START_DEADBAND_COUNTS &&
               H3_LAUNCH_REARM_SPEED_MM_S <
               H3_SETTLE_SPEED_LIMIT_MM_S,
               "H3 launch rearm window is invalid");
_Static_assert(H3_PLUS_RELAUNCH_TILT_MDEG >
               H3_MEASURED_POSITIVE_BREAKAWAY_MDEG &&
               H3_PLUS_RELAUNCH_TILT_MDEG <=
               H3_LAUNCH_PULSE_FEEDBACK_MDEG &&
               H3_MINUS_RELAUNCH_TILT_MDEG >
               H3_MEASURED_POSITIVE_BREAKAWAY_MDEG &&
               H3_MINUS_RELAUNCH_TILT_MDEG <=
               H3_LAUNCH_PULSE_FEEDBACK_MDEG &&
               H3_RELAUNCH_STILL_SPEED_MM_S <
               H3_RELAUNCH_MOVING_SPEED_MM_S &&
               H3_RELAUNCH_MOVING_SPEED_MM_S <=
               H3_LAUNCH_REARM_SPEED_MM_S,
               "H3 bounded relaunch profile is invalid");
_Static_assert(H3_PLUS_TOPUP_TRIGGER_X10 > 0L &&
               H3_PLUS_TOPUP_TRIGGER_X10 <
               (H3_TARGET_POSITION_MM - H3_PASS_POSITION_TOLERANCE_MM) *
                   10L &&
               H3_PLUS_TOPUP_BRAKE_TRIGGER_X10 >
               H3_PLUS_TOPUP_TRIGGER_X10 &&
               H3_PLUS_TOPUP_BRAKE_TRIGGER_X10 <
               (H3_TARGET_POSITION_MM - H3_PASS_POSITION_TOLERANCE_MM) *
                   10L &&
               H3_PLUS_TOPUP_MAX_SPEED_MM_S > 0L &&
               H3_PLUS_TOPUP_RECOVERY_SPEED_MM_S >
               H3_PLUS_TOPUP_MAX_SPEED_MM_S &&
               H3_PLUS_TOPUP_TILT_MDEG >=
               H3_PLUS_RELAUNCH_TILT_MDEG &&
               H3_PLUS_TOPUP_TILT_MDEG <= H3_LAUNCH_PULSE_FEEDBACK_MDEG,
               "H3 positive top-up profile is invalid");
_Static_assert(H3_MINUS_HARD_BRAKE_PREDICT_MS >
               H3_BALL_EDGE_PREDICT_MS &&
               H3_MINUS_HARD_BRAKE_TRIGGER_X10 > 0L &&
               H3_MINUS_HARD_BRAKE_TRIGGER_X10 <=
               (H3_TARGET_POSITION_MM - H3_PASS_POSITION_TOLERANCE_MM) *
                   10L &&
               H3_MINUS_HARD_BRAKE_RELEASE_MM_S >
               H3_MINUS_HARD_BRAKE_ARM_SPEED_MM_S &&
               H3_MINUS_HARD_BRAKE_TILT_MDEG >
               H3_MINUS_BRAKE_ADAPTIVE_MIN_MDEG &&
               H3_MINUS_HARD_BRAKE_TILT_MDEG <= H3_TILT_LIMIT_MDEG,
               "H3 negative-stage hard brake is invalid");
_Static_assert(H3_MINUS_BRAKE_ENCODER_REFERENCE_MDEG > 0L &&
               H3_MINUS_BRAKE_ENCODER_REFERENCE_MDEG <
               H3_PLUS_HARD_BRAKE_TILT_MDEG &&
               H3_MINUS_BRAKE_ADAPTIVE_MIN_MDEG > 0L &&
               H3_MINUS_BRAKE_ADAPTIVE_MIN_MDEG <
               H3_MINUS_HARD_BRAKE_TILT_MDEG &&
               H3_MINUS_BRAKE_ADAPTIVE_MAX_MDEG >
               H3_MINUS_HARD_BRAKE_TILT_MDEG &&
               H3_MINUS_BRAKE_ADAPTIVE_MAX_MDEG <= H3_TILT_LIMIT_MDEG,
               "H3 adaptive negative brake bounds are invalid");
_Static_assert(H3_PLUS_HARD_BRAKE_PREDICT_MS >
               H3_BALL_EDGE_PREDICT_MS &&
               H3_PLUS_HARD_BRAKE_TRIGGER_X10 > 0L &&
               H3_PLUS_HARD_BRAKE_TRIGGER_X10 <=
               (H3_BALL_EDGE_LIMIT_MM + 2L) * 10L &&
               H3_PLUS_HARD_BRAKE_RELEASE_MM_S >
               H3_PLUS_HARD_BRAKE_ARM_SPEED_MM_S &&
               H3_PLUS_PREPASS_BRAKE_TILT_MDEG > 0L &&
               H3_PLUS_PREPASS_BRAKE_TILT_MDEG <
               H3_PLUS_HARD_BRAKE_TILT_MDEG &&
               H3_PLUS_HARD_BRAKE_TILT_MDEG >=
               H3_TILT_GUARD_LIMIT_MDEG &&
               H3_PLUS_HARD_BRAKE_TILT_MDEG <= H3_TILT_LIMIT_MDEG,
               "H3 positive-stage predictive brake is invalid");
_Static_assert(H3_PLUS_TO_MINUS_BRAKE_HOLD_MS >
               H3_PLUS_TO_MINUS_BRAKE_DELAY_MS &&
               H3_PLUS_TO_MINUS_BRAKE_HOLD_MS < H3_MINUS_TARGET_RAMP_MS,
               "H3 plus-to-minus brake handoff timing is invalid");
_Static_assert(((H3_MINUS_RELAUNCH_TILT_MDEG -
                 H3_MEASURED_POSITIVE_BREAKAWAY_MDEG) *
                65536L / 360000L) >
               H3_LEVER_START_DEADBAND_COUNTS,
               "H3 relaunch must cross the actuator start deadband");
_Static_assert(H3_BREAKAWAY_TIMEOUT_MS < H3_RUN_TIMEOUT_MS,
               "H3 no-roll return must precede the mission timeout");
_Static_assert(H3_TILT_GUARD_LIMIT_MDEG <= H3_TILT_LIMIT_MDEG,
               "H3 direction guard exceeds normal tilt limit");
_Static_assert(H3_BRAKE_TILT_SLEW_MDEG_PER_20MS >=
               H3_BREAKAWAY_TILT_SLEW_MDEG_PER_20MS &&
               H3_BREAKAWAY_TILT_SLEW_MDEG_PER_20MS >=
               H3_TILT_SLEW_MDEG_PER_20MS,
               "H3 braking slew must not be slower than normal tracking");
_Static_assert(H3_INTEGRAL_LIMIT_MDEG < H3_TILT_LIMIT_MDEG &&
               H3_INTEGRAL_ACTIVE_SPEED_MM_S <
               H3_SETTLE_SPEED_LIMIT_MM_S,
               "H3 integral guards are invalid");
_Static_assert(H3_LEVER_BREAKAWAY_VELOCITY_RPM >
               H3_LEVER_VELOCITY_RPM,
               "H3 breakaway profile needs a real initial velocity boost");
_Static_assert(H3_PLUS_LEVER_BREAKAWAY_VELOCITY_RPM >
               H3_LEVER_BREAKAWAY_VELOCITY_RPM &&
               H3_PLUS_LEVER_BREAKAWAY_VELOCITY_RPM <= 8U,
               "H3 positive launch velocity is invalid");
_Static_assert(H3_DIRECTION_CONFIRM_POSITION_X10 > 0L &&
               H3_DIRECTION_REVERSE_POSITION_X10 > 0L,
               "H3 ball direction evidence must be nonzero");
_Static_assert(H3_BALL_EDGE_LIMIT_MM >
               (H3_TARGET_POSITION_MM + H3_SETTLE_POSITION_LIMIT_MM),
               "H3 edge guard must leave room for the settle window");
_Static_assert(H3_LEVER_COMPLETION_TOLERANCE_COUNTS <
               H3_LEVER_START_DEADBAND_COUNTS,
               "H3 completion tolerance must remain below start deadband");
_Static_assert(H3_LEVER_START_DEADBAND_COUNTS <
               H3_LEVER_BRAKE_MARGIN_COUNTS &&
               H3_LEVER_BRAKE_MARGIN_COUNTS <=
               H3_LEVER_REVERSE_REARM_COUNTS,
               "H3 velocity brake windows are invalid");
_Static_assert(H3_LEVER_BRAKE_SETTLE_COUNTS <=
               H3_LEVER_START_DEADBAND_COUNTS &&
               H3_LEVER_BRAKE_SETTLE_SAMPLES >= 2U,
               "H3 reversal requires confirmed stationary feedback");
_Static_assert(H3_LEVER_POLARITY_PROBE_COUNTS <=
               H3_LEVER_DIRECTION_FAULT_COUNTS,
               "H3 velocity polarity must be learned before direction fault");
_Static_assert(H3_LEVER_COMPLETION_SAMPLES >= 2U,
               "H3 completion requires settled encoder feedback");
_Static_assert(H3_LEVER_FEEDBACK_HANDOFF_MS <
               H3_LEVER_PROGRESS_TIMEOUT_MS,
               "H3 feedback handoff must remain below progress timeout");
_Static_assert(H3_LEVER_PROGRESS_TIMEOUT_MS <
               H3_LEVER_RETURN_HOLD_TIMEOUT_MS,
               "H3 final position hold needs the longer deadline");
_Static_assert(H3_LEVER_RETURN_FAST_VELOCITY_RPM >=
               H3_LEVER_BREAKAWAY_VELOCITY_RPM,
               "H3 far safety return must not level slower than launch");
_Static_assert(H3_LEVER_RETURN_VELOCITY_RPM >=
               H3_LEVER_VELOCITY_RPM &&
               H3_LEVER_RETURN_VELOCITY_RPM <
               H3_LEVER_RETURN_FAST_VELOCITY_RPM &&
               H3_LEVER_RETURN_SLOW_ZONE_COUNTS >
               H3_LEVER_BRAKE_MARGIN_COUNTS,
               "H3 staged safety return profile is invalid");
_Static_assert(H3_LEVER_RETURN_SPEED_RPM <=
               H3_LEVER_RETURN_VELOCITY_RPM,
               "H3 final position hold must be slower than velocity return");
_Static_assert(H3_LEVER_WITHDRAW_FAST_VELOCITY_RPM ==
               H3_LEVER_RETURN_FAST_VELOCITY_RPM &&
               H3_LEVER_WITHDRAW_VELOCITY_RPM <
               H3_LEVER_WITHDRAW_FAST_VELOCITY_RPM,
               "H3 active withdrawal must reproduce the r94 return profile");

#endif
