#ifndef H4_BALL_CONTROL_CONFIG_H
#define H4_BALL_CONTROL_CONFIG_H

#define H4_BALL_TARGET_POSITION_X10                         0L
#define H4_BALL_START_POSITION_LIMIT_X10                  100L
#define H4_BALL_POSITION_DEADBAND_X10                      10U
#define H4_BALL_EDGE_LIMIT_X10                           1000L
#define H4_BALL_EDGE_PREDICT_MS                           200UL

/* The outer-loop gain reaches the measured breakaway region before the
 * internal 7 mm H4 release target.  The speed-loop gain retains the proven
 * H3 velocity damping scale while making the two loops independently
 * observable and tunable. */
#define H4_BALL_POSITION_KP_X1000_PER_S                 24000UL
#define H4_BALL_POSITION_KI_X1000_PER_S2                    0UL
#define H4_BALL_POSITION_INTEGRAL_LIMIT_MM_S               40U
#define H4_BALL_VELOCITY_LIMIT_MM_S                        220U
#define H4_BALL_SPEED_KP_X100_MDEG_PER_MM_S               3600UL
#define H4_BALL_SPEED_KI_X100_MDEG_PER_MM                    0UL
#define H4_BALL_SPEED_INTEGRAL_LIMIT_MDEG                  3000U
#define H4_BALL_POSITIVE_TILT_LIMIT_MDEG                  12000U
#define H4_BALL_NEGATIVE_TILT_LIMIT_MDEG                  12000U
#define H4_BALL_TILT_SLEW_MDEG_PER_20MS                    1000U
#define H4_BALL_DIRECTION_SIGN                                1

#define H4_BALL_TRACKING_RPM                                  3U
#define H4_BALL_RETURN_RPM                                    3U

_Static_assert((H4_BALL_DIRECTION_SIGN == 1) ||
               (H4_BALL_DIRECTION_SIGN == -1),
               "H4 ball direction sign must be +/-1");
_Static_assert(H4_BALL_POSITION_DEADBAND_X10 <
               H4_BALL_START_POSITION_LIMIT_X10,
               "H4 ball deadband must fit inside the start window");
_Static_assert(H4_BALL_START_POSITION_LIMIT_X10 <
               H4_BALL_EDGE_LIMIT_X10 &&
               H4_BALL_EDGE_LIMIT_X10 < 1150L,
               "H4 ball start/edge windows exceed the vision range");
_Static_assert(H4_BALL_VELOCITY_LIMIT_MM_S > 0U &&
               H4_BALL_VELOCITY_LIMIT_MM_S < 4000U,
               "H4 ball target velocity is outside the vision contract");
_Static_assert(H4_BALL_POSITIVE_TILT_LIMIT_MDEG <= 18000U &&
               H4_BALL_NEGATIVE_TILT_LIMIT_MDEG <= 18000U,
               "H4 ball tilt exceeds the bounded control envelope");
_Static_assert(H4_BALL_TRACKING_RPM > 0U &&
               H4_BALL_TRACKING_RPM <= 8U &&
               H4_BALL_RETURN_RPM > 0U && H4_BALL_RETURN_RPM <= 8U,
               "H4 ball actuator speed is invalid");

#endif
