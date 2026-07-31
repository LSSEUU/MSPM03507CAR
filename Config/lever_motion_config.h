#ifndef LEVER_MOTION_CONFIG_H
#define LEVER_MOTION_CONFIG_H

/* Mechanical landmarks captured from the Emm42 0x31 single-turn absolute
 * encoder on 2026-07-31.  0x31 survives MCU reset and is authoritative at
 * boot; 0x36 is retained only as an independent motion-delta cross-check. */
#define LEVER_ABSOLUTE_PROFILE_VALID               1U
#define LEVER_ABSOLUTE_BOTTOM_ENCODER           38857U
#define LEVER_ABSOLUTE_HORIZONTAL_ENCODER       43253U
#define LEVER_ABSOLUTE_MAX_ENCODER              52053U
#ifndef LEVER_BOOT_PROFILE_ENABLED
#define LEVER_BOOT_PROFILE_ENABLED                 1U
#endif
#define LEVER_POSITION_COUNTS_PER_REVOLUTION   65536L
#define LEVER_MECHANICAL_MIN_COUNTS                  0L
#define LEVER_HORIZONTAL_COUNTS                   4396L
#define LEVER_MECHANICAL_MAX_COUNTS              13196L

/* Runtime limits are inset by 10% of each measured horizontal-to-end span.
 * Recovery is allowed to start at the gravity reference below SOFT_MIN, but
 * normal closed-loop commands must remain inside this interval. */
#define LEVER_SOFT_MIN_COUNTS                       440L
#define LEVER_SOFT_MAX_COUNTS                     12316L

/* A fresh boot may recover to horizontal from anywhere inside the normal
 * software travel envelope.  Positions outside it are never enabled. */
#define LEVER_BOOT_RECOVERY_MIN_COUNTS LEVER_SOFT_MIN_COUNTS
#define LEVER_BOOT_RECOVERY_MAX_COUNTS LEVER_SOFT_MAX_COUNTS

#define LEVER_GRAVITY_STABILITY_TOLERANCE_COUNTS      8L
#define LEVER_MECHANICAL_TOLERANCE_COUNTS             64L
#define LEVER_BOOT_HORIZONTAL_TOLERANCE_COUNTS         64L
#define LEVER_BOOT_DIRECTION_TOLERANCE_COUNTS          16L
#define LEVER_BOOT_FEEDBACK_DISAGREEMENT_COUNTS        768L
#define LEVER_BOOT_REFERENCE_SAMPLE_COUNT               5U
/* On a simultaneous cold start the X42S becomes responsive after the MCU.
 * Do not address UART3 until it has had a deterministic power-up window. */
#define LEVER_BOOT_DRIVER_READY_MS                    2000UL
#define LEVER_BOOT_SETTLE_MS                          2500UL
#define LEVER_BOOT_REFERENCE_TIMEOUT_MS               5000UL
#define LEVER_BOOT_ENABLE_TIMEOUT_MS                   1000UL
#define LEVER_BOOT_MOVE_TIMEOUT_MS                     5000UL
#define LEVER_BOOT_POSITION_MAX_AGE_MS                  150UL
#define LEVER_BOOT_MOTOR_STATUS_MAX_AGE_MS              150UL
#define LEVER_BOOT_CONFIG_MAX_AGE_MS                   5000UL
#define LEVER_BOOT_ACTIVE_FEEDBACK_TIMEOUT_MS            500UL
#define LEVER_BOOT_HOME_SPEED_RPM                        10U
#define LEVER_BOOT_HOME_ACCELERATION                       5U

_Static_assert(LEVER_ABSOLUTE_PROFILE_VALID,
               "lever absolute position profile must be valid");
_Static_assert((LEVER_ABSOLUTE_HORIZONTAL_ENCODER -
                LEVER_ABSOLUTE_BOTTOM_ENCODER) ==
               LEVER_HORIZONTAL_COUNTS,
               "lever horizontal absolute and relative values disagree");
_Static_assert((LEVER_ABSOLUTE_MAX_ENCODER -
                LEVER_ABSOLUTE_BOTTOM_ENCODER) ==
               LEVER_MECHANICAL_MAX_COUNTS,
               "lever maximum absolute and relative values disagree");

_Static_assert(LEVER_MECHANICAL_MIN_COUNTS <
               LEVER_SOFT_MIN_COUNTS,
               "lever minimum soft limit must be inside the mechanism");
_Static_assert(LEVER_SOFT_MIN_COUNTS <
               LEVER_HORIZONTAL_COUNTS,
               "lever horizontal must be above the minimum soft limit");
_Static_assert(LEVER_HORIZONTAL_COUNTS <
               LEVER_SOFT_MAX_COUNTS,
               "lever horizontal must be below the maximum soft limit");
_Static_assert(LEVER_SOFT_MAX_COUNTS <
               LEVER_MECHANICAL_MAX_COUNTS,
               "lever maximum soft limit must be inside the mechanism");
_Static_assert(LEVER_BOOT_RECOVERY_MIN_COUNTS <=
               LEVER_HORIZONTAL_COUNTS,
               "boot recovery minimum must include horizontal");
_Static_assert(LEVER_HORIZONTAL_COUNTS <=
               LEVER_BOOT_RECOVERY_MAX_COUNTS,
               "boot recovery maximum must include horizontal");
_Static_assert(LEVER_BOOT_DRIVER_READY_MS < LEVER_BOOT_SETTLE_MS,
               "driver ready window must end before boot reference sampling");

#endif
