#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#define SYSTEM_TICK_HZ                  1000U

#define TASK_5MS_PERIOD_MS                 5U
#define TASK_10MS_PERIOD_MS               10U
#define TASK_20MS_PERIOD_MS               20U
#define TASK_100MS_PERIOD_MS             100U

#define GYRO_TIMEOUT_MS                  250U
#define GRAYSCALE_TIMEOUT_MS             300U
#define BOOT_SELF_TEST_SETTLE_MS          750U

#define WATCHDOG_SERVICE_PERIOD_MS         20U
/* WWDT0 uses LFCLK (nominally 32.768 kHz): 2^10 ticks is about 31 ms. */
#define WATCHDOG_PERIOD_DESCRIPTION_MS     31U

/* New powered-wheel-first chassis: motor A is the physical right wheel and
 * motor B is the physical left wheel.  Both raw motor polarities are opposite
 * to the new vehicle-forward direction. */
#define CHASSIS_MOTOR_A_FORWARD_SIGN        (-1)
#define CHASSIS_MOTOR_B_FORWARD_SIGN        (-1)
/* Measured contact-patch centre distance between the powered wheels. */
#define CHASSIS_DRIVE_TRACK_MM             214.2f

#define DIAGNOSTIC_MOTOR_PWM              250
#define DIAGNOSTIC_MOTOR_PHASE_MS         500U
#define DIAGNOSTIC_MOTOR_GAP_MS           150U
#define DIAGNOSTIC_ARM_TIMEOUT_MS       10000U
#define DIAGNOSTIC_ACTION_TIMEOUT_MS     3500U

#endif
