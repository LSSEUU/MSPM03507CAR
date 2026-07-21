#include "motor.h"
#include "board.h"

#include <stddef.h>
#include <string.h>

static MotorStats_t motor_stats;

static int16_t Motor_ClampPWM(int16_t pwm)
{
    if (pwm > 999)  { motor_stats.clamp_count++; return 999; }
    if (pwm < -999) { motor_stats.clamp_count++; return -999; }
    return pwm;
}

static uint16_t Motor_PWMMagnitude(int16_t pwm)
{
    return (uint16_t)((pwm < 0) ? -pwm : pwm);
}

void Motor_Init(void)
{
    memset(&motor_stats, 0, sizeof(motor_stats));
    Motor_Stop();
}

void Motor_SetPWM(int16_t motor_a, int16_t motor_b)
{
    motor_a = Motor_ClampPWM(motor_a);
    motor_b = Motor_ClampPWM(motor_b);

    motor_stats.command_count++;
    motor_stats.last_a = motor_a;
    motor_stats.last_b = motor_b;

    if (motor_a > 0) {
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
    } else if (motor_a < 0) {
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);
    } else {
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    }
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, Motor_PWMMagnitude(motor_a),
                                     GPIO_PWM_0_C1_IDX);

    if (motor_b > 0) {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);
    } else if (motor_b < 0) {
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_setPins(BIN_PORT, BIN_BIN2_PIN);
    } else {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
    }
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, Motor_PWMMagnitude(motor_b),
                                     GPIO_PWM_0_C0_IDX);
}

void Motor_Stop(void)
{
    DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    motor_stats.last_a = 0;
    motor_stats.last_b = 0;
}

void Motor_GetStats(MotorStats_t *stats)
{
    if (stats != NULL) *stats = motor_stats;
}
