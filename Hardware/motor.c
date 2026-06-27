
#include "motor.h"

MotorParam_t MotorA, MotorB;

static int16_t Motor_ClampPWM(int16_t pwm)
{
    if (pwm > 999)  return 999;
    if (pwm < -999) return -999;
    return pwm;
}

void Motor_Init(void)
{
    
    Motor_Stop();
}


void Motor_SetPWM(int16_t motor_a, int16_t motor_b)
{
    motor_a = Motor_ClampPWM(motor_a);
    motor_b = Motor_ClampPWM(motor_b);

    MotorA.Motor_Pwm = (float)motor_a;
    MotorB.Motor_Pwm = (float)motor_b;

    
    if (motor_a > 0) {
        
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
    } else if (motor_a < 0) {
        
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);
    } else {
        
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    }
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, ABS(motor_a),
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
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, ABS(motor_b),
                                     GPIO_PWM_0_C0_IDX);
}


void Motor_SetDir(uint8_t motor, uint8_t dir)
{
    if (motor == MOTOR_A) {
        if (dir) {
            DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
            DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);
        } else {
            DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
            DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
        }
    } else {
        if (dir) {
            DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);
            DL_GPIO_setPins(BIN_PORT, BIN_BIN2_PIN);
        } else {
            DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN);
            DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);
        }
    }
}

void Motor_Stop(void)
{
    
    DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    MotorA.Motor_Pwm = 0;
    MotorB.Motor_Pwm = 0;
}
