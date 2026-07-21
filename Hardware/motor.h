/*
 * motor.h  —  电机驱动 (TB6612 / AT8236 / L298N)
 * 参考: WHEELTEC_C07A_Bluetooth/Hardware/motor.h
 */
#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

/* ── 电机编号 ── */
#define MOTOR_A  0
#define MOTOR_B  1

/* ── 电机参数结构体 ── */
typedef struct {
    float Current_Encoder;    /* 当前编码器速度 */
    float Motor_Pwm;          /* 当前 PWM 占空比 */
    float Target_Encoder;     /* 目标速度 */
    float Velocity;           /* 实际速度 mm/s */
} MotorParam_t;

extern MotorParam_t MotorA, MotorB;

typedef struct {
    uint32_t command_count;
    uint32_t clamp_count;
    int16_t  last_a;
    int16_t  last_b;
} MotorStats_t;

/* ── API ── */
void Motor_Init(void);                            /* 初始化 GPIO + PWM */
void Motor_SetPWM(int16_t motor_a, int16_t motor_b); /* 设置两路 PWM */
void Motor_SetDir(uint8_t motor, uint8_t dir);    /* 设置方向 */
void Motor_Stop(void);                             /* 急停 */
void Motor_GetStats(MotorStats_t *stats);

#endif
