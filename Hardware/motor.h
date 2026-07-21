#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef struct {
    uint32_t command_count;
    uint32_t clamp_count;
    int16_t last_a;
    int16_t last_b;
} MotorStats_t;

void Motor_Init(void);
void Motor_SetPWM(int16_t motor_a, int16_t motor_b);
void Motor_Stop(void);
void Motor_GetStats(MotorStats_t *stats);

#endif
