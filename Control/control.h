#ifndef CONTROL_H
#define CONTROL_H
#include "board.h"

typedef struct {
    float kp, ki, kd;
    float integral;
    float previous_error;
    float output_min, output_max;
    float integral_min, integral_max;
} PID_HandleTypeDef;

void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd,
              float output_min, float output_max);
void PID_Reset(PID_HandleTypeDef *pid);
float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float feedback);

extern float Target_Velocity;

#endif
