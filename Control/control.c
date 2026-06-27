#include "control.h"

float Target_Velocity = 0.0f;
float Velocity_KP = 0.5f, Velocity_KI = 0.02f;

static float PID_Clamp(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd,
              float out_lo, float out_hi)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = out_lo;
    pid->output_max = out_hi;
    pid->integral_min = out_lo;
    pid->integral_max = out_hi;
    PID_Reset(pid);
}

void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;
    float derivative = error - pid->previous_error;

    pid->integral += error;
    pid->integral = PID_Clamp(pid->integral,
        pid->integral_min, pid->integral_max);

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    output = PID_Clamp(output, pid->output_min, pid->output_max);
    pid->previous_error = error;

    return output;
}
