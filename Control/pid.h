#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float integral;
    float output_min;
    float output_max;
} PIController_t;

void PI_Init(PIController_t *controller, float kp, float ki,
             float output_min, float output_max);
void PI_Reset(PIController_t *controller);
void PI_SetGains(PIController_t *controller, float kp, float ki);
float PI_Update(PIController_t *controller, float error, float dt_s);

#endif
