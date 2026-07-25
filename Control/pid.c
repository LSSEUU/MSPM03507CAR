#include "pid.h"

#include <stddef.h>

static float PI_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

void PI_Init(PIController_t *controller, float kp, float ki,
             float output_min, float output_max)
{
    if (controller == NULL) return;
    controller->kp = (kp < 0.0f) ? 0.0f : kp;
    controller->ki = (ki < 0.0f) ? 0.0f : ki;
    controller->integral = 0.0f;
    controller->output_min = output_min;
    controller->output_max = output_max;
    controller->last_proportional = 0.0f;
    controller->last_feedforward = 0.0f;
    controller->last_unsaturated = 0.0f;
    controller->last_output = 0.0f;
    controller->last_saturated = false;
}

void PI_Reset(PIController_t *controller)
{
    if (controller == NULL) return;
    controller->integral = 0.0f;
    controller->last_proportional = 0.0f;
    controller->last_feedforward = 0.0f;
    controller->last_unsaturated = 0.0f;
    controller->last_output = 0.0f;
    controller->last_saturated = false;
}

void PI_SetGains(PIController_t *controller, float kp, float ki)
{
    if (controller == NULL) return;
    controller->kp = (kp < 0.0f) ? 0.0f : kp;
    controller->ki = (ki < 0.0f) ? 0.0f : ki;
}

float PI_Update(PIController_t *controller, float error, float dt_s)
{
    return PI_UpdateWithFeedforward(controller, error, 0.0f, dt_s);
}

float PI_UpdateWithFeedforward(PIController_t *controller, float error,
                               float feedforward, float dt_s)
{
    float proportional;
    float candidate_integral;
    float candidate_output;
    float output;

    if ((controller == NULL) || (dt_s <= 0.0f)) return 0.0f;
    proportional = controller->kp * error;
    candidate_integral = controller->integral +
                         (controller->ki * error * dt_s);
    candidate_output = proportional + candidate_integral + feedforward;
    output = PI_Clamp(candidate_output, controller->output_min,
                      controller->output_max);

    /* Conditional integration prevents wind-up while still unwinding. */
    if ((output == candidate_output) ||
        ((output >= controller->output_max) && (error < 0.0f)) ||
        ((output <= controller->output_min) && (error > 0.0f))) {
        controller->integral = candidate_integral;
    }
    controller->last_proportional = proportional;
    controller->last_feedforward = feedforward;
    controller->last_unsaturated = candidate_output;
    controller->last_output = output;
    controller->last_saturated = (output != candidate_output);
    return output;
}
