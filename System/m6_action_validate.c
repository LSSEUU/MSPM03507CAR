#include "m6_action_validate.h"

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

M6Result_t M6Action_Validate(const uint8_t *body, uint16_t body_length,
                             uint64_t enabled_capabilities)
{
    int16_t left;
    int16_t right;
    int32_t angle;
    uint32_t duration;
    uint64_t required;
    if ((body == 0) || (body_length != 24U)) return M6_RESULT_BAD_LENGTH;
    if ((get_u16(&body[2]) != 0U) || ((int32_t)get_u32(&body[12]) != 0))
        return M6_RESULT_INVALID_ARGUMENT;
    if (body[0] == M6_ACTION_KIND_WHEEL_SPEED_TEST)
        required = M6_CAP_WHEEL_DIAG_ACTION;
    else if ((body[0] == M6_ACTION_KIND_HEADING_HOLD) ||
             (body[0] == M6_ACTION_KIND_HEADING_TURN))
        required = M6_CAP_HEADING_ACTION;
    else
        return M6_RESULT_UNSUPPORTED_CAPABILITY;
    if ((enabled_capabilities & required) != required)
        return M6_RESULT_UNSUPPORTED_CAPABILITY;

    left = (int16_t)get_u16(&body[4]);
    right = (int16_t)get_u16(&body[6]);
    angle = (int32_t)get_u32(&body[8]);
    duration = get_u32(&body[16]);
    if (body[0] == M6_ACTION_KIND_WHEEL_SPEED_TEST) {
        if ((body[1] < 1U) || (body[1] > 3U) || (angle != 0) ||
            (((body[1] & 1U) == 0U) && (left != 0)) ||
            (((body[1] & 2U) == 0U) && (right != 0)) ||
            (((body[1] & 1U) != 0U) && (left > -50) && (left < 50)) ||
            (((body[1] & 2U) != 0U) && (right > -50) && (right < 50)))
            return M6_RESULT_INVALID_ARGUMENT;
        if ((left < -400) || (left > 400) ||
            (right < -400) || (right > 400) ||
            (duration < 100U) || (duration > 10000U))
            return M6_RESULT_OUT_OF_RANGE;
    } else if (body[0] == M6_ACTION_KIND_HEADING_HOLD) {
        if ((body[1] != 3U) || (left != right) ||
            ((left > -50) && (left < 50)) || (angle != 0))
            return M6_RESULT_INVALID_ARGUMENT;
        if ((left < -400) || (left > 400) ||
            (duration < 100U) || (duration > 30000U))
            return M6_RESULT_OUT_OF_RANGE;
    } else {
        if ((body[1] != 3U) || (left != 0) || (right != 0))
            return M6_RESULT_INVALID_ARGUMENT;
        if ((angle > 360000) || (angle < -360000) ||
            ((angle > -1000) && (angle < 1000)) ||
            (duration < 500U) || (duration > 30000U))
            return M6_RESULT_OUT_OF_RANGE;
        if (((enabled_capabilities & M6_CAP_CONFIGURABLE_TURN) == 0U) &&
            (angle != 90000) && (angle != -90000))
            return M6_RESULT_UNSUPPORTED_CAPABILITY;
    }
    return M6_RESULT_OK;
}
