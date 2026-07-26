#include "m6_action_validate.h"

#include <assert.h>
#include <string.h>

static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

int main(void)
{
    uint8_t action[24];
    memset(action, 0, sizeof(action));
    action[0] = M6_ACTION_KIND_WHEEL_SPEED_TEST;
    action[1] = 3U;
    action[4] = 100U;
    action[6] = 100U;
    put_u32(&action[16], 1500U);
    put_u32(&action[20], 1U);
    assert(M6Action_Validate(action, sizeof(action),
        M6_CAP_WHEEL_DIAG_ACTION) == M6_RESULT_OK);
    assert(M6Action_Validate(action, sizeof(action), 0U) ==
        M6_RESULT_UNSUPPORTED_CAPABILITY);

    memset(action, 0, sizeof(action));
    action[0] = M6_ACTION_KIND_HEADING_TURN;
    action[1] = 3U;
    put_u32(&action[8], 90000U);
    put_u32(&action[16], 3000U);
    put_u32(&action[20], 1U);
    assert(M6Action_Validate(action, sizeof(action), M6_CAP_HEADING_ACTION) ==
        M6_RESULT_OK);
    put_u32(&action[8], 45000U);
    assert(M6Action_Validate(action, sizeof(action), M6_CAP_HEADING_ACTION) ==
        M6_RESULT_UNSUPPORTED_CAPABILITY);
    assert(M6Action_Validate(action, sizeof(action),
        M6_CAP_HEADING_ACTION | M6_CAP_CONFIGURABLE_TURN) == M6_RESULT_OK);
    action[1] = 1U;
    assert(M6Action_Validate(action, sizeof(action),
        M6_CAP_HEADING_ACTION | M6_CAP_CONFIGURABLE_TURN) ==
        M6_RESULT_INVALID_ARGUMENT);
    return 0;
}
