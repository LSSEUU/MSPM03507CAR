#include "line_follow.h"

#include <assert.h>
#include <stdint.h>

static const uint16_t expected_white[LINE_SENSOR_CHANNEL_COUNT] = {
    2804U, 2548U, 3026U, 2709U, 1112U, 3063U, 3071U, 2253U
};
static const uint16_t expected_black[LINE_SENSOR_CHANNEL_COUNT] = {
    102U, 106U, 108U, 109U, 109U, 106U, 282U, 109U
};

int main(void)
{
    LineFollowSnapshot_t snapshot;
    uint32_t i;

    LineFollow_Init();
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.calibration.compiled_verified);
    assert(snapshot.calibration.valid);
    assert(snapshot.calibration.valid_mask == 0xFFU);
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        uint16_t enter = (uint16_t)(
            ((2U * (uint32_t)expected_black[i]) + expected_white[i]) / 3U);
        uint16_t exit = (uint16_t)(
            ((uint32_t)expected_black[i] +
             (2U * (uint32_t)expected_white[i])) / 3U);
        assert(snapshot.calibration.white_adc[i] == expected_white[i]);
        assert(snapshot.calibration.black_adc[i] == expected_black[i]);
        assert(snapshot.calibration.enter_black_adc[i] == enter);
        assert(snapshot.calibration.exit_black_adc[i] == exit);
    }
    return 0;
}
