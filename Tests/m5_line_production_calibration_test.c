#include "line_follow.h"

#include <assert.h>
#include <stdint.h>

static const uint16_t expected_white[LINE_SENSOR_CHANNEL_COUNT] = {
    4094U, 4094U, 4094U, 4094U, 4094U, 4094U, 4094U, 4094U
};
static const uint16_t expected_black[LINE_SENSOR_CHANNEL_COUNT] = {
    1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U
};

int main(void)
{
    LineFollowSnapshot_t snapshot;
    uint16_t adc[LINE_SENSOR_CHANNEL_COUNT];
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
        adc[i] = expected_white[i];
    }

    adc[3] = expected_black[3];
    adc[4] = expected_black[4];
    LineFollow_ProcessSample(adc, 1U, 20U);
    LineFollow_ProcessSample(adc, 2U, 40U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.active_mask == 0x18U);
    assert(snapshot.line_valid);
    assert(snapshot.position > -0.01f && snapshot.position < 0.01f);

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++)
        adc[i] = (i < 2U) ? expected_black[i] : expected_white[i];
    LineFollow_ProcessSample(adc, 3U, 60U);
    LineFollow_ProcessSample(adc, 4U, 80U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.active_mask == 0x03U);
    assert(snapshot.line_valid && snapshot.position < -0.75f);

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++)
        adc[i] = (i >= 6U) ? expected_black[i] : expected_white[i];
    LineFollow_ProcessSample(adc, 5U, 100U);
    LineFollow_ProcessSample(adc, 6U, 120U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.active_mask == 0xC0U);
    assert(snapshot.line_valid && snapshot.position > 0.75f);

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++)
        adc[i] = expected_black[i];
    LineFollow_ProcessSample(adc, 7U, 140U);
    LineFollow_ProcessSample(adc, 8U, 160U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.active_mask == 0xFFU);
    assert(snapshot.ambiguous && !snapshot.line_valid);
    return 0;
}
