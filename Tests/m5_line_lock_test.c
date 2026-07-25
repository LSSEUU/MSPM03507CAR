#include "line_follow.h"

#include <assert.h>
#include <stdint.h>

int main(void)
{
    uint16_t adc[LINE_SENSOR_CHANNEL_COUNT] = {0U};
    LineFollowSnapshot_t snapshot;

    LineFollow_Init();
    LineFollow_ProcessSample(adc, 1U, 20U);
    LineFollow_GetSnapshot(&snapshot);
    assert(!snapshot.calibration.compiled_verified);
    assert(!snapshot.calibration.valid);
    assert(!snapshot.line_valid);
    assert(!LineFollow_Start(20U));
    return 0;
}
