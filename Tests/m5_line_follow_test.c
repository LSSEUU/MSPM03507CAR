#include "control_config.h"
#include "line_follow.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

static void qualify_channels(void)
{
    uint8_t low[LINE_SENSOR_CHANNEL_COUNT] = {0U};
    uint8_t high[LINE_SENSOR_CHANNEL_COUNT] = {
        1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U
    };
    uint32_t i;
    LineFollow_StartCalibration();
    for (i = 1U; i <= LINE_CALIBRATION_MIN_SAMPLES; i++) {
        LineFollow_ProcessSample((i & 1U) ? low : high, i, i * 20U);
    }
    assert(LineFollow_FinishCalibration());
}

static void make_active_high_line(uint8_t level[LINE_SENSOR_CHANNEL_COUNT],
                                  uint32_t first, uint32_t second)
{
    uint32_t i;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 0U;
    level[first] = 1U;
    level[second] = 1U;
}

static void test_qualification_position_and_active_level(void)
{
    uint8_t level[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowSnapshot_t snapshot;
    uint32_t i;
    LineFollow_Init(LINE_POLARITY_ACTIVE_HIGH);
    qualify_channels();
    make_active_high_line(level, 6U, 7U);
    LineFollow_ProcessSample(level, 21U, 420U);
    LineFollow_ProcessSample(level, 22U, 440U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.calibration.valid);
    assert(snapshot.calibration.valid_mask == 0xFFU);
    assert(snapshot.line_valid);
    assert(snapshot.position > 0.75f);
    assert(snapshot.confidence > 0.90f);

    LineFollow_SetPolarity(LINE_POLARITY_ACTIVE_LOW);
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 1U;
    level[0] = 0U;
    level[1] = 0U;
    LineFollow_ProcessSample(level, 23U, 460U);
    LineFollow_ProcessSample(level, 24U, 480U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.line_valid);
    assert(snapshot.position < -0.75f);
}

static void test_tracking_search_recovery_and_timeout(void)
{
    uint8_t level[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowSnapshot_t snapshot;
    uint32_t i;
    LineFollow_Init(LINE_POLARITY_ACTIVE_HIGH);
    qualify_channels();
    make_active_high_line(level, 5U, 6U);
    LineFollow_ProcessSample(level, 21U, 420U);
    LineFollow_ProcessSample(level, 22U, 440U);
    assert(LineFollow_Start(440U));
    LineFollow_Update(460U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(snapshot.left_target_mm_s > snapshot.right_target_mm_s);

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 0U;
    LineFollow_ProcessSample(level, 23U, 480U);
    LineFollow_ProcessSample(level, 24U, 500U);
    LineFollow_Update(500U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_SEARCHING);
    assert(snapshot.left_target_mm_s > snapshot.right_target_mm_s);

    make_active_high_line(level, 0U, 1U);
    LineFollow_ProcessSample(level, 25U, 520U);
    LineFollow_ProcessSample(level, 26U, 540U);
    LineFollow_Update(540U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(snapshot.position < -0.75f);

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 0U;
    LineFollow_ProcessSample(level, 27U, 560U);
    LineFollow_ProcessSample(level, 28U, 580U);
    LineFollow_Update(580U);
    LineFollow_Update(580U + LINE_LOST_TIMEOUT_MS);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_LOST_TIMEOUT);
    assert(fabsf(snapshot.left_target_mm_s) < 0.01f);
    assert(fabsf(snapshot.right_target_mm_s) < 0.01f);
}

static void test_rejects_incomplete_qualification_and_duplicate_scan(void)
{
    uint8_t level[LINE_SENSOR_CHANNEL_COUNT] = {0U};
    LineFollowSnapshot_t snapshot;
    uint32_t i;
    LineFollow_Init(LINE_POLARITY_ACTIVE_HIGH);
    LineFollow_StartCalibration();
    for (i = 1U; i <= LINE_CALIBRATION_MIN_SAMPLES; i++)
        LineFollow_ProcessSample(level, i, i * 20U);
    LineFollow_ProcessSample(level, LINE_CALIBRATION_MIN_SAMPLES, 999U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.calibration.sample_count == LINE_CALIBRATION_MIN_SAMPLES);
    assert(!LineFollow_FinishCalibration());
    assert(!LineFollow_Start(1000U));
}

int main(void)
{
    test_qualification_position_and_active_level();
    test_tracking_search_recovery_and_timeout();
    test_rejects_incomplete_qualification_and_duplicate_scan();
    return 0;
}
