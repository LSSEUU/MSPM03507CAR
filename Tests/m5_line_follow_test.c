#include "control_config.h"
#include "line_follow.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

static void make_active_high_line(uint8_t level[LINE_SENSOR_CHANNEL_COUNT],
                                  uint32_t first, uint32_t second)
{
    uint32_t i;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 0U;
    level[first] = 1U;
    level[second] = 1U;
}

static void test_fixed_digital_position_and_active_level(void)
{
    uint8_t level[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowSnapshot_t snapshot;
    uint32_t i;
    LineFollow_Init(LINE_POLARITY_ACTIVE_HIGH);
    make_active_high_line(level, 6U, 7U);
    LineFollow_ProcessSample(level, 1U, 20U);
    LineFollow_ProcessSample(level, 2U, 40U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.line_valid);
    assert(snapshot.position > 0.75f);
    assert(snapshot.confidence > 0.90f);

    LineFollow_SetPolarity(LINE_POLARITY_ACTIVE_LOW);
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 1U;
    level[0] = 0U;
    level[1] = 0U;
    LineFollow_ProcessSample(level, 3U, 60U);
    LineFollow_ProcessSample(level, 4U, 80U);
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
    make_active_high_line(level, 5U, 6U);
    LineFollow_ProcessSample(level, 1U, 20U);
    LineFollow_ProcessSample(level, 2U, 40U);
    assert(LineFollow_Start(40U));
    LineFollow_Update(60U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(snapshot.left_target_mm_s > snapshot.right_target_mm_s);

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 0U;
    LineFollow_ProcessSample(level, 3U, 80U);
    LineFollow_ProcessSample(level, 4U, 100U);
    LineFollow_Update(100U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_SEARCHING);
    assert(snapshot.left_target_mm_s > snapshot.right_target_mm_s);

    make_active_high_line(level, 0U, 1U);
    LineFollow_ProcessSample(level, 5U, 120U);
    LineFollow_ProcessSample(level, 6U, 140U);
    LineFollow_Update(140U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(snapshot.position < -0.75f);

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 0U;
    LineFollow_ProcessSample(level, 7U, 160U);
    LineFollow_ProcessSample(level, 8U, 180U);
    LineFollow_Update(180U);
    LineFollow_Update(180U + LINE_LOST_TIMEOUT_MS);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_LOST_TIMEOUT);
    assert(fabsf(snapshot.left_target_mm_s) < 0.01f);
    assert(fabsf(snapshot.right_target_mm_s) < 0.01f);
}

static void test_rejects_missing_line_and_duplicate_scan(void)
{
    uint8_t level[LINE_SENSOR_CHANNEL_COUNT] = {0U};
    LineFollowSnapshot_t snapshot;
    LineFollow_Init(LINE_POLARITY_ACTIVE_HIGH);
    LineFollow_ProcessSample(level, 1U, 20U);
    LineFollow_ProcessSample(level, 1U, 999U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.update_count == 1U);
    assert(!snapshot.line_valid);
    assert(!LineFollow_Start(1000U));
}

int main(void)
{
    test_fixed_digital_position_and_active_level();
    test_tracking_search_recovery_and_timeout();
    test_rejects_missing_line_and_duplicate_scan();
    return 0;
}
