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

static void test_topology_marks_full_black_and_multiple_clusters_ambiguous(void)
{
    uint8_t level[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowSnapshot_t snapshot;
    uint32_t i;
    LineFollow_Init(LINE_POLARITY_ACTIVE_HIGH);
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 1U;
    LineFollow_ProcessSample(level, 1U, 20U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.active_mask == 0xFFU);
    assert(snapshot.active_count == 8U);
    assert(snapshot.cluster_count == 1U);
    assert(snapshot.ambiguous);
    assert(!snapshot.line_valid);

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 0U;
    level[1] = 1U;
    level[6] = 1U;
    LineFollow_ProcessSample(level, 2U, 40U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.active_mask == 0x42U);
    assert(snapshot.active_count == 2U);
    assert(snapshot.cluster_count == 2U);
    assert(snapshot.ambiguous);
    assert(snapshot.ambiguity_count == 2U);
}

static void test_filtered_scheduled_control_slew_and_reacquire_reset(void)
{
    uint8_t level[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowConfig_t config;
    LineFollowSnapshot_t snapshot;
    uint32_t i;
    LineFollow_Init(LINE_POLARITY_ACTIVE_HIGH);
    LineFollow_GetDefaultConfig(&config);
    config.speed_min_mm_s = 100.0f;
    config.speed_max_mm_s = 220.0f;
    config.position_alpha = 0.5f;
    config.derivative_alpha = 0.5f;
    config.correction_slew_mm_s2 = 100.0f;
    config.curve_gain = 1.0f;
    assert(LineFollow_ApplyConfig(&config));
    make_active_high_line(level, 3U, 4U);
    LineFollow_ProcessSample(level, 1U, 20U);
    LineFollow_ProcessSample(level, 2U, 40U);
    assert(LineFollow_Start(40U));
    LineFollow_Update(60U);

    make_active_high_line(level, 6U, 7U);
    LineFollow_ProcessSample(level, 3U, 80U);
    LineFollow_Update(80U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.raw_position > snapshot.position);
    assert(snapshot.filtered_derivative > 0.0f);
    assert(snapshot.correction_mm_s <= 2.01f);
    assert(snapshot.dynamic_base_mm_s < config.speed_max_mm_s);
    assert(snapshot.left_target_mm_s >= 50.0f);
    assert(snapshot.right_target_mm_s >= 50.0f);

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 0U;
    LineFollow_ProcessSample(level, 4U, 100U);
    LineFollow_ProcessSample(level, 5U, 120U);
    LineFollow_Update(120U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state != LINE_FOLLOW_LOST_TIMEOUT);
    make_active_high_line(level, 0U, 1U);
    LineFollow_ProcessSample(level, 6U, 140U);
    LineFollow_ProcessSample(level, 7U, 160U);
    LineFollow_Update(160U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(fabsf(snapshot.filtered_derivative) < 0.01f);
}

static void test_short_ambiguity_slows_then_enters_search(void)
{
    uint8_t level[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowConfig_t config;
    LineFollowSnapshot_t snapshot;
    uint32_t i;
    LineFollow_Init(LINE_POLARITY_ACTIVE_HIGH);
    LineFollow_GetDefaultConfig(&config);
    config.speed_min_mm_s = 100.0f;
    config.speed_max_mm_s = 220.0f;
    config.curve_gain = 1.0f;
    assert(LineFollow_ApplyConfig(&config));
    make_active_high_line(level, 5U, 6U);
    LineFollow_ProcessSample(level, 1U, 20U);
    LineFollow_ProcessSample(level, 2U, 40U);
    assert(LineFollow_Start(40U));
    LineFollow_Update(60U);

    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) level[i] = 0U;
    level[1] = 1U;
    level[6] = 1U;
    LineFollow_ProcessSample(level, 3U, 80U);
    LineFollow_Update(80U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.line_valid);
    assert(snapshot.ambiguous);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(fabsf(snapshot.dynamic_base_mm_s - 100.0f) < 0.01f);

    LineFollow_ProcessSample(level, 4U, 100U);
    LineFollow_Update(100U);
    LineFollow_GetSnapshot(&snapshot);
    assert(!snapshot.line_valid);
    assert(snapshot.state == LINE_FOLLOW_SEARCHING);
}

int main(void)
{
    test_fixed_digital_position_and_active_level();
    test_tracking_search_recovery_and_timeout();
    test_rejects_missing_line_and_duplicate_scan();
    test_topology_marks_full_black_and_multiple_clusters_ambiguous();
    test_filtered_scheduled_control_slew_and_reacquire_reset();
    test_short_ambiguity_slows_then_enters_search();
    return 0;
}
