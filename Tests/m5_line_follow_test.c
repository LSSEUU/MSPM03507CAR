#include "control_config.h"
#include "line_follow.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#define TEST_WHITE_ADC 3000U
#define TEST_BLACK_ADC  800U
#define TEST_ENTER_ADC ((2U * TEST_BLACK_ADC + TEST_WHITE_ADC) / 3U)
#define TEST_EXIT_ADC  ((TEST_BLACK_ADC + 2U * TEST_WHITE_ADC) / 3U)

static void fill_background(uint16_t adc[LINE_SENSOR_CHANNEL_COUNT])
{
    uint32_t i;
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) adc[i] = TEST_WHITE_ADC;
}

static void make_black_line(uint16_t adc[LINE_SENSOR_CHANNEL_COUNT],
                            uint32_t first, uint32_t second)
{
    fill_background(adc);
    adc[first] = TEST_BLACK_ADC;
    adc[second] = TEST_BLACK_ADC;
}

static void make_black_mask(uint16_t adc[LINE_SENSOR_CHANNEL_COUNT],
                            uint8_t mask)
{
    uint32_t i;
    fill_background(adc);
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        if (((mask >> i) & 1U) != 0U) adc[i] = TEST_BLACK_ADC;
    }
}

static void test_calibration_hysteresis_and_position(void)
{
    uint16_t adc[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowSnapshot_t snapshot;

    LineFollow_Init();
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.calibration.compiled_verified);
    assert(snapshot.calibration.valid);
    assert(snapshot.calibration.valid_mask == 0xFFU);
    assert(snapshot.calibration.enter_black_adc[0] == TEST_ENTER_ADC);
    assert(snapshot.calibration.exit_black_adc[0] == TEST_EXIT_ADC);

    fill_background(adc);
    adc[6] = TEST_ENTER_ADC + 1U;
    LineFollow_ProcessSample(adc, 1U, 20U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.level[6] == 0U);

    adc[6] = TEST_ENTER_ADC;
    adc[7] = TEST_BLACK_ADC;
    LineFollow_ProcessSample(adc, 2U, 40U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.level[6] == 1U);
    assert(snapshot.level[7] == 1U);
    assert(!snapshot.line_valid);

    adc[6] = (uint16_t)((TEST_ENTER_ADC + TEST_EXIT_ADC) / 2U);
    LineFollow_ProcessSample(adc, 3U, 60U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.level[6] == 1U);
    assert(snapshot.line_valid);
    assert(snapshot.position > 0.75f);

    adc[6] = TEST_EXIT_ADC - 1U;
    LineFollow_ProcessSample(adc, 4U, 80U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.level[6] == 1U);

    adc[6] = TEST_EXIT_ADC;
    adc[7] = TEST_EXIT_ADC;
    LineFollow_ProcessSample(adc, 5U, 100U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.level[6] == 0U);
    assert(snapshot.level[7] == 0U);
}

static void test_tracking_search_recovery_and_timeout(void)
{
    uint16_t adc[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowSnapshot_t snapshot;

    LineFollow_Init();
    make_black_line(adc, 5U, 6U);
    LineFollow_ProcessSample(adc, 1U, 20U);
    LineFollow_ProcessSample(adc, 2U, 40U);
    assert(LineFollow_Start(40U));
    LineFollow_Update(60U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(snapshot.left_target_mm_s > snapshot.right_target_mm_s);

    fill_background(adc);
    LineFollow_ProcessSample(adc, 3U, 80U);
    LineFollow_ProcessSample(adc, 4U, 100U);
    LineFollow_Update(100U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_SEARCHING);
    assert(snapshot.left_target_mm_s > snapshot.right_target_mm_s);

    make_black_line(adc, 0U, 1U);
    LineFollow_ProcessSample(adc, 5U, 120U);
    LineFollow_ProcessSample(adc, 6U, 140U);
    LineFollow_Update(140U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(snapshot.position < -0.75f);

    fill_background(adc);
    LineFollow_ProcessSample(adc, 7U, 160U);
    LineFollow_ProcessSample(adc, 8U, 180U);
    LineFollow_Update(180U);
    LineFollow_Update(180U + LINE_LOST_TIMEOUT_MS);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_LOST_TIMEOUT);
    assert(fabsf(snapshot.left_target_mm_s) < 0.01f);
    assert(fabsf(snapshot.right_target_mm_s) < 0.01f);
}

static void test_rejects_missing_line_and_duplicate_scan(void)
{
    uint16_t adc[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowSnapshot_t snapshot;
    fill_background(adc);
    LineFollow_Init();
    LineFollow_ProcessSample(adc, 1U, 20U);
    LineFollow_ProcessSample(adc, 1U, 999U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.update_count == 1U);
    assert(!snapshot.line_valid);
    assert(!LineFollow_Start(1000U));
}

static void test_topology_marks_full_black_and_multiple_clusters_ambiguous(void)
{
    uint16_t adc[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowSnapshot_t snapshot;
    LineFollow_Init();
    make_black_mask(adc, 0xFFU);
    LineFollow_ProcessSample(adc, 1U, 20U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.active_mask == 0xFFU);
    assert(snapshot.active_count == 8U);
    assert(snapshot.cluster_count == 1U);
    assert(snapshot.ambiguous);
    assert(!snapshot.line_valid);

    make_black_mask(adc, 0x42U);
    LineFollow_ProcessSample(adc, 2U, 40U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.active_mask == 0x42U);
    assert(snapshot.active_count == 2U);
    assert(snapshot.cluster_count == 2U);
    assert(snapshot.ambiguous);
    assert(snapshot.ambiguity_count == 2U);
}

static void test_filtered_scheduled_control_slew_and_reacquire_reset(void)
{
    uint16_t adc[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowConfig_t config;
    LineFollowSnapshot_t snapshot;
    LineFollow_Init();
    LineFollow_GetDefaultConfig(&config);
    config.speed_min_mm_s = 100.0f;
    config.speed_max_mm_s = 220.0f;
    config.position_alpha = 0.5f;
    config.derivative_alpha = 0.5f;
    config.correction_slew_mm_s2 = 100.0f;
    config.curve_gain = 1.0f;
    assert(LineFollow_ApplyConfig(&config));
    make_black_line(adc, 3U, 4U);
    LineFollow_ProcessSample(adc, 1U, 20U);
    LineFollow_ProcessSample(adc, 2U, 40U);
    assert(LineFollow_Start(40U));
    LineFollow_Update(60U);

    make_black_line(adc, 6U, 7U);
    LineFollow_ProcessSample(adc, 3U, 80U);
    LineFollow_Update(80U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.raw_position > snapshot.position);
    assert(snapshot.filtered_derivative > 0.0f);
    assert(snapshot.correction_mm_s <= 2.01f);
    assert(snapshot.dynamic_base_mm_s < config.speed_max_mm_s);
    assert(snapshot.left_target_mm_s >= 50.0f);
    assert(snapshot.right_target_mm_s >= 50.0f);

    fill_background(adc);
    LineFollow_ProcessSample(adc, 4U, 100U);
    LineFollow_ProcessSample(adc, 5U, 120U);
    LineFollow_Update(120U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state != LINE_FOLLOW_LOST_TIMEOUT);
    make_black_line(adc, 0U, 1U);
    LineFollow_ProcessSample(adc, 6U, 140U);
    LineFollow_ProcessSample(adc, 7U, 160U);
    LineFollow_Update(160U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(fabsf(snapshot.filtered_derivative) < 0.01f);
}

static void test_short_ambiguity_slows_then_enters_search(void)
{
    uint16_t adc[LINE_SENSOR_CHANNEL_COUNT];
    LineFollowConfig_t config;
    LineFollowSnapshot_t snapshot;
    LineFollow_Init();
    LineFollow_GetDefaultConfig(&config);
    config.speed_min_mm_s = 100.0f;
    config.speed_max_mm_s = 220.0f;
    config.curve_gain = 1.0f;
    assert(LineFollow_ApplyConfig(&config));
    make_black_line(adc, 5U, 6U);
    LineFollow_ProcessSample(adc, 1U, 20U);
    LineFollow_ProcessSample(adc, 2U, 40U);
    assert(LineFollow_Start(40U));
    LineFollow_Update(60U);

    make_black_mask(adc, 0x42U);
    LineFollow_ProcessSample(adc, 3U, 80U);
    LineFollow_Update(80U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.line_valid);
    assert(snapshot.ambiguous);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(fabsf(snapshot.dynamic_base_mm_s - 100.0f) < 0.01f);

    LineFollow_ProcessSample(adc, 4U, 100U);
    LineFollow_Update(100U);
    LineFollow_GetSnapshot(&snapshot);
    assert(!snapshot.line_valid);
    assert(snapshot.state == LINE_FOLLOW_SEARCHING);
}

int main(void)
{
    test_calibration_hysteresis_and_position();
    test_tracking_search_recovery_and_timeout();
    test_rejects_missing_line_and_duplicate_scan();
    test_topology_marks_full_black_and_multiple_clusters_ambiguous();
    test_filtered_scheduled_control_slew_and_reacquire_reset();
    test_short_ambiguity_slows_then_enters_search();
    return 0;
}
