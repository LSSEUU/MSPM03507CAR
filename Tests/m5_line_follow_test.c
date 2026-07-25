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

int main(void)
{
    test_calibration_hysteresis_and_position();
    test_tracking_search_recovery_and_timeout();
    test_rejects_missing_line_and_duplicate_scan();
    return 0;
}
