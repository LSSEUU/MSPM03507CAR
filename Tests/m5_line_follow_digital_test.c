#include "line_follow.h"
#include "control_config.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

static uint32_t scan_id;

static void apply_safe_config(void)
{
    LineFollowConfig_t config;
    LineFollow_GetDefaultConfig(&config);
    config.speed_min_mm_s = 100.0f;
    config.speed_max_mm_s = 100.0f;
    config.position_alpha = 0.35f;
    config.derivative_alpha = 0.25f;
    config.kp = 150.0f;
    config.kd = 1.1f;
    config.correction_limit_mm_s = 150.0f;
    config.correction_slew_mm_s2 = 800.0f;
    config.curve_gain = 0.0f;
    config.lost_search_speed_mm_s = 105.0f;
    assert(LineFollow_ApplyConfig(&config));
}

static void initialize_at(uint32_t start_ms)
{
    LineFollow_Init();
    apply_safe_config();
    scan_id = 0U;
    LineFollow_ProcessDigitalMask(0x18U, ++scan_id, start_ms - 40U);
    LineFollow_ProcessDigitalMask(0x18U, ++scan_id, start_ms - 20U);
    LineFollow_ProcessDigitalMask(0x18U, ++scan_id, start_ms);
    assert(LineFollow_Start(start_ms));
}

static void assert_forward(const LineFollowSnapshot_t *snapshot)
{
    assert(snapshot->left_target_mm_s >= 0.0f);
    assert(snapshot->right_target_mm_s >= 0.0f);
}

static void test_transient_center_chatter_deadband(void)
{
    static const uint8_t centers[] = {0x08U, 0x10U, 0x18U};
    LineFollowSnapshot_t snapshot;
    uint32_t i;

    LineFollow_Init();
    apply_safe_config();
    scan_id = 0U;
    for (i = 0U; i < sizeof(centers); i++) {
        LineFollow_ProcessDigitalMask(centers[i], ++scan_id, 20U * (i + 1U));
        LineFollow_GetSnapshot(&snapshot);
        assert(fabsf(snapshot.raw_position) < 0.001f);
        assert(fabsf(snapshot.position) < 0.001f);
        assert(!snapshot.ambiguous);
    }
    assert(snapshot.line_valid);
}

static void test_persistent_single_center_steers_gently_and_sets_hint(void)
{
    LineFollowSnapshot_t snapshot;
    uint32_t i;

    initialize_at(60U);
    LineFollow_Update(80U);
    for (i = 0U; i < LINE_DIGITAL_CENTER_HINT_SAMPLES; i++) {
        uint32_t now_ms = 100U + (20U * i);
        LineFollow_ProcessDigitalMask(0x10U, ++scan_id, now_ms);
        LineFollow_Update(now_ms);
        LineFollow_GetSnapshot(&snapshot);
        if ((i + 1U) < LINE_DIGITAL_CENTER_CONTROL_SAMPLES) {
            assert(fabsf(snapshot.raw_position) < 0.001f);
        }
    }
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.raw_position > 0.05f);
    assert(snapshot.raw_position < 0.09f);
    assert(snapshot.correction_mm_s > 0.0f);
    assert(snapshot.left_target_mm_s > snapshot.right_target_mm_s);
    assert(snapshot.search_direction == 1);
    assert_forward(&snapshot);

    initialize_at(260U);
    LineFollow_Update(280U);
    for (i = 0U; i < LINE_DIGITAL_CENTER_HINT_SAMPLES; i++) {
        uint32_t now_ms = 300U + (20U * i);
        LineFollow_ProcessDigitalMask(0x08U, ++scan_id, now_ms);
        LineFollow_Update(now_ms);
    }
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.raw_position < -0.05f);
    assert(snapshot.raw_position > -0.09f);
    assert(snapshot.correction_mm_s < 0.0f);
    assert(snapshot.left_target_mm_s < snapshot.right_target_mm_s);
    assert(snapshot.search_direction == -1);
    assert_forward(&snapshot);
}

static void test_left_and_right_single_clusters(void)
{
    LineFollowSnapshot_t snapshot;

    LineFollow_Init();
    apply_safe_config();
    LineFollow_ProcessDigitalMask(0x04U, 1U, 20U);
    LineFollow_ProcessDigitalMask(0x04U, 2U, 40U);
    LineFollow_ProcessDigitalMask(0x04U, 3U, 60U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.line_valid);
    assert(snapshot.raw_position < 0.0f);
    assert(snapshot.position < 0.0f);

    LineFollow_Init();
    apply_safe_config();
    LineFollow_ProcessDigitalMask(0x20U, 1U, 20U);
    LineFollow_ProcessDigitalMask(0x20U, 2U, 40U);
    LineFollow_ProcessDigitalMask(0x20U, 3U, 60U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.line_valid);
    assert(snapshot.raw_position > 0.0f);
    assert(snapshot.position > 0.0f);
}

static void test_curve_gain_boost_preserves_center_and_adds_arc_authority(void)
{
    LineFollowConfig_t config;
    LineFollowSnapshot_t snapshot;
    uint32_t i;

    LineFollow_Init();
    LineFollow_GetDefaultConfig(&config);
    config.speed_min_mm_s = 160.0f;
    config.speed_max_mm_s = 160.0f;
    config.position_alpha = 1.0f;
    config.derivative_alpha = 1.0f;
    config.kp = 55.0f;
    config.kd = 0.0f;
    config.correction_limit_mm_s = 60.0f;
    config.correction_slew_mm_s2 = 2000.0f;
    assert(LineFollow_ApplyConfig(&config));
    scan_id = 0U;
    for (i = 1U; i <= LINE_DIGITAL_REACQUIRE_SAMPLES; i++)
        LineFollow_ProcessDigitalMask(0x18U, ++scan_id, 20U * i);
    assert(LineFollow_Start(60U));
    LineFollow_Update(80U);
    LineFollow_GetSnapshot(&snapshot);
    assert(fabsf(snapshot.raw_correction_mm_s) < 0.001f);

    /* A persistent single centre sensor keeps the configured Kp=55, but its
     * conditioned hint is half of the 1/7 geometric sensor position. */
    for (i = 0U; i < LINE_DIGITAL_CENTER_CONTROL_SAMPLES; i++) {
        uint32_t now_ms = 100U + (20U * i);
        LineFollow_ProcessDigitalMask(0x10U, ++scan_id, now_ms);
        LineFollow_Update(now_ms);
    }
    LineFollow_GetSnapshot(&snapshot);
    assert(fabsf(snapshot.raw_position - (0.5f / 7.0f)) < 0.001f);
    assert(fabsf(snapshot.raw_correction_mm_s - (27.5f / 7.0f)) < 0.01f);

    /* Sensor 6 is position=3/7.  Smooth curve boost raises the proportional
     * request from 23.6 mm/s to about 36.3 mm/s, matching the measured
     * differential-speed demand of the 0.5 m radius course arc. */
    LineFollow_ProcessDigitalMask(0x20U, ++scan_id, 160U);
    LineFollow_Update(160U);
    LineFollow_GetSnapshot(&snapshot);
    assert(fabsf(snapshot.raw_position - (3.0f / 7.0f)) < 0.001f);
    assert(snapshot.raw_correction_mm_s > 35.0f);
    assert(snapshot.raw_correction_mm_s < 38.0f);
    assert(snapshot.left_target_mm_s > snapshot.right_target_mm_s);
    assert_forward(&snapshot);
}

static void test_loss_recovery_and_timeout(void)
{
    LineFollowSnapshot_t snapshot;

    initialize_at(60U);
    LineFollow_Update(80U);
    LineFollow_ProcessDigitalMask(0x20U, ++scan_id, 100U);
    LineFollow_Update(100U);
    LineFollow_ProcessDigitalMask(0x20U, ++scan_id, 120U);
    LineFollow_Update(120U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.search_direction == 1);
    assert(snapshot.left_target_mm_s > snapshot.right_target_mm_s);
    assert_forward(&snapshot);

    LineFollow_ProcessDigitalMask(0x00U, ++scan_id, 140U);
    LineFollow_Update(140U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(snapshot.dynamic_base_mm_s <= 105.0f);
    assert_forward(&snapshot);

    LineFollow_Update(240U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert_forward(&snapshot);
    LineFollow_Update(260U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_SEARCHING);
    assert(snapshot.left_target_mm_s > snapshot.right_target_mm_s);
    assert(snapshot.right_target_mm_s >= 50.0f);

    LineFollow_Update(739U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_SEARCHING);
    assert_forward(&snapshot);
    LineFollow_Update(740U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_LOST_TIMEOUT);
    assert(snapshot.left_target_mm_s == 0.0f);
    assert(snapshot.right_target_mm_s == 0.0f);
}

static void test_ambiguous_bridge_and_reacquire(void)
{
    LineFollowSnapshot_t snapshot;

    initialize_at(60U);
    LineFollow_Update(80U);
    LineFollow_ProcessDigitalMask(0x20U, ++scan_id, 100U);
    LineFollow_Update(100U);
    LineFollow_ProcessDigitalMask(0x20U, ++scan_id, 120U);
    LineFollow_Update(120U);
    LineFollow_ProcessDigitalMask(0x81U, ++scan_id, 140U);
    LineFollow_Update(140U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.ambiguous);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert_forward(&snapshot);

    LineFollow_ProcessDigitalMask(0xFFU, ++scan_id, 160U);
    LineFollow_Update(380U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert_forward(&snapshot);
    LineFollow_Update(411U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_SEARCHING);
    assert_forward(&snapshot);

    LineFollow_ProcessDigitalMask(0x10U, ++scan_id, 420U);
    LineFollow_Update(420U);
    LineFollow_GetSnapshot(&snapshot);
    assert(!snapshot.line_valid);
    LineFollow_ProcessDigitalMask(0x10U, ++scan_id, 440U);
    LineFollow_Update(440U);
    LineFollow_GetSnapshot(&snapshot);
    assert(!snapshot.line_valid);
    LineFollow_ProcessDigitalMask(0x10U, ++scan_id, 460U);
    LineFollow_Update(460U);
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.line_valid);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
    assert(fabsf(snapshot.raw_derivative) < 0.001f);
    assert_forward(&snapshot);
}

static void test_failed_sequence_has_no_impulse_or_reverse(void)
{
    static const uint8_t sequence[] = {
        0x10U, 0x08U, 0x00U, 0x08U, 0x00U, 0x10U, 0x20U,
        0x20U, 0x20U
    };
    LineFollowSnapshot_t snapshot;
    float previous_correction = 0.0f;
    uint32_t time_ms = 80U;
    uint32_t i;

    initialize_at(60U);
    LineFollow_Update(60U);
    for (i = 0U; i < sizeof(sequence); i++, time_ms += 20U) {
        LineFollow_ProcessDigitalMask(sequence[i], ++scan_id, time_ms);
        LineFollow_Update(time_ms);
        LineFollow_GetSnapshot(&snapshot);
        assert_forward(&snapshot);
        assert(fabsf(snapshot.correction_mm_s - previous_correction) <= 16.01f);
        assert(fabsf(snapshot.raw_derivative) <= 4.001f);
        assert(fabsf(snapshot.filtered_derivative) <= 4.001f);
        previous_correction = snapshot.correction_mm_s;
    }
    assert(snapshot.line_valid);
    assert(snapshot.state == LINE_FOLLOW_TRACKING);
}

static void test_loss_timer_wrap(void)
{
    LineFollowSnapshot_t snapshot;
    uint32_t lost_ms = UINT32_MAX - 100U;

    initialize_at(lost_ms - 20U);
    LineFollow_Update(lost_ms - 20U);
    LineFollow_ProcessDigitalMask(0x00U, ++scan_id, lost_ms);
    LineFollow_Update(lost_ms);
    LineFollow_Update((uint32_t)(lost_ms + 599U));
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state != LINE_FOLLOW_LOST_TIMEOUT);
    LineFollow_Update((uint32_t)(lost_ms + 600U));
    LineFollow_GetSnapshot(&snapshot);
    assert(snapshot.state == LINE_FOLLOW_LOST_TIMEOUT);
}

int main(void)
{
    test_transient_center_chatter_deadband();
    test_persistent_single_center_steers_gently_and_sets_hint();
    test_left_and_right_single_clusters();
    test_curve_gain_boost_preserves_center_and_adds_arc_authority();
    test_loss_recovery_and_timeout();
    test_ambiguous_bridge_and_reacquire();
    test_failed_sequence_has_no_impulse_or_reverse();
    test_loss_timer_wrap();
    return 0;
}
