#include "control_config.h"
#include "heading.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

static int close_to(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void calibrate_at(float yaw)
{
    uint32_t frame;
    Heading_Init();
    for (frame = 1U; frame <= HEADING_CALIBRATION_SAMPLES; frame++)
        Heading_Update(yaw, frame, frame * 20U);
}

static void test_normalization(void)
{
    assert(close_to(Heading_NormalizeDegrees(180.0f), -180.0f, 0.01f));
    assert(close_to(Heading_NormalizeDegrees(540.0f), -180.0f, 0.01f));
    assert(close_to(Heading_NormalizeDegrees(-181.0f), 179.0f, 0.01f));
    assert(close_to(Heading_NormalizeDegrees(725.0f), 5.0f, 0.01f));
}

static void test_wrap_safe_calibration_and_duplicate_filter(void)
{
    HeadingSnapshot_t snapshot;
    uint32_t frame;
    Heading_Init();
    Heading_Update(179.75f, 1U, 20U);
    Heading_Update(-179.75f, 1U, 21U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.calibration_count == 1U);
    for (frame = 2U; frame <= HEADING_CALIBRATION_SAMPLES; frame++)
        Heading_Update((frame & 1U) ? 179.75f : -179.75f,
                       frame, frame * 20U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.calibrated);
    assert(!snapshot.calibrating);
    assert(close_to(fabsf(snapshot.zero_offset_deg), 180.0f, 1.1f));
    assert(fabsf(snapshot.yaw_deg) <= 1.1f);
}

static void test_heading_hold_correction(void)
{
    HeadingSnapshot_t snapshot;
    calibrate_at(0.0f);
    assert(Heading_StartHold(250.0f));
    Heading_Update(10.0f, HEADING_CALIBRATION_SAMPLES + 1U, 500U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.mode == HEADING_MODE_HOLD);
    assert(close_to(snapshot.error_deg, -10.0f, 0.01f));
    assert(close_to(snapshot.yaw_rate_deg_s, 35.0f, 0.01f));
    assert(close_to(snapshot.left_target_mm_s, 308.0f, 0.01f));
    assert(close_to(snapshot.right_target_mm_s, 192.0f, 0.01f));
}

static void test_nonblocking_turn_and_wrap(void)
{
    HeadingSnapshot_t snapshot;
    uint32_t frame = HEADING_CALIBRATION_SAMPLES;
    calibrate_at(0.0f);
    assert(Heading_StartTurn(90.0f));
    Heading_GetSnapshot(&snapshot);
    assert(close_to(snapshot.left_target_mm_s,
                    -HEADING_TURN_MAX_SPEED_MM_S, 0.01f));
    assert(close_to(snapshot.right_target_mm_s,
                    HEADING_TURN_MAX_SPEED_MM_S, 0.01f));
    Heading_Update(89.0f, ++frame, 500U);
    Heading_Update(90.0f, ++frame, 520U);
    Heading_GetSnapshot(&snapshot);
    assert(!snapshot.complete);
    Heading_Update(91.0f, ++frame, 540U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.complete);
    assert(fabsf(snapshot.error_deg) <= HEADING_TURN_TOLERANCE_DEG);

    Heading_Stop();
    Heading_Update(170.0f, ++frame, 560U);
    assert(Heading_StartTurn(30.0f));
    Heading_GetSnapshot(&snapshot);
    assert(close_to(snapshot.target_deg, -160.0f, 0.01f));
    assert(close_to(snapshot.error_deg, 30.0f, 0.01f));
}

static void test_turn_rate_damping(void)
{
    HeadingSnapshot_t snapshot;
    calibrate_at(0.0f);
    assert(Heading_StartTurn(90.0f));
    Heading_Update(5.0f, HEADING_CALIBRATION_SAMPLES + 1U, 420U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.yaw_rate_deg_s > 80.0f);
    assert(snapshot.yaw_rate_deg_s < 95.0f);
    assert(snapshot.left_target_mm_s < 0.0f);
    assert(fabsf(snapshot.left_target_mm_s) <
           HEADING_TURN_MAX_SPEED_MM_S);
}

static void test_configurable_turns_cross_wrap_continuously(void)
{
    HeadingSnapshot_t snapshot;
    uint32_t frame = HEADING_CALIBRATION_SAMPLES;
    calibrate_at(0.0f);
    assert(Heading_StartTurn(360.0f));
    Heading_Update(90.0f, ++frame, 500U);
    Heading_Update(179.0f, ++frame, 520U);
    Heading_Update(-91.0f, ++frame, 540U);
    Heading_Update(0.0f, ++frame, 560U);
    Heading_Update(0.0f, ++frame, 580U);
    Heading_Update(0.0f, ++frame, 600U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.complete);
    assert(close_to(snapshot.error_deg, 0.0f, 0.01f));

    Heading_Stop();
    assert(Heading_StartTurn(-210.0f));
    Heading_Update(-90.0f, ++frame, 620U);
    Heading_Update(-179.0f, ++frame, 640U);
    Heading_Update(150.0f, ++frame, 660U);
    Heading_Update(150.0f, ++frame, 680U);
    Heading_Update(150.0f, ++frame, 700U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.complete);
    assert(close_to(snapshot.target_deg, 150.0f, 0.01f));
    assert(close_to(snapshot.error_deg, 0.0f, 0.01f));
}

static void test_direct_rate_preferred_then_stale_fallback(void)
{
    HeadingSnapshot_t snapshot;
    calibrate_at(0.0f);
    assert(Heading_StartHold(200.0f));
    Heading_UpdateWithRate(1.0f, HEADING_CALIBRATION_SAMPLES + 1U,
                           50.0f, 1U, 420U, 420U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.rate_source == HEADING_RATE_GYRO_FRAME);
    assert(close_to(snapshot.yaw_rate_deg_s, 17.5f, 0.01f));

    Heading_UpdateWithRate(2.0f, HEADING_CALIBRATION_SAMPLES + 2U,
                           50.0f, 1U, 420U, 540U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.rate_source == HEADING_RATE_ANGLE_DIFFERENCE);
    assert(close_to(snapshot.raw_yaw_rate_deg_s, 8.3333f, 0.01f));
}

static void test_calibration_dispersion_gate(void)
{
    HeadingSnapshot_t snapshot;
    uint32_t frame;
    Heading_Init();
    for (frame = 1U; frame <= HEADING_CALIBRATION_SAMPLES; frame++)
        Heading_Update((frame & 1U) ? -0.5f : 0.5f,
                       frame, frame * 20U);
    Heading_GetSnapshot(&snapshot);
    assert(!snapshot.calibrated);
    assert(snapshot.calibration_rejects == 1U);
    assert(!snapshot.calibration_stationary);

    for (frame = HEADING_CALIBRATION_SAMPLES + 1U;
         frame <= (2U * HEADING_CALIBRATION_SAMPLES); frame++)
        Heading_Update((frame & 1U) ? 0.0f : 2.0f,
                       frame, frame * 20U);
    Heading_GetSnapshot(&snapshot);
    assert(!snapshot.calibrated);
    assert(snapshot.calibration_rejects == 2U);
    assert(!snapshot.calibration_stationary);

    for (frame = (2U * HEADING_CALIBRATION_SAMPLES) + 1U;
         frame <= (3U * HEADING_CALIBRATION_SAMPLES); frame++)
        Heading_Update(3.0f, frame, frame * 20U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.calibrated);
    assert(snapshot.calibration_stationary);
}

static void test_calibration_rate_gate(void)
{
    HeadingSnapshot_t snapshot;
    Heading_Init();
    Heading_UpdateWithRate(0.0f, 1U, 3.1f, 1U, 20U, 20U);
    Heading_GetSnapshot(&snapshot);
    assert(!snapshot.calibrated);
    assert(snapshot.calibration_count == 0U);
    assert(snapshot.calibration_rejects == 1U);
    assert(!snapshot.calibration_stationary);
}

static void test_turn_smooth_zone_has_no_minimum_speed_step(void)
{
    HeadingConfig_t config;
    HeadingSnapshot_t snapshot;
    calibrate_at(0.0f);
    Heading_GetDefaultConfig(&config);
    config.smooth_zone_deg = 20.0f;
    assert(Heading_ApplyConfig(&config));
    assert(Heading_StartTurn(90.0f));
    Heading_UpdateWithRate(87.0f, HEADING_CALIBRATION_SAMPLES + 1U,
                           0.0f, 1U, 420U, 420U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.left_target_mm_s < 0.0f);
    assert(close_to(snapshot.left_target_mm_s, -1.33745f, 0.01f));
    assert(close_to(snapshot.right_target_mm_s, 1.33745f, 0.01f));
}

static void test_heading_rate_control(void)
{
    HeadingSnapshot_t snapshot;
    calibrate_at(0.0f);
    assert(Heading_StartRate(90.0f));
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.mode == HEADING_MODE_RATE);
    assert(close_to(snapshot.target_rate_deg_s, 90.0f, 0.01f));
    assert(close_to(snapshot.rate_error_deg_s, 90.0f, 0.01f));
    assert(close_to(snapshot.left_target_mm_s, -49.5f, 0.01f));
    assert(close_to(snapshot.right_target_mm_s, 49.5f, 0.01f));

    Heading_UpdateWithRate(1.0f, HEADING_CALIBRATION_SAMPLES + 1U,
                           60.0f, 1U, 420U, 420U);
    Heading_GetSnapshot(&snapshot);
    assert(close_to(snapshot.yaw_rate_deg_s, 21.0f, 0.01f));
    assert(close_to(snapshot.rate_error_deg_s, 69.0f, 0.01f));
    assert(close_to(snapshot.left_target_mm_s, -49.5f, 0.01f));
    assert(close_to(snapshot.right_target_mm_s, 49.5f, 0.01f));

    Heading_UpdateWithRate(2.0f, HEADING_CALIBRATION_SAMPLES + 2U,
                           60.0f, 2U, 440U, 440U);
    Heading_GetSnapshot(&snapshot);
    assert(close_to(snapshot.yaw_rate_deg_s, 34.65f, 0.01f));
    assert(close_to(snapshot.rate_error_deg_s, 55.35f, 0.01f));
    assert(close_to(snapshot.left_target_mm_s, -51.714f, 0.01f));
    assert(close_to(snapshot.right_target_mm_s, 51.714f, 0.01f));

    Heading_Stop();
    assert(Heading_StartRate(-90.0f));
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.left_target_mm_s > 0.0f);
    assert(snapshot.right_target_mm_s < 0.0f);
    assert(!Heading_StartRate(0.0f));
    assert(!Heading_StartRate(361.0f));
}

int main(void)
{
    test_normalization();
    test_wrap_safe_calibration_and_duplicate_filter();
    test_heading_hold_correction();
    test_nonblocking_turn_and_wrap();
    test_turn_rate_damping();
    test_configurable_turns_cross_wrap_continuously();
    test_direct_rate_preferred_then_stale_fallback();
    test_calibration_dispersion_gate();
    test_calibration_rate_gate();
    test_turn_smooth_zone_has_no_minimum_speed_step();
    test_heading_rate_control();
    return 0;
}
