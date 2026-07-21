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
    Heading_Update(179.0f, 1U, 20U);
    Heading_Update(-179.0f, 1U, 21U);
    Heading_GetSnapshot(&snapshot);
    assert(snapshot.calibration_count == 1U);
    for (frame = 2U; frame <= HEADING_CALIBRATION_SAMPLES; frame++)
        Heading_Update((frame & 1U) ? 179.0f : -179.0f,
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

int main(void)
{
    test_normalization();
    test_wrap_safe_calibration_and_duplicate_filter();
    test_heading_hold_correction();
    test_nonblocking_turn_and_wrap();
    test_turn_rate_damping();
    return 0;
}
