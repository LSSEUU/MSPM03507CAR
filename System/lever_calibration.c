#include "lever_calibration.h"

#include "parameter_store.h"
#include "safety.h"
#include "system_state.h"
#include "zdt_stepper.h"
#include "zdt_stepper_test.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define LEVER_CAL_WAIT_TIMEOUT_MS       120000UL
#define LEVER_CAL_CAPTURE_TIMEOUT_MS    2000UL
#define LEVER_CAL_STATUS_FRESH_MS       500UL
#define LEVER_CAL_CONFIG_TO_ENCODER_GAP_MS 10UL
#define LEVER_CAL_ENCODER_RETRY_MS        50UL
#define LEVER_CAL_REFERENCE_SPREAD_MAX  32U
#define LEVER_CAL_POINT_SPREAD_MAX      16U
#define LEVER_CAL_REPEATABILITY_MAX     200U
#define LEVER_CAL_BACKLASH_MAX          300U
#define LEVER_CAL_HORIZONTAL_CHECK_MAX  32L
#define LEVER_CAL_ANGLE_ABS_MAX_MDEG    30000L

typedef enum {
    CAPTURE_NONE = 0,
    CAPTURE_REFERENCE,
    CAPTURE_HORIZONTAL,
    CAPTURE_POINT
} CaptureKind_t;

typedef enum {
    CAPTURE_WAIT_STATUS = 0,
    CAPTURE_WAIT_CONFIG,
    CAPTURE_WAIT_ENCODER
} CapturePhase_t;

static LeverCalibrationStatus_t s_status;
static LeverCalibrationValues_t s_persisted;
static bool s_persisted_valid;
static uint16_t s_session_reference;
static int32_t s_session_horizontal;
static int32_t s_angles[LEVER_CALIBRATION_POINT_COUNT];
static int16_t s_captures[2][LEVER_CALIBRATION_REPEAT_COUNT]
                         [LEVER_CALIBRATION_POINT_COUNT];
static uint16_t s_samples[LEVER_CALIBRATION_POINT_SAMPLES];
static uint32_t s_deadline_ms;
static uint32_t s_capture_status_ms;
static uint32_t s_encoder_request_ms;
static uint32_t s_seen_motor_sequence;
static uint32_t s_seen_config_sequence;
static uint32_t s_seen_encoder_sequence;
static uint32_t s_candidate_fingerprint;
static int32_t s_requested_angle_mdeg;
static uint16_t s_reference_spread;
static uint16_t s_max_point_spread;
static uint8_t s_sample_required;
static CaptureKind_t s_capture_kind;
static CapturePhase_t s_capture_phase;
static bool s_capture_request_sent;

static uint32_t abs_i32(int32_t value)
{
    if (value >= 0) return (uint32_t)value;
    return (uint32_t)(-(value + 1)) + 1U;
}

static bool reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static int32_t divide_round(int64_t numerator, int32_t denominator)
{
    bool negative;
    uint64_t magnitude;
    uint32_t divisor;
    if (denominator == 0) return 0;
    negative = (numerator < 0) != (denominator < 0);
    magnitude = numerator < 0 ? (uint64_t)(-(numerator + 1)) + 1U :
                                (uint64_t)numerator;
    divisor = denominator < 0 ? abs_i32(denominator) :
                                (uint32_t)denominator;
    magnitude = (magnitude + divisor / 2U) / divisor;
    if (negative) return magnitude > 2147483648ULL ? INT32_MIN :
        (magnitude == 2147483648ULL ? INT32_MIN : -(int32_t)magnitude);
    return magnitude > INT32_MAX ? INT32_MAX : (int32_t)magnitude;
}

static uint16_t local_mdeg_for_count(
    const int32_t angles[LEVER_CALIBRATION_POINT_COUNT],
    const int32_t counts[LEVER_CALIBRATION_POINT_COUNT], uint8_t point,
    uint32_t count_delta)
{
    uint8_t left = point == 0U ? 0U : (uint8_t)(point - 1U);
    uint8_t right = point == (LEVER_CALIBRATION_POINT_COUNT - 1U) ?
        point : (uint8_t)(point + 1U);
    uint32_t dc = abs_i32(counts[right] - counts[left]);
    uint32_t da = abs_i32(angles[right] - angles[left]);
    uint32_t value;
    if (dc == 0U) return UINT16_MAX;
    value = (uint32_t)(((uint64_t)count_delta * da + dc / 2U) / dc);
    return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}

static int32_t interpolate(const int32_t *angles, const int32_t *counts,
                           int32_t angle)
{
    uint8_t i;
    if (angle <= angles[0]) return counts[0];
    if (angle >= angles[LEVER_CALIBRATION_POINT_COUNT - 1U])
        return counts[LEVER_CALIBRATION_POINT_COUNT - 1U];
    for (i = 1U; i < LEVER_CALIBRATION_POINT_COUNT; i++) {
        if (angle <= angles[i]) {
            int32_t da = angles[i] - angles[i - 1U];
            int32_t offset = angle - angles[i - 1U];
            int32_t dc = counts[i] - counts[i - 1U];
            return counts[i - 1U] +
                divide_round((int64_t)dc * offset, da);
        }
    }
    return counts[LEVER_CALIBRATION_POINT_COUNT - 1U];
}

LeverCalibrationResult_t LeverCalibration_Evaluate(
    const int32_t angles_mdeg[LEVER_CALIBRATION_POINT_COUNT],
    const int16_t captures[2][LEVER_CALIBRATION_REPEAT_COUNT]
                          [LEVER_CALIBRATION_POINT_COUNT],
    uint16_t reference_encoder, int32_t horizontal_offset_counts,
    uint32_t driver_config_fingerprint,
    LeverCalibrationValues_t *candidate)
{
    int32_t mean[2][LEVER_CALIBRATION_POINT_COUNT];
    int32_t sign;
    uint32_t max_repeat = 0U;
    uint32_t max_backlash = 0U;
    uint8_t direction;
    uint8_t point;
    if ((angles_mdeg == NULL) || (captures == NULL) || (candidate == NULL) ||
        (driver_config_fingerprint == 0U)) return LEVER_CAL_RESULT_CONFIG;
    memset(candidate, 0, sizeof(*candidate));
    if ((angles_mdeg[0] >= 0) ||
        (angles_mdeg[LEVER_CALIBRATION_POINT_COUNT - 1U] <= 0))
        return LEVER_CAL_RESULT_NON_MONOTONIC;
    for (point = 1U; point < LEVER_CALIBRATION_POINT_COUNT; point++)
        if (angles_mdeg[point] <= angles_mdeg[point - 1U])
            return LEVER_CAL_RESULT_NON_MONOTONIC;
    for (direction = 0U; direction < 2U; direction++)
        for (point = 0U; point < LEVER_CALIBRATION_POINT_COUNT; point++)
            mean[direction][point] = divide_round(
                (int32_t)captures[direction][0][point] +
                captures[direction][1][point] +
                captures[direction][2][point], 3);
    if (mean[0][1] == mean[0][0]) return LEVER_CAL_RESULT_NON_MONOTONIC;
    sign = mean[0][1] > mean[0][0] ? 1 : -1;
    for (direction = 0U; direction < 2U; direction++) {
        for (point = 1U; point < LEVER_CALIBRATION_POINT_COUNT; point++) {
            int32_t delta = mean[direction][point] -
                            mean[direction][point - 1U];
            if ((sign > 0 && delta <= 0) || (sign < 0 && delta >= 0))
                return LEVER_CAL_RESULT_NON_MONOTONIC;
        }
        for (point = 0U; point < LEVER_CALIBRATION_POINT_COUNT; point++) {
            int16_t minimum = captures[direction][0][point];
            int16_t maximum = minimum;
            uint8_t repeat;
            uint16_t repeat_mdeg;
            for (repeat = 1U; repeat < LEVER_CALIBRATION_REPEAT_COUNT;
                 repeat++) {
                int16_t value = captures[direction][repeat][point];
                if (value < minimum) minimum = value;
                if (value > maximum) maximum = value;
            }
            repeat_mdeg = local_mdeg_for_count(angles_mdeg,
                mean[direction], point, (uint16_t)(maximum - minimum));
            if (repeat_mdeg > max_repeat) max_repeat = repeat_mdeg;
        }
    }
    if (max_repeat > LEVER_CAL_REPEATABILITY_MAX)
        return LEVER_CAL_RESULT_REPEATABILITY;
    for (point = 0U; point < LEVER_CALIBRATION_POINT_COUNT; point++) {
        uint16_t backlash = local_mdeg_for_count(angles_mdeg, mean[0], point,
            abs_i32(mean[0][point] - mean[1][point]));
        if (backlash > max_backlash) max_backlash = backlash;
    }
    if (max_backlash > LEVER_CAL_BACKLASH_MAX)
        return LEVER_CAL_RESULT_BACKLASH;
    for (direction = 0U; direction < 2U; direction++) {
        uint8_t segment;
        uint64_t previous_num = 0U;
        uint32_t previous_den = 1U;
        for (segment = 1U; segment < LEVER_CALIBRATION_POINT_COUNT;
             segment++) {
            uint32_t dc = abs_i32(mean[direction][segment] -
                                  mean[direction][segment - 1U]);
            uint32_t da = (uint32_t)(angles_mdeg[segment] -
                                     angles_mdeg[segment - 1U]);
            if ((segment > 1U) &&
                (((uint64_t)dc * previous_den >
                  2ULL * previous_num * da) ||
                 (previous_num * da > 2ULL * (uint64_t)dc * previous_den)))
                return LEVER_CAL_RESULT_SLOPE;
            previous_num = dc;
            previous_den = da;
        }
    }
    candidate->data_version = LEVER_CALIBRATION_DATA_VERSION;
    candidate->driver_config_fingerprint = driver_config_fingerprint;
    candidate->reference_encoder = reference_encoder;
    candidate->horizontal_offset_counts = horizontal_offset_counts;
    candidate->repeatability_mdeg = (uint16_t)max_repeat;
    candidate->backlash_mdeg = (uint16_t)max_backlash;
    for (point = 0U; point < LEVER_CALIBRATION_POINT_COUNT; point++) {
        candidate->angle_mdeg[point] = angles_mdeg[point];
        candidate->increasing_counts[point] = mean[0][point];
        candidate->decreasing_counts[point] = mean[1][point];
    }
    {
        int32_t left_span = -angles_mdeg[0];
        int32_t right_span =
            angles_mdeg[LEVER_CALIBRATION_POINT_COUNT - 1U];
        int32_t left_inset = left_span / 10;
        int32_t right_inset = right_span / 10;
        int32_t bounds[4];
        if (left_inset < 500) left_inset = 500;
        if (right_inset < 500) right_inset = 500;
        candidate->soft_min_angle_mdeg = angles_mdeg[0] + left_inset;
        candidate->soft_max_angle_mdeg =
            angles_mdeg[LEVER_CALIBRATION_POINT_COUNT - 1U] - right_inset;
        if (candidate->soft_min_angle_mdeg >=
            candidate->soft_max_angle_mdeg)
            return LEVER_CAL_RESULT_NON_MONOTONIC;
        bounds[0] = interpolate(angles_mdeg, mean[0],
                                candidate->soft_min_angle_mdeg);
        bounds[1] = interpolate(angles_mdeg, mean[0],
                                candidate->soft_max_angle_mdeg);
        bounds[2] = interpolate(angles_mdeg, mean[1],
                                candidate->soft_min_angle_mdeg);
        bounds[3] = interpolate(angles_mdeg, mean[1],
                                candidate->soft_max_angle_mdeg);
        candidate->soft_min_counts = bounds[0];
        candidate->soft_max_counts = bounds[0];
        for (point = 1U; point < 4U; point++) {
            if (bounds[point] < candidate->soft_min_counts)
                candidate->soft_min_counts = bounds[point];
            if (bounds[point] > candidate->soft_max_counts)
                candidate->soft_max_counts = bounds[point];
        }
    }
    return LEVER_CAL_RESULT_OK;
}

static void finish(LeverCalibrationState_t state,
                   LeverCalibrationResult_t result)
{
    s_status.state = state;
    s_status.result = result;
    s_status.capture_pending = false;
    s_capture_kind = CAPTURE_NONE;
    s_status.owner = LEVER_CAL_OWNER_NONE;
    s_status.owner_id = 0U;
    if (SystemState_Get() == SYSTEM_STATE_CALIBRATION)
        (void)SystemState_EndCalibration();
}

static void fail(LeverCalibrationResult_t result, uint32_t detail)
{
    s_status.reject_detail = detail;
    s_status.session_zero_valid = false;
    finish(LEVER_CAL_STATE_FAILED, result);
}

static bool start_read(uint32_t now_ms, CaptureKind_t kind,
                       uint8_t required)
{
    ZdtStepperSnapshot_t device;
    ZdtStepper_GetSnapshot(&device);
    s_seen_motor_sequence = device.motor_status_sequence;
    s_seen_config_sequence = device.driver_config_sequence;
    s_seen_encoder_sequence = device.encoder_sequence;
    if (ZdtStepper_ReadMotorStatus() != ZDT_STEPPER_OK) return false;
    s_capture_kind = kind;
    s_capture_phase = CAPTURE_WAIT_STATUS;
    s_capture_request_sent = true;
    s_sample_required = required;
    s_status.sample_count = 0U;
    s_status.capture_pending = true;
    s_deadline_ms = now_ms + LEVER_CAL_CAPTURE_TIMEOUT_MS;
    return true;
}

static uint16_t circular_spread(const uint16_t *samples, uint8_t count)
{
    int32_t minimum = 0;
    int32_t maximum = 0;
    uint8_t i;
    for (i = 1U; i < count; i++) {
        int32_t delta = ZdtStepper_CircularDelta(samples[i], samples[0]);
        if (delta < minimum) minimum = delta;
        if (delta > maximum) maximum = delta;
    }
    return (uint16_t)(maximum - minimum);
}

static uint16_t circular_mean(const uint16_t *samples, uint8_t count)
{
    int32_t sum = 0;
    uint8_t i;
    for (i = 1U; i < count; i++)
        sum += ZdtStepper_CircularDelta(samples[i], samples[0]);
    return (uint16_t)(samples[0] + divide_round(sum, count));
}

static void advance_point(void)
{
    s_status.progress++;
    if (s_status.expected_direction == LEVER_CAL_DIRECTION_INCREASING) {
        if (s_status.expected_point < LEVER_CALIBRATION_POINT_COUNT - 1U) {
            s_status.expected_point++;
        } else {
            s_status.expected_direction = LEVER_CAL_DIRECTION_DECREASING;
            s_status.expected_point = LEVER_CALIBRATION_POINT_COUNT - 1U;
        }
    } else if (s_status.expected_point > 0U) {
        s_status.expected_point--;
    } else if (s_status.expected_repeat + 1U <
               LEVER_CALIBRATION_REPEAT_COUNT) {
        s_status.expected_repeat++;
        s_status.expected_direction = LEVER_CAL_DIRECTION_INCREASING;
        s_status.expected_point = 0U;
    }
}

static void finish_samples(uint32_t now_ms)
{
    uint16_t spread = circular_spread(s_samples, s_sample_required);
    uint16_t mean = circular_mean(s_samples, s_sample_required);
    s_status.capture_pending = false;
    s_capture_kind = CAPTURE_NONE;
    if (s_sample_required == LEVER_CALIBRATION_REFERENCE_SAMPLES) {
        if (spread > LEVER_CAL_REFERENCE_SPREAD_MAX) {
            fail(LEVER_CAL_RESULT_UNSTABLE, spread);
            return;
        }
        s_reference_spread = spread;
        if (s_status.rearm_only &&
            abs_i32(ZdtStepper_CircularDelta(mean,
                    s_persisted.reference_encoder)) >
                    LEVER_CAL_REFERENCE_SPREAD_MAX) {
            fail(LEVER_CAL_RESULT_CONFIG,
                 abs_i32(ZdtStepper_CircularDelta(mean,
                         s_persisted.reference_encoder)));
            return;
        }
        s_session_reference = mean;
        s_status.last_capture_counts = 0;
        s_status.candidate.reference_encoder = mean;
        s_status.candidate.reference_spread_counts = spread;
        s_status.candidate.driver_config_fingerprint =
            s_candidate_fingerprint;
        s_status.state = LEVER_CAL_STATE_WAIT_HORIZONTAL;
        s_deadline_ms = now_ms + LEVER_CAL_WAIT_TIMEOUT_MS;
        return;
    }
    if (spread > LEVER_CAL_POINT_SPREAD_MAX) {
        fail(LEVER_CAL_RESULT_UNSTABLE, spread);
        return;
    }
    if (spread > s_max_point_spread) s_max_point_spread = spread;
    {
        int32_t relative = ZdtStepper_CircularDelta(mean,
                                                    s_session_reference);
        uint8_t direction = s_status.expected_direction;
        uint8_t repeat = s_status.expected_repeat;
        uint8_t point = s_status.expected_point;
        if ((relative < INT16_MIN) || (relative > INT16_MAX)) {
            fail(LEVER_CAL_RESULT_NON_MONOTONIC, (uint32_t)relative);
            return;
        }
        if ((direction == LEVER_CAL_DIRECTION_INCREASING) &&
            (repeat == 0U)) {
            if ((point > 0U) &&
                (s_requested_angle_mdeg <= s_angles[point - 1U])) {
                fail(LEVER_CAL_RESULT_NON_MONOTONIC, point);
                return;
            }
            s_angles[point] = s_requested_angle_mdeg;
        } else if (abs_i32(s_requested_angle_mdeg - s_angles[point]) >
                   LEVER_CAL_REPEATABILITY_MAX) {
            fail(LEVER_CAL_RESULT_REPEATABILITY, point);
            return;
        }
        s_captures[direction][repeat][point] = (int16_t)relative;
        s_status.last_capture_counts = relative;
        advance_point();
        s_deadline_ms = now_ms + LEVER_CAL_WAIT_TIMEOUT_MS;
    }
}

void LeverCalibration_Init(uint32_t now_ms)
{
    ParameterStoreInfo_t store;
    (void)now_ms;
    memset(&s_status, 0, sizeof(s_status));
    memset(&s_persisted, 0, sizeof(s_persisted));
    ParameterStore_GetInfo(&store);
    s_status.generation = store.generation;
    s_status.expected_generation = store.generation;
    s_status.required = LEVER_CALIBRATION_TOTAL_POINTS;
    s_persisted_valid = ParameterStore_GetLoadedLeverCalibration(&s_persisted);
    s_status.persisted_valid = s_persisted_valid;
    if (s_persisted_valid) {
        s_status.candidate = s_persisted;
        s_status.candidate_crc32 = ParameterStore_Crc32(
            &s_persisted, sizeof(s_persisted));
    }
}

LeverCalibrationResult_t LeverCalibration_Command(
    LeverCalibrationOwner_t owner, uint32_t owner_id,
    LeverCalibrationCommand_t command, uint32_t expected_generation,
    uint8_t point, LeverCalibrationDirection_t direction, uint8_t repeat,
    int32_t angle_mdeg, uint16_t flags, uint32_t now_ms)
{
    ParameterStoreInfo_t store;
    ParameterStoreResult_t saved;
    if ((owner == LEVER_CAL_OWNER_NONE) || (owner_id == 0U))
        return LEVER_CAL_RESULT_WRONG_OWNER;
    ParameterStore_GetInfo(&store);
    if (command == LEVER_CAL_COMMAND_START) {
        ZdtStepperTestSnapshot_t stepper;
        if ((flags & (uint16_t)~(LEVER_CAL_START_PHONE_ZERO_VERIFIED |
                                 LEVER_CAL_START_FORCE_FULL)) != 0U ||
            (flags & LEVER_CAL_START_PHONE_ZERO_VERIFIED) == 0U ||
            abs_i32(angle_mdeg) > LEVER_CAL_REPEATABILITY_MAX)
            return LEVER_CAL_RESULT_INVALID_REQUEST;
        if ((s_status.owner != LEVER_CAL_OWNER_NONE) &&
            ((s_status.owner != owner) || (s_status.owner_id != owner_id)))
            return LEVER_CAL_RESULT_BUSY;
        if (expected_generation != store.generation)
            return LEVER_CAL_RESULT_STALE_GENERATION;
        ZdtStepperTest_GetSnapshot(&stepper, now_ms);
        if ((SystemState_Get() != SYSTEM_STATE_IDLE) || Safety_HasFault() ||
            (stepper.state == ZDT_TEST_STATE_RUNNING) ||
            (stepper.state == ZDT_TEST_STATE_WAITING_ACK) ||
            !SystemState_BeginCalibration())
            return LEVER_CAL_RESULT_WRONG_STATE;
        memset(&s_status, 0, sizeof(s_status));
        memset(s_angles, 0, sizeof(s_angles));
        memset(s_captures, 0, sizeof(s_captures));
        s_status.state = LEVER_CAL_STATE_WAIT_REFERENCE;
        s_status.owner = owner;
        s_status.owner_id = owner_id;
        s_status.generation = store.generation;
        s_status.expected_generation = expected_generation;
        s_status.required = LEVER_CALIBRATION_TOTAL_POINTS;
        s_status.persisted_valid = s_persisted_valid;
        s_status.rearm_only = s_persisted_valid &&
            ((flags & LEVER_CAL_START_FORCE_FULL) == 0U);
        if (s_status.rearm_only) {
            s_status.candidate = s_persisted;
            s_status.candidate_crc32 = ParameterStore_Crc32(
                &s_persisted, sizeof(s_persisted));
        }
        s_status.expected_direction = LEVER_CAL_DIRECTION_INCREASING;
        s_deadline_ms = now_ms + LEVER_CAL_WAIT_TIMEOUT_MS;
        s_candidate_fingerprint = 0U;
        s_reference_spread = 0U;
        s_max_point_spread = 0U;
        return LEVER_CAL_RESULT_OK;
    }
    if ((s_status.owner != owner) || (s_status.owner_id != owner_id))
        return LEVER_CAL_RESULT_WRONG_OWNER;
    if (command == LEVER_CAL_COMMAND_CANCEL) {
        finish(LEVER_CAL_STATE_CANCELLED, LEVER_CAL_RESULT_CANCELLED);
        return LEVER_CAL_RESULT_OK;
    }
    if (expected_generation != s_status.expected_generation)
        return LEVER_CAL_RESULT_STALE_GENERATION;
    if (s_status.capture_pending) return LEVER_CAL_RESULT_BUSY;
    if (command == LEVER_CAL_COMMAND_CAPTURE_REFERENCE) {
        if (s_status.state != LEVER_CAL_STATE_WAIT_REFERENCE)
            return LEVER_CAL_RESULT_WRONG_STATE;
        return start_read(now_ms, CAPTURE_REFERENCE,
            LEVER_CALIBRATION_REFERENCE_SAMPLES) ?
            LEVER_CAL_RESULT_OK : LEVER_CAL_RESULT_BUSY;
    }
    if (command == LEVER_CAL_COMMAND_CAPTURE_HORIZONTAL) {
        if ((s_status.state != LEVER_CAL_STATE_WAIT_HORIZONTAL) ||
            (abs_i32(angle_mdeg) > LEVER_CAL_REPEATABILITY_MAX))
            return LEVER_CAL_RESULT_INVALID_REQUEST;
        s_requested_angle_mdeg = angle_mdeg;
        return start_read(now_ms, CAPTURE_HORIZONTAL,
            LEVER_CALIBRATION_POINT_SAMPLES) ?
            LEVER_CAL_RESULT_OK : LEVER_CAL_RESULT_BUSY;
    }
    if (command == LEVER_CAL_COMMAND_CAPTURE_POINT) {
        if ((s_status.state != LEVER_CAL_STATE_CAPTURE_POINTS) ||
            (point != s_status.expected_point) ||
            ((uint8_t)direction != s_status.expected_direction) ||
            (repeat != s_status.expected_repeat) ||
            (abs_i32(angle_mdeg) > LEVER_CAL_ANGLE_ABS_MAX_MDEG))
            return LEVER_CAL_RESULT_INVALID_REQUEST;
        s_requested_angle_mdeg = angle_mdeg;
        return start_read(now_ms, CAPTURE_POINT,
            LEVER_CALIBRATION_POINT_SAMPLES) ?
            LEVER_CAL_RESULT_OK : LEVER_CAL_RESULT_BUSY;
    }
    if (command == LEVER_CAL_COMMAND_REVIEW) {
        LeverCalibrationResult_t evaluated;
        if ((s_status.state != LEVER_CAL_STATE_CAPTURE_POINTS) ||
            (s_status.progress != LEVER_CALIBRATION_TOTAL_POINTS))
            return LEVER_CAL_RESULT_WRONG_STATE;
        evaluated = LeverCalibration_Evaluate(s_angles, s_captures,
            s_session_reference, s_session_horizontal,
            s_candidate_fingerprint, &s_status.candidate);
        s_status.candidate.reference_spread_counts = s_reference_spread;
        s_status.candidate.point_spread_counts = s_max_point_spread;
        if (evaluated != LEVER_CAL_RESULT_OK) {
            fail(evaluated, 0U);
            return evaluated;
        }
        s_status.candidate_valid = true;
        s_status.candidate_crc32 = ParameterStore_Crc32(
            &s_status.candidate, sizeof(s_status.candidate));
        s_status.state = LEVER_CAL_STATE_REVIEW;
        s_deadline_ms = now_ms + LEVER_CAL_WAIT_TIMEOUT_MS;
        return LEVER_CAL_RESULT_OK;
    }
    if (command != LEVER_CAL_COMMAND_COMMIT ||
        (s_status.state != LEVER_CAL_STATE_REVIEW) ||
        !s_status.candidate_valid)
        return LEVER_CAL_RESULT_WRONG_STATE;
    s_status.state = LEVER_CAL_STATE_SAVING;
    saved = ParameterStore_SaveLeverCalibration(&s_status.candidate,
                                                expected_generation);
    if (saved == PARAMETER_STORE_ERR_STALE_GENERATION) {
        fail(LEVER_CAL_RESULT_STALE_GENERATION, store.generation);
        return LEVER_CAL_RESULT_STALE_GENERATION;
    }
    if (saved != PARAMETER_STORE_OK) {
        fail(LEVER_CAL_RESULT_STORAGE, (uint32_t)saved);
        return LEVER_CAL_RESULT_STORAGE;
    }
    s_persisted = s_status.candidate;
    s_persisted_valid = true;
    s_status.persisted_valid = true;
    s_status.session_zero_valid = true;
    ParameterStore_GetInfo(&store);
    s_status.generation = store.generation;
    finish(LEVER_CAL_STATE_COMPLETE, LEVER_CAL_RESULT_OK);
    return LEVER_CAL_RESULT_OK;
}

void LeverCalibration_Task(uint32_t now_ms)
{
    ZdtStepperSnapshot_t device;
    uint8_t bad_mask = ZDT_STEPPER_MOTOR_STALLED |
                       ZDT_STEPPER_MOTOR_STALL_PROTECTION |
                       ZDT_STEPPER_MOTOR_LEFT_LIMIT |
                       ZDT_STEPPER_MOTOR_RIGHT_LIMIT |
                       ZDT_STEPPER_MOTOR_POWER_LOSS;
    if (s_status.owner == LEVER_CAL_OWNER_NONE) return;
    if (Safety_HasFault() || (SystemState_Get() == SYSTEM_STATE_FAULT)) {
        fail(LEVER_CAL_RESULT_WRONG_STATE, 0U);
        return;
    }
    if (reached(now_ms, s_deadline_ms)) {
        fail(LEVER_CAL_RESULT_TIMEOUT, 0U);
        return;
    }
    if (!s_status.capture_pending) return;
    ZdtStepper_GetSnapshot(&device);
    if (s_capture_phase == CAPTURE_WAIT_STATUS) {
        if (device.motor_status_sequence == s_seen_motor_sequence) return;
        s_seen_motor_sequence = device.motor_status_sequence;
        if (!device.motor_status_valid) {
            fail(LEVER_CAL_RESULT_STATUS_STALE, 0U);
            return;
        }
        if ((device.motor_status & ZDT_STEPPER_MOTOR_ENABLED) != 0U) {
            fail(LEVER_CAL_RESULT_MOTOR_NOT_DISABLED, device.motor_status);
            return;
        }
        if ((device.motor_status & bad_mask) != 0U) {
            fail(LEVER_CAL_RESULT_MOTOR_PROTECTION, device.motor_status);
            return;
        }
        s_capture_status_ms = now_ms;
        s_capture_phase = CAPTURE_WAIT_CONFIG;
        s_capture_request_sent =
            ZdtStepper_ReadDriverConfig() == ZDT_STEPPER_OK;
        return;
    }
    if (s_capture_phase == CAPTURE_WAIT_CONFIG) {
        if (!s_capture_request_sent) {
            s_capture_request_sent =
                ZdtStepper_ReadDriverConfig() == ZDT_STEPPER_OK;
            return;
        }
        if (device.driver_config_sequence == s_seen_config_sequence) return;
        s_seen_config_sequence = device.driver_config_sequence;
        if (!device.driver_config_valid ||
            (device.driver_config_fingerprint == 0U)) {
            fail(LEVER_CAL_RESULT_CONFIG, 0U);
            return;
        }
        if (s_candidate_fingerprint == 0U)
            s_candidate_fingerprint = device.driver_config_fingerprint;
        s_status.candidate.driver_config_fingerprint =
            s_candidate_fingerprint;
        if ((device.driver_config_fingerprint != s_candidate_fingerprint) ||
            (s_status.rearm_only &&
             device.driver_config_fingerprint !=
             s_persisted.driver_config_fingerprint)) {
            fail(LEVER_CAL_RESULT_CONFIG,
                 device.driver_config_fingerprint);
            return;
        }
        s_capture_phase = CAPTURE_WAIT_ENCODER;
        /* The configuration reply is much longer than the ordinary status
         * replies.  Some Emm V5.0 revisions ignore a new command sent at the
         * same frame boundary, so defer the first 0x31 request explicitly. */
        s_capture_request_sent = false;
        s_encoder_request_ms = now_ms +
            LEVER_CAL_CONFIG_TO_ENCODER_GAP_MS;
        return;
    }
    if (device.encoder_sequence != s_seen_encoder_sequence) {
        s_seen_encoder_sequence = device.encoder_sequence;
        if (!device.encoder_valid ||
            reached(now_ms,
                    s_capture_status_ms + LEVER_CAL_STATUS_FRESH_MS + 1U)) {
            fail(LEVER_CAL_RESULT_STATUS_STALE, 0U);
            return;
        }
        s_samples[s_status.sample_count++] = device.encoder_raw;
        s_capture_request_sent = false;
    } else {
        /* Read responses can be lost on a noisy shared TTL link.  A single
         * dropped 0x31 frame must not turn a read-only calibration into a
         * two-second dead end; retry at a bounded cadence until the overall
         * capture deadline expires. */
        if (!reached(now_ms, s_encoder_request_ms)) return;
        if (ZdtStepper_ReadEncoder() == ZDT_STEPPER_OK) {
            s_capture_request_sent = true;
            s_encoder_request_ms = now_ms + LEVER_CAL_ENCODER_RETRY_MS;
        }
        return;
    }
    if (s_status.sample_count < s_sample_required) {
        s_encoder_request_ms = now_ms +
            LEVER_CAL_CONFIG_TO_ENCODER_GAP_MS;
        return;
    }
    if (s_capture_kind == CAPTURE_HORIZONTAL) {
        uint16_t spread = circular_spread(s_samples, s_sample_required);
        uint16_t mean = circular_mean(s_samples, s_sample_required);
        int32_t horizontal = ZdtStepper_CircularDelta(mean,
                                                      s_session_reference);
        s_status.capture_pending = false;
        s_capture_kind = CAPTURE_NONE;
        if (spread > LEVER_CAL_POINT_SPREAD_MAX) {
            fail(LEVER_CAL_RESULT_UNSTABLE, spread);
            return;
        }
        if (s_status.rearm_only) {
            if (abs_i32(horizontal - s_persisted.horizontal_offset_counts) >
                LEVER_CAL_HORIZONTAL_CHECK_MAX) {
                fail(LEVER_CAL_RESULT_CONFIG, abs_i32(horizontal -
                     s_persisted.horizontal_offset_counts));
                return;
            }
            s_session_horizontal = horizontal;
            s_status.candidate = s_persisted;
            s_status.candidate_valid = true;
            s_status.session_zero_valid = true;
            s_status.candidate_crc32 = ParameterStore_Crc32(
                &s_status.candidate, sizeof(s_status.candidate));
            finish(LEVER_CAL_STATE_COMPLETE, LEVER_CAL_RESULT_OK);
            return;
        }
        s_session_horizontal = horizontal;
        s_status.last_capture_counts = horizontal;
        s_status.candidate.horizontal_offset_counts = horizontal;
        s_status.candidate.point_spread_counts = spread;
        s_status.state = LEVER_CAL_STATE_CAPTURE_POINTS;
        s_deadline_ms = now_ms + LEVER_CAL_WAIT_TIMEOUT_MS;
        return;
    }
    finish_samples(now_ms);
}

void LeverCalibration_CancelHost(uint32_t owner_id, uint32_t now_ms)
{
    if ((s_status.owner == LEVER_CAL_OWNER_HOST) &&
        (s_status.owner_id == owner_id))
        (void)LeverCalibration_Command(LEVER_CAL_OWNER_HOST, owner_id,
            LEVER_CAL_COMMAND_CANCEL, s_status.expected_generation,
            0U, LEVER_CAL_DIRECTION_INCREASING, 0U, 0, 0U, now_ms);
}

void LeverCalibration_GetStatus(LeverCalibrationStatus_t *status)
{
    if (s_status.owner == LEVER_CAL_OWNER_NONE) {
        ParameterStoreInfo_t store;
        ParameterStore_GetInfo(&store);
        s_status.generation = store.generation;
        s_status.expected_generation = store.generation;
    }
    if (status != NULL) *status = s_status;
}

bool LeverCalibration_IsActive(void)
{
    return s_status.owner != LEVER_CAL_OWNER_NONE;
}

bool LeverCalibration_GetActive(LeverCalibrationValues_t *values)
{
    if ((values == NULL) || !s_persisted_valid) return false;
    *values = s_persisted;
    return true;
}

bool LeverCalibration_GetSessionReference(uint16_t *reference_encoder,
                                          int32_t *horizontal_counts)
{
    if (!s_status.session_zero_valid) return false;
    if (reference_encoder != NULL) *reference_encoder = s_session_reference;
    if (horizontal_counts != NULL) *horizontal_counts = s_session_horizontal;
    return true;
}

void LeverCalibration_InvalidateDriverConfig(uint32_t actual_fingerprint)
{
    s_persisted_valid = false;
    s_status.persisted_valid = false;
    s_status.candidate_valid = false;
    s_status.session_zero_valid = false;
    s_status.reject_detail = actual_fingerprint;
    s_status.result = LEVER_CAL_RESULT_CONFIG;
    if (s_status.owner != LEVER_CAL_OWNER_NONE)
        finish(LEVER_CAL_STATE_FAILED, LEVER_CAL_RESULT_CONFIG);
    else
        s_status.state = LEVER_CAL_STATE_FAILED;
}
