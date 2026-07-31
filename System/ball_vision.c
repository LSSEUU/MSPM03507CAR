#include "ball_vision.h"

#include "m6_vision_uart.h"
#include "vision_uart.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define BALL_HEADER_0                  0xA5U
#define BALL_HEADER_1                  0x5AU
#define BALL_FLAG_VALID                0x01U
#define BALL_VALID_STREAK_REQUIRED        2U
#define BALL_READ_CHUNK_SIZE             32U

static BallVisionSnapshot_t s_snapshot;
static uint8_t s_frame[BALL_VISION_FRAME_SIZE];
static uint8_t s_frame_length;
static uint32_t s_seen_rx_overflow;
static uint32_t s_seen_hardware_errors;

static uint8_t crc8_atm(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    uint8_t index;
    for (index = 0U; index < length; index++) {
        uint8_t bit;
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc & 0x80U) != 0U ?
                (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
    return crc;
}

static int16_t i16_le(const uint8_t *data)
{
    uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return (int16_t)value;
}

static void invalidate_measurement(void)
{
    s_snapshot.valid_streak = 0U;
    s_snapshot.valid = false;
    s_snapshot.fresh = false;
}

/* Retain a possible embedded or trailing header after a malformed frame. */
static void resync_frame(void)
{
    uint8_t index;
    for (index = 1U; index < s_frame_length; index++) {
        if (s_frame[index] != BALL_HEADER_0) continue;
        if ((index + 1U < s_frame_length) &&
            (s_frame[index + 1U] != BALL_HEADER_1)) continue;
        memmove(s_frame, &s_frame[index], s_frame_length - index);
        s_frame_length = (uint8_t)(s_frame_length - index);
        return;
    }
    s_frame_length = 0U;
}

static bool consume_frame(uint32_t now_ms)
{
    uint8_t sequence = s_frame[2];
    uint8_t flags = s_frame[3];
    int16_t x = i16_le(&s_frame[4]);
    int16_t vx = i16_le(&s_frame[6]);
    bool sequence_gap = false;

    if (crc8_atm(s_frame, BALL_VISION_FRAME_SIZE - 1U) !=
        s_frame[BALL_VISION_FRAME_SIZE - 1U]) {
        s_snapshot.crc_errors++;
        s_snapshot.malformed_frames++;
        return false;
    }
    if ((flags & (uint8_t)~BALL_FLAG_VALID) != 0U ||
        (((flags & BALL_FLAG_VALID) == 0U) && ((x != 0) || (vx != 0))) ||
        (((flags & BALL_FLAG_VALID) != 0U) &&
         ((x < -BALL_VISION_POSITION_LIMIT_X10) ||
          (x > BALL_VISION_POSITION_LIMIT_X10) ||
          (vx < -BALL_VISION_VELOCITY_LIMIT_MM_S) ||
          (vx > BALL_VISION_VELOCITY_LIMIT_MM_S)))) {
        s_snapshot.malformed_frames++;
        return false;
    }

    /* Invalid is authoritative even if a stopped camera repeats a sequence. */
    if ((flags & BALL_FLAG_VALID) == 0U) invalidate_measurement();
    if (s_snapshot.seen_sequence) {
        uint8_t delta = (uint8_t)(sequence - s_snapshot.seq);
        if (delta == 0U) {
            s_snapshot.duplicate_frames++;
            return true;
        }
        if (delta != 1U) {
            s_snapshot.sequence_gaps += (uint32_t)(delta - 1U);
            sequence_gap = true;
        }
    }

    s_snapshot.seen_sequence = true;
    s_snapshot.seq = sequence;
    s_snapshot.frames++;
    s_snapshot.received_ms = now_ms;
    if ((flags & BALL_FLAG_VALID) == 0U) {
        s_snapshot.explicit_invalid_frames++;
        s_snapshot.x_mm_x10 = 0;
        s_snapshot.vx_mm_s = 0;
        return true;
    }

    if (sequence_gap) s_snapshot.valid_streak = 0U;
    if (s_snapshot.valid_streak < UINT8_MAX) s_snapshot.valid_streak++;
    s_snapshot.valid_frames++;
    s_snapshot.x_mm_x10 = x;
    s_snapshot.vx_mm_s = vx;
    s_snapshot.last_valid_ms = now_ms;
    s_snapshot.valid =
        s_snapshot.valid_streak >= BALL_VALID_STREAK_REQUIRED;
    s_snapshot.fresh = s_snapshot.valid;
    return true;
}

void BallVision_ProcessBytes(const uint8_t *data, uint32_t length,
                             uint32_t now_ms)
{
    uint32_t index;
    if ((data == NULL) || (length == 0U)) return;
    for (index = 0U; index < length; index++) {
        uint8_t byte = data[index];
        if (s_frame_length == 0U) {
            if (byte == BALL_HEADER_0) s_frame[s_frame_length++] = byte;
            continue;
        }
        if (s_frame_length == 1U) {
            if (byte == BALL_HEADER_1) {
                s_frame[s_frame_length++] = byte;
            } else if (byte != BALL_HEADER_0) {
                s_frame_length = 0U;
            }
            continue;
        }
        s_frame[s_frame_length++] = byte;
        if (s_frame_length == BALL_VISION_FRAME_SIZE) {
            if (consume_frame(now_ms)) s_frame_length = 0U;
            else resync_frame();
        }
    }
}

void BallVision_Init(uint32_t now_ms)
{
    VisionUartStats_t stats;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(s_frame, 0, sizeof(s_frame));
    s_frame_length = 0U;
    s_snapshot.time_ms = now_ms;
    s_snapshot.received_ms = UINT32_MAX;
    s_snapshot.last_valid_ms = UINT32_MAX;
    s_snapshot.age_ms = UINT32_MAX;
    VisionUart_GetStats(&stats);
    s_seen_rx_overflow = stats.rx_overflow;
    s_seen_hardware_errors = stats.hardware_errors;
    s_snapshot.rx_overflow = stats.rx_overflow;
    s_snapshot.hardware_errors = stats.hardware_errors;
}

void BallVision_Service(uint32_t now_ms)
{
    VisionUartStats_t stats;
    uint8_t data[BALL_READ_CHUNK_SIZE];
    size_t length;

    VisionUart_GetStats(&stats);
    if ((stats.rx_overflow != s_seen_rx_overflow) ||
        (stats.hardware_errors != s_seen_hardware_errors)) {
        s_seen_rx_overflow = stats.rx_overflow;
        s_seen_hardware_errors = stats.hardware_errors;
        s_snapshot.rx_overflow = stats.rx_overflow;
        s_snapshot.hardware_errors = stats.hardware_errors;
        s_frame_length = 0U;
        invalidate_measurement();
    }

    do {
        length = VisionUart_Read(data, sizeof(data));
        if (length != 0U) {
            M6VisionUart_MirrorRx(data, length);
            BallVision_ProcessBytes(data, (uint32_t)length, now_ms);
        }
    } while (length == sizeof(data));

    s_snapshot.time_ms = now_ms;
    if (s_snapshot.last_valid_ms == UINT32_MAX) {
        s_snapshot.age_ms = UINT32_MAX;
        invalidate_measurement();
    } else {
        s_snapshot.age_ms = now_ms - s_snapshot.last_valid_ms;
        if (s_snapshot.age_ms > BALL_VISION_FRESH_TIMEOUT_MS)
            invalidate_measurement();
        else
            s_snapshot.fresh = s_snapshot.valid;
    }
}

void BallVision_GetSnapshot(uint32_t now_ms, BallVisionSnapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    s_snapshot.time_ms = now_ms;
    if (s_snapshot.last_valid_ms == UINT32_MAX) {
        s_snapshot.age_ms = UINT32_MAX;
        s_snapshot.fresh = false;
    } else {
        s_snapshot.age_ms = now_ms - s_snapshot.last_valid_ms;
        if (s_snapshot.age_ms > BALL_VISION_FRESH_TIMEOUT_MS) {
            s_snapshot.valid = false;
            s_snapshot.fresh = false;
            s_snapshot.valid_streak = 0U;
        } else {
            s_snapshot.fresh = s_snapshot.valid;
        }
    }
    *snapshot = s_snapshot;
}
