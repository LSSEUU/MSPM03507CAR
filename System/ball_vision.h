#ifndef BALL_VISION_H
#define BALL_VISION_H

#include <stdbool.h>
#include <stdint.h>

#define BALL_VISION_FRAME_SIZE              9U
#define BALL_VISION_FRESH_TIMEOUT_MS       150UL
#define BALL_VISION_POSITION_LIMIT_X10    1150
#define BALL_VISION_VELOCITY_LIMIT_MM_S   4000

typedef struct {
    uint32_t time_ms;
    uint32_t received_ms;
    uint32_t last_valid_ms;
    uint32_t age_ms;
    uint32_t frames;
    uint32_t valid_frames;
    uint32_t explicit_invalid_frames;
    uint32_t malformed_frames;
    uint32_t crc_errors;
    uint32_t duplicate_frames;
    uint32_t sequence_gaps;
    uint32_t rx_overflow;
    uint32_t hardware_errors;
    int16_t x_mm_x10;
    int16_t vx_mm_s;
    uint8_t seq;
    uint8_t valid_streak;
    bool seen_sequence;
    bool valid;
    bool fresh;
} BallVisionSnapshot_t;

void BallVision_Init(uint32_t now_ms);
void BallVision_Service(uint32_t now_ms);
void BallVision_GetSnapshot(uint32_t now_ms, BallVisionSnapshot_t *snapshot);

/* Host-test entry point.  Production receives the same stream from UART1. */
void BallVision_ProcessBytes(const uint8_t *data, uint32_t length,
                             uint32_t now_ms);

#endif
