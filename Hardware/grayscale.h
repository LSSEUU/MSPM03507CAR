#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include "board.h"

#define GRAYSCALE_CHANNEL_COUNT  8U
#define GRAY_SAMPLE_COUNT        5U
#define GRAY_DEBOUNCE_COUNT      3U
#define GRAY_POSITION_SCALE      1000
#define GRAY_LINE_LOST           (-1)
#define GRAY_MIN_LINE_BITS       3U

typedef struct {
    uint8_t raw_state;
    uint8_t filtered_state;
    int32_t position;
    uint8_t debounce_cnt[GRAYSCALE_CHANNEL_COUNT];
    uint8_t min_bits;
    uint32_t scan_count;
    uint32_t error_count;
} Grayscale_t;

typedef Grayscale_t GrayscaleSnapshot_t;

extern Grayscale_t Gray;

void Grayscale_Init(void);
void Grayscale_Update(void);
void Grayscale_SetMinBits(uint8_t bits);
void Grayscale_GetSnapshot(GrayscaleSnapshot_t *snapshot);

#endif
