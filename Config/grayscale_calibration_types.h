#ifndef GRAYSCALE_CALIBRATION_TYPES_H
#define GRAYSCALE_CALIBRATION_TYPES_H

#include <stdint.h>

#define GRAYSCALE_CALIBRATION_CHANNEL_COUNT 8U

typedef struct {
    uint16_t white_adc[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
    uint16_t black_adc[GRAYSCALE_CALIBRATION_CHANNEL_COUNT];
} LineCalibrationValues_t;

#endif
