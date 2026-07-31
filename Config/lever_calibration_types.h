#ifndef LEVER_CALIBRATION_TYPES_H
#define LEVER_CALIBRATION_TYPES_H

#include <stdint.h>

#define LEVER_CALIBRATION_DATA_VERSION 3UL
#define LEVER_CALIBRATION_POINT_COUNT  9U

/* Persisted calibration data.  Counts are signed, unwrapped offsets from the
 * captured gravity reference; angles are lever tilt in millidegrees. */
typedef struct {
    uint32_t data_version;
    uint32_t driver_config_fingerprint;
    int32_t horizontal_offset_counts;
    int32_t angle_mdeg[LEVER_CALIBRATION_POINT_COUNT];
    int32_t increasing_counts[LEVER_CALIBRATION_POINT_COUNT];
    int32_t decreasing_counts[LEVER_CALIBRATION_POINT_COUNT];
    int32_t soft_min_angle_mdeg;
    int32_t soft_max_angle_mdeg;
    int32_t soft_min_counts;
    int32_t soft_max_counts;
    uint16_t reference_encoder;
    uint16_t repeatability_mdeg;
    uint16_t backlash_mdeg;
    uint16_t reference_spread_counts;
    uint16_t point_spread_counts;
    uint16_t reserved;
} LeverCalibrationValues_t;

#endif
