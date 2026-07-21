#ifndef GYRO_H
#define GYRO_H

#include <stdint.h>

typedef struct {
    float yaw;
    float pitch;
    float roll;
} Gyro_t;

typedef struct {
    Gyro_t sample;
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t last_update_ms;
    uint32_t checksum_errors;
    uint32_t frame_errors;
    uint8_t recent_count;
    uint8_t recent_bytes[32];
} GyroSnapshot_t;

typedef struct {
    uint32_t pa21_transitions;
    uint32_t pa22_transitions;
    uint8_t pa21_level;
    uint8_t pa22_level;
} GyroPinProbe_t;

void Gyro_Init(void);
void Gyro_Update(void);
void Gyro_GetSnapshot(GyroSnapshot_t *snapshot);
void Gyro_ProbePins(GyroPinProbe_t *probe);

#endif
