#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include <stdbool.h>
#include <stdint.h>

#define GRAYSCALE_CHANNEL_COUNT       8U
#define GRAYSCALE_SAMPLES_PER_CHANNEL 8U

typedef struct {
    uint16_t adc[GRAYSCALE_CHANNEL_COUNT];
    uint32_t scan_count;
    uint32_t last_update_ms;
    uint32_t error_count;
    uint32_t last_adc_status;
    uint32_t last_adc_flags;
    bool last_scan_ok;
    bool adc_powered;
    bool adc_enabled;
    bool adc_started;
} GrayscaleSnapshot_t;

void Grayscale_Init(void);
bool Grayscale_Scan(void);
bool Grayscale_RequestScan(void);
void Grayscale_Task(uint32_t now_ms);
bool Grayscale_IsScanPending(void);
void Grayscale_GetSnapshot(GrayscaleSnapshot_t *snapshot);

#endif
