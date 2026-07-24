#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include <stdbool.h>
#include <stdint.h>

#define GRAYSCALE_CHANNEL_COUNT       8U
typedef struct {
    uint8_t level[GRAYSCALE_CHANNEL_COUNT];
    uint8_t high_mask;
    uint32_t scan_count;
    uint32_t last_update_ms;
    uint32_t error_count;
    bool last_scan_ok;
} GrayscaleSnapshot_t;

void Grayscale_Init(void);
bool Grayscale_Scan(void);
bool Grayscale_RequestScan(void);
void Grayscale_Task(uint32_t now_ms);
bool Grayscale_IsScanPending(void);
void Grayscale_GetSnapshot(GrayscaleSnapshot_t *snapshot);

#endif
