#ifndef APP_MANUAL_DRIVE_H
#define APP_MANUAL_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_MANUAL_SOURCE_NONE = 0,
    APP_MANUAL_SOURCE_IR = 1,
    APP_MANUAL_SOURCE_VISION_UART = 2,
    APP_MANUAL_SOURCE_HOST_RPC = 3
} AppManualDriveSource_t;

typedef enum {
    APP_MANUAL_STOP_NONE = 0,
    APP_MANUAL_STOP_RELEASE = 1,
    APP_MANUAL_STOP_LEASE_EXPIRED = 2,
    APP_MANUAL_STOP_PREEMPTED = 3,
    APP_MANUAL_STOP_FAULT = 4,
    APP_MANUAL_STOP_ACTION_TAKEOVER = 5
} AppManualDriveStopReason_t;

typedef struct {
    AppManualDriveSource_t source;
    AppManualDriveStopReason_t stop_reason;
    int16_t left_mm_s;
    int16_t right_mm_s;
    uint32_t command_id;
    uint32_t last_update_ms;
    uint32_t lease_deadline_ms;
    uint32_t transition_id;
    uint32_t accepted_commands;
    uint32_t rejected_commands;
    uint32_t lease_stops;
    bool active;
} AppManualDriveSnapshot_t;

void AppManualDrive_Init(void);
bool AppManualDrive_Submit(AppManualDriveSource_t source,
                           int16_t left_mm_s, int16_t right_mm_s,
                           uint16_t lease_ms, uint32_t required_sensor_mask,
                           uint32_t command_id, uint32_t now_ms);
bool AppManualDrive_Stop(AppManualDriveSource_t source,
                         AppManualDriveStopReason_t reason,
                         uint32_t now_ms);
/* Reserved adapter for a future versioned UART1 vision parser. */
bool AppManualDrive_SubmitVision(int16_t left_mm_s, int16_t right_mm_s,
                                 uint16_t lease_ms, uint32_t command_id,
                                 uint32_t now_ms);
bool AppManualDrive_StopVision(uint32_t now_ms);
void AppManualDrive_Task20ms(uint32_t now_ms);
bool AppManualDrive_IsIdle(void);
void AppManualDrive_GetSnapshot(AppManualDriveSnapshot_t *snapshot);

#endif
