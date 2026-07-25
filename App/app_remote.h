#ifndef APP_REMOTE_H
#define APP_REMOTE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_REMOTE_IDLE = 0,
    APP_REMOTE_MANUAL,
    APP_REMOTE_PENDING_DEMO,
    APP_REMOTE_DEMO_RUNNING,
    APP_REMOTE_CAPTURE
} AppRemoteState_t;

typedef enum {
    APP_REMOTE_DIRECTION_NONE = 0,
    APP_REMOTE_DIRECTION_FORWARD,
    APP_REMOTE_DIRECTION_REVERSE,
    APP_REMOTE_DIRECTION_LEFT,
    APP_REMOTE_DIRECTION_RIGHT
} AppRemoteDirection_t;

typedef struct {
    float speed_low_mm_s;
    float speed_medium_mm_s;
    float speed_high_mm_s;
    bool heading_hold;
} AppRemoteConfig_t;

typedef struct {
    AppRemoteState_t state;
    AppRemoteDirection_t direction;
    uint16_t speed_mm_s;
    uint32_t last_drive_event_ms;
    uint32_t pending_deadline_ms;
    uint32_t capture_deadline_ms;
    uint32_t address_rejects;
    uint32_t unknown_commands;
    uint32_t action_rejects;
    uint32_t heartbeat_stops;
    uint32_t captured_keys;
    bool actions_enabled;
    bool heading_hold_enabled;
    bool heading_hold_active;
} AppRemoteSnapshot_t;

void AppRemote_Init(void);
void AppRemote_GetDefaultConfig(AppRemoteConfig_t *config);
void AppRemote_GetConfig(AppRemoteConfig_t *config);
bool AppRemote_ApplyConfig(const AppRemoteConfig_t *config);
void AppRemote_Poll(void);
void AppRemote_Task20ms(uint32_t now_ms);
bool AppRemote_StartCapture(uint32_t now_ms);
bool AppRemote_NormalStop(uint32_t now_ms);
void AppRemote_GetSnapshot(AppRemoteSnapshot_t *snapshot);

#endif
