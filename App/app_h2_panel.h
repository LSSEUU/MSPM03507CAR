#ifndef APP_H2_PANEL_H
#define APP_H2_PANEL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    APP_H2_PANEL_READY = 0,
    APP_H2_PANEL_WAIT,
    APP_H2_PANEL_RUNNING,
    APP_H2_PANEL_FINISHED,
    APP_H2_PANEL_STOPPED
} AppH2PanelDisplay_t;

typedef enum {
    APP_H2_START_READY = 0,
    APP_H2_START_FAULT,
    APP_H2_START_NOT_IDLE,
    APP_H2_START_CALIBRATION,
    APP_H2_START_MOVING,
    APP_H2_START_LINE_INVALID,
    APP_H2_START_NOT_CENTERED,
    APP_H2_START_GYRO_INVALID,
    APP_H2_START_REJECTED
} AppH2StartStatus_t;

typedef struct {
    AppH2PanelDisplay_t display;
    AppH2StartStatus_t start_status;
    uint32_t key_presses;
    uint32_t starts;
    uint32_t rejects;
    uint32_t displayed_elapsed_ms;
    bool debounced_pressed;
    bool release_armed;
} AppH2PanelSnapshot_t;

void AppH2Panel_Init(uint32_t now_ms);
void AppH2Panel_Task20ms(uint32_t now_ms);
void AppH2Panel_Task100ms(uint32_t now_ms);
void AppH2Panel_GetSnapshot(AppH2PanelSnapshot_t *snapshot);

#endif
