#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>

#define OLED_WIDTH   128U
#define OLED_HEIGHT   64U
#define OLED_PAGES    (OLED_HEIGHT / 8U)

typedef struct {
    uint32_t tx_bytes;
    uint32_t ack_errors;
    uint32_t refresh_count;
    bool present;
} OLED_Stats_t;

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_RequestRefresh(void);
void OLED_Service(void);
void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_ShowChar(uint8_t x, uint8_t y, char c);
void OLED_ShowString(uint8_t x, uint8_t y, const char *s);
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len);
void OLED_DrawPoint(uint8_t x, uint8_t y);
void OLED_ShowTestPattern(void);
bool OLED_IsPresent(void);
void OLED_GetStats(OLED_Stats_t *stats);

#endif
