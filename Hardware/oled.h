/*
 * oled.h  —  OLED 显示屏驱动 (I2C/SPI)
 * 参考: WHEELTEC_C07A_Bluetooth/Hardware/oled.h
 */
#ifndef OLED_H
#define OLED_H
#include "board.h"

/* ── 显示尺寸 ── */
#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_PAGES    (OLED_HEIGHT / 8)

typedef struct {
    uint32_t tx_bytes;
    uint32_t ack_errors;
    uint32_t refresh_count;
    bool present;
} OLED_Stats_t;

/* ── API ── */
void OLED_Init(void);                       /* 初始化 OLED */
void OLED_Clear(void);                      /* 清屏 */
void OLED_Refresh(void);                    /* 刷新显存到屏幕 */
void OLED_SetPos(uint8_t x, uint8_t y);      /* 设置光标位置 */
void OLED_ShowChar(uint8_t x, uint8_t y, char c);   /* 显示字符 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *s); /* 显示字符串 */
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len); /* 显示数字 */
void OLED_DrawPoint(uint8_t x, uint8_t y);   /* 画点 */
void OLED_ShowTestPattern(void);
bool OLED_IsPresent(void);
void OLED_GetStats(OLED_Stats_t *stats);

#endif
