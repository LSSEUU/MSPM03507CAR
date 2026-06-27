
#include "oled.h"


static uint8_t OLED_Buffer[OLED_WIDTH * OLED_HEIGHT / 8];


static void OLED_WriteCmd(uint8_t cmd) {
    
    (void)cmd;
}

static void OLED_WriteData(uint8_t data) {
    
    (void)data;
}


void OLED_Init(void) {
    
    OLED_Clear();
    OLED_Refresh();
}

void OLED_Clear(void) {
    for (uint32_t i = 0; i < sizeof(OLED_Buffer); i++)
        OLED_Buffer[i] = 0x00;
}

void OLED_Refresh(void) {
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        OLED_WriteCmd(0xB0 + page);   
        OLED_WriteCmd(0x00);           
        OLED_WriteCmd(0x10);           
        for (uint8_t col = 0; col < OLED_WIDTH; col++)
            OLED_WriteData(OLED_Buffer[page * OLED_WIDTH + col]);
    }
}

void OLED_SetPos(uint8_t x, uint8_t y) {
    OLED_WriteCmd(0xB0 + y);
    OLED_WriteCmd(((x & 0xF0) >> 4) | 0x10);
    OLED_WriteCmd(x & 0x0F);
}

void OLED_DrawPoint(uint8_t x, uint8_t y) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    OLED_Buffer[x + (y / 8) * OLED_WIDTH] |= (1 << (y % 8));
}

void OLED_ShowChar(uint8_t x, uint8_t y, char c) {
    
    (void)x; (void)y; (void)c;
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *s) {
    while (*s) OLED_ShowChar(x, y, *s++);
}

void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%*d", len, (int)num);
    OLED_ShowString(x, y, buf);
}
