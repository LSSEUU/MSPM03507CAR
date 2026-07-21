#include "oled.h"

#define OLED_ADDRESS_PRIMARY       0x3CU
#define OLED_ADDRESS_SECONDARY     0x3DU
#define OLED_I2C_DELAY_CYCLES      80U

static uint8_t OLED_Buffer[OLED_WIDTH * OLED_HEIGHT / 8];
static OLED_Stats_t oled_stats;
static uint8_t oled_address = OLED_ADDRESS_PRIMARY;

static void OLED_I2CDelay(void) { delay_cycles(OLED_I2C_DELAY_CYCLES); }

static void OLED_SDALow(void)
{
    DL_GPIO_clearPins(OLED_SDA_PORT, OLED_SDA_PIN);
    DL_GPIO_enableOutput(OLED_SDA_PORT, OLED_SDA_PIN);
}

static void OLED_SDARelease(void) { DL_GPIO_disableOutput(OLED_SDA_PORT, OLED_SDA_PIN); }

static void OLED_SCLLow(void)
{
    DL_GPIO_clearPins(OLED_SCL_PORT, OLED_SCL_PIN);
    DL_GPIO_enableOutput(OLED_SCL_PORT, OLED_SCL_PIN);
}

static void OLED_SCLRelease(void) { DL_GPIO_disableOutput(OLED_SCL_PORT, OLED_SCL_PIN); }

static void OLED_I2CStart(void)
{
    OLED_SDARelease(); OLED_SCLRelease(); OLED_I2CDelay();
    OLED_SDALow(); OLED_I2CDelay(); OLED_SCLLow();
}

static void OLED_I2CStop(void)
{
    OLED_SDALow(); OLED_I2CDelay(); OLED_SCLRelease(); OLED_I2CDelay();
    OLED_SDARelease(); OLED_I2CDelay();
}

static bool OLED_I2CWriteByte(uint8_t value)
{
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) OLED_SDARelease(); else OLED_SDALow();
        OLED_I2CDelay(); OLED_SCLRelease(); OLED_I2CDelay(); OLED_SCLLow();
        value <<= 1U;
    }
    OLED_SDARelease(); OLED_I2CDelay(); OLED_SCLRelease(); OLED_I2CDelay();
    bool ack = (DL_GPIO_readPins(OLED_SDA_PORT, OLED_SDA_PIN) == 0U);
    OLED_SCLLow();
    return ack;
}

static bool OLED_WritePacket(uint8_t control, const uint8_t *data, uint16_t length)
{
    bool ok;
    OLED_I2CStart();
    ok = OLED_I2CWriteByte((uint8_t)(oled_address << 1U));
    if (ok) ok = OLED_I2CWriteByte(control);
    for (uint16_t i = 0U; ok && i < length; i++) {
        ok = OLED_I2CWriteByte(data[i]);
        if (ok) oled_stats.tx_bytes++;
    }
    OLED_I2CStop();
    if (!ok) oled_stats.ack_errors++;
    return ok;
}

static bool OLED_Probe(uint8_t address)
{
    bool ack;
    oled_address = address;
    OLED_I2CStart();
    ack = OLED_I2CWriteByte((uint8_t)(address << 1U));
    OLED_I2CStop();
    return ack;
}

static void OLED_WriteCmd(uint8_t cmd)
{
    if (oled_stats.present) (void)OLED_WritePacket(0x00U, &cmd, 1U);
}

void OLED_Init(void)
{
    static const uint8_t init_commands[] = {
        0xAEU,0x20U,0x02U,0xB0U,0xC8U,0x00U,0x10U,0x40U,
        0x81U,0x7FU,0xA1U,0xA6U,0xA8U,0x3FU,0xA4U,0xD3U,
        0x00U,0xD5U,0x80U,0xD9U,0xF1U,0xDAU,0x12U,0xDBU,
        0x40U,0x8DU,0x14U,0xAFU
    };
    memset(&oled_stats, 0, sizeof(oled_stats));
    /* Input enable remains active while DOE is toggled for open-drain bit-bang. */
    DL_GPIO_initDigitalInput(OLED_SDA_IOMUX);
    DL_GPIO_initDigitalInput(OLED_SCL_IOMUX);
    DL_GPIO_clearPins(OLED_SDA_PORT, OLED_SDA_PIN | OLED_SCL_PIN);
    OLED_SDARelease(); OLED_SCLRelease();
    oled_stats.present = OLED_Probe(OLED_ADDRESS_PRIMARY);
    if (!oled_stats.present) oled_stats.present = OLED_Probe(OLED_ADDRESS_SECONDARY);
    if (!oled_stats.present) {
        oled_stats.ack_errors++;
        OLED_Clear();
        return;
    }
    if (!OLED_WritePacket(0x00U, init_commands, sizeof(init_commands))) {
        oled_stats.present = false;
        OLED_Clear();
        return;
    }
    OLED_Clear();
    OLED_Refresh();
}

void OLED_Clear(void) { memset(OLED_Buffer, 0, sizeof(OLED_Buffer)); }

void OLED_Refresh(void)
{
    if (!oled_stats.present) return;
    for (uint8_t page = 0U; page < OLED_PAGES; page++) {
        uint8_t position[] = {(uint8_t)(0xB0U + page), 0x00U, 0x10U};
        (void)OLED_WritePacket(0x00U, position, sizeof(position));
        (void)OLED_WritePacket(0x40U, &OLED_Buffer[(uint16_t)page * OLED_WIDTH], OLED_WIDTH);
    }
    oled_stats.refresh_count++;
}

void OLED_SetPos(uint8_t x, uint8_t y)
{
    if (x >= OLED_WIDTH || y >= OLED_PAGES) return;
    OLED_WriteCmd((uint8_t)(0xB0U + y));
    OLED_WriteCmd((uint8_t)(((x & 0xF0U) >> 4U) | 0x10U));
    OLED_WriteCmd((uint8_t)(x & 0x0FU));
}

void OLED_DrawPoint(uint8_t x, uint8_t y)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    OLED_Buffer[x + (uint16_t)(y / 8U) * OLED_WIDTH] |= (uint8_t)(1U << (y % 8U));
}

static const uint8_t *OLED_GetGlyph(char c)
{
    static const uint8_t digits[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E}
    };
    static const uint8_t letters[26][5] = {
        {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
        {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
        {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
        {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},
        {0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
        {0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
        {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
        {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
        {0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
    };
    static const uint8_t blank[5] = {0,0,0,0,0};
    static const uint8_t colon[5] = {0,0x36,0x36,0,0};
    static const uint8_t dash[5] = {0x08,0x08,0x08,0x08,0x08};
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    if (c >= '0' && c <= '9') return digits[(uint8_t)(c - '0')];
    if (c >= 'A' && c <= 'Z') return letters[(uint8_t)(c - 'A')];
    if (c == ':') return colon;
    if (c == '-') return dash;
    return blank;
}

void OLED_ShowChar(uint8_t x, uint8_t y, char c)
{
    const uint8_t *glyph;
    if (x > (OLED_WIDTH - 5U) || y >= OLED_PAGES) return;
    glyph = OLED_GetGlyph(c);
    for (uint8_t i = 0U; i < 5U; i++) OLED_Buffer[(uint16_t)y * OLED_WIDTH + x + i] = glyph[i];
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *s)
{
    while (s != NULL && *s != '\0' && x <= (OLED_WIDTH - 5U)) {
        OLED_ShowChar(x, y, *s++);
        x = (uint8_t)(x + 6U);
    }
}

void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len)
{
    char buf[12];
    (void)snprintf(buf, sizeof(buf), "%*d", len, (int)num);
    OLED_ShowString(x, y, buf);
}

void OLED_ShowTestPattern(void)
{
    for (uint16_t i = 0U; i < sizeof(OLED_Buffer); i++) OLED_Buffer[i] = ((i / 16U) & 1U) ? 0xAAU : 0x55U;
    OLED_Refresh();
}

bool OLED_IsPresent(void) { return oled_stats.present; }

void OLED_GetStats(OLED_Stats_t *stats)
{
    if (stats != NULL) *stats = oled_stats;
}
