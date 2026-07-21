#include "led.h"

/*
 * The verified carrier exposes only power indicators, which are not driven by
 * the MCU.  Keep the diagnostic API explicit: zero configured LEDs is a
 * DEGRADED capability, not a fabricated successful test.
 */
static uint32_t led_error_count;

void LED_Init(void)
{
    led_error_count = 0U;
}

void LED_On(uint8_t index)
{
    (void)index;
    led_error_count++;
}

void LED_Off(uint8_t index)
{
    (void)index;
    led_error_count++;
}

void LED_Toggle(uint8_t index)
{
    (void)index;
    led_error_count++;
}

void LED_Flash(uint16_t period_ms)
{
    (void)period_ms;
}

uint8_t LED_GetConfiguredCount(void)
{
    return (uint8_t)BOARD_STATUS_LED_COUNT;
}

uint32_t LED_GetErrorCount(void)
{
    return led_error_count;
}
