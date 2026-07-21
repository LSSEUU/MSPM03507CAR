#include "board.h"
#include "uart_debug.h"
#include "motor.h"
#include "encoder.h"
#include "gyro.h"
#include "grayscale.h"
#include "oled.h"
#include "led.h"
#include "diagnostics.h"

int main(void)
{
    SYSCFG_DL_init();
    DL_SYSTICK_init(SYSTICK_MAX_COUNT);
    DL_SYSTICK_enable();

    Motor_Init();
    Encoder_Init();
    Grayscale_Init();
    Gyro_Init();
    LED_Init();
    OLED_Init();

    Diagnostics_Init();
    Diagnostics_RunBootSelfTest();

    while (1) {
        Diagnostics_Poll();
        delay_ms(5U);
    }
}
