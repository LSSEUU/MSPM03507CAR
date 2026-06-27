
#include "show.h"
#include "oled.h"
#include "motor.h"
#include "control.h"

void Show_Init(void) {
    OLED_Init();
}


void oled_show(void) {
    
    OLED_Clear();
    OLED_ShowString(0, 0, "MSPM0G3507");
    OLED_ShowString(0, 2, "Vel:");
    OLED_ShowNum(40, 2, (int32_t)Target_Velocity, 5);
    OLED_Refresh();
}


void APP_Show(void) {
    
}
