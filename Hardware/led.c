
#include "led.h"


#define LED_MAX   4

typedef struct {
    uint32_t  pincmIndex;    
    GPIO_Regs *port;         
    uint32_t  pin;           
} LedPin_t;

static const LedPin_t leds[LED_MAX] = {
    
    
    
};


void LED_Init(void) {
    for (uint8_t i = 0; i < LED_MAX; i++) {
        if (leds[i].port == NULL) continue;
        
        DL_GPIO_initPeripheralOutputFunction(leds[i].pincmIndex, 0x00000001);
        
        leds[i].port->DOUT31_0 |= leds[i].pin;
    }
}


void LED_On(uint8_t index) {
    if (index >= LED_MAX || leds[index].port == NULL) return;
    leds[index].port->DOUT31_0 &= ~leds[index].pin;
}


void LED_Off(uint8_t index) {
    if (index >= LED_MAX || leds[index].port == NULL) return;
    leds[index].port->DOUT31_0 |= leds[index].pin;
}


void LED_Toggle(uint8_t index) {
    if (index >= LED_MAX || leds[index].port == NULL) return;
    leds[index].port->DOUT31_0 ^= leds[index].pin;
}


void LED_Flash(uint16_t period_ms) {
    static uint16_t tick = 0;
    tick++;
    if (tick >= period_ms) {
        tick = 0;
        LED_Toggle(0);
    }
}
