#include "buzzer.h"

#include "board.h"

void Buzzer_Init(void)
{
    /* Preload the inactive level before enabling the output to avoid a chirp
     * while the PA12 pad changes from its reset state to push-pull GPIO. */
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
    DL_GPIO_initDigitalOutput(BUZZER_IOMUX);
    DL_GPIO_enableOutput(BUZZER_PORT, BUZZER_PIN);
}

void Buzzer_On(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
}

void Buzzer_Off(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
}
