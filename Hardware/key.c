#include "key.h"

#include "board.h"

void Key_Init(void)
{
    /* All three keys have external pull-downs on the carrier; do not add an internal
     * pull resistor that would change the verified input network. */
    DL_GPIO_initDigitalInput(KEY1_IOMUX);
    DL_GPIO_initDigitalInput(KEY2_IOMUX);
    DL_GPIO_initDigitalInput(KEY3_IOMUX);
}

bool Key2_IsPressed(void)
{
    return (DL_GPIO_readPins(KEY2_PORT, KEY2_PIN) != 0U);
}

bool Key3_IsPressed(void)
{
    return (DL_GPIO_readPins(KEY3_PORT, KEY3_PIN) != 0U);
}

bool Key1_IsPressed(void)
{
    return (DL_GPIO_readPins(KEY1_PORT, KEY1_PIN) != 0U);
}
