#include "key.h"

#include "board.h"

void Key_Init(void)
{
    /* KEY1 has an external pull-down on the carrier; do not add an internal
     * pull resistor that would change the verified input network. */
    DL_GPIO_initDigitalInput(KEY1_IOMUX);
}

bool Key1_IsPressed(void)
{
    return (DL_GPIO_readPins(KEY1_PORT, KEY1_PIN) != 0U);
}
