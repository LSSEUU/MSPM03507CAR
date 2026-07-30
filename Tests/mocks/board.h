#ifndef TEST_BOARD_H
#define TEST_BOARD_H

#include <stdint.h>

typedef struct {
    uint32_t unused;
} GPIO_Regs;

extern GPIO_Regs mock_gpioa;

#define GPIOA                       (&mock_gpioa)
#define DL_GPIO_PIN_24              (1UL << 24)
#define DL_GPIO_PIN_26              (1UL << 26)
#define IOMUX_PINCM54               54U
#define IOMUX_PINCM59               59U

#define GRAY_I2C_SDA_PORT           GPIOA
#define GRAY_I2C_SDA_PIN            DL_GPIO_PIN_24
#define GRAY_I2C_SDA_IOMUX          IOMUX_PINCM54
#define GRAY_I2C_SCL_PORT           GPIOA
#define GRAY_I2C_SCL_PIN            DL_GPIO_PIN_26
#define GRAY_I2C_SCL_IOMUX          IOMUX_PINCM59

void delay_cycles(uint32_t cycles);
void DL_GPIO_initDigitalInput(uint32_t iomux);
void DL_GPIO_clearPins(GPIO_Regs *port, uint32_t pins);
void DL_GPIO_enableOutput(GPIO_Regs *port, uint32_t pins);
void DL_GPIO_disableOutput(GPIO_Regs *port, uint32_t pins);
uint32_t DL_GPIO_readPins(GPIO_Regs *port, uint32_t pins);

#endif
