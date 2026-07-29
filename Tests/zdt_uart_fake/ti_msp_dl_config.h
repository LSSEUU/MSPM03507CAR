#ifndef TI_MSP_DL_CONFIG_H
#define TI_MSP_DL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define UART_3_INST ((void *)1)
#define UART_3_INST_INT_IRQN 1
#define DL_UART_INTERRUPT_TX              (1U << 0)
#define DL_UART_INTERRUPT_RX              (1U << 1)
#define DL_UART_INTERRUPT_RX_TIMEOUT_ERROR (1U << 2)
#define DL_UART_INTERRUPT_OVERRUN_ERROR   (1U << 3)
#define DL_UART_INTERRUPT_FRAMING_ERROR   (1U << 4)
#define DL_UART_INTERRUPT_PARITY_ERROR    (1U << 5)
#define DL_UART_INTERRUPT_NOISE_ERROR     (1U << 6)
#define DL_UART_IIDX_NO_INTERRUPT         0U
#define DL_UART_IIDX_RX                   1U
#define DL_UART_IIDX_RX_TIMEOUT_ERROR     2U
#define DL_UART_IIDX_TX                   3U
#define DL_UART_IIDX_OVERRUN_ERROR        4U
#define DL_UART_IIDX_BREAK_ERROR          5U
#define DL_UART_IIDX_PARITY_ERROR         6U
#define DL_UART_IIDX_FRAMING_ERROR        7U
#define DL_UART_IIDX_NOISE_ERROR          8U

static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void NVIC_ClearPendingIRQ(int irq) { (void)irq; }
static inline void NVIC_EnableIRQ(int irq) { (void)irq; }

bool DL_UART_isTXFIFOFull(void *instance);
bool DL_UART_isRXFIFOEmpty(void *instance);
void DL_UART_transmitData(void *instance, uint8_t data);
uint32_t DL_UART_receiveData(void *instance);
void DL_UART_enableInterrupt(void *instance, uint32_t mask);
void DL_UART_disableInterrupt(void *instance, uint32_t mask);
uint32_t DL_UART_getPendingInterrupt(void *instance);

#endif
