#include "zdt_stepper_uart.h"
#include "ti_msp_dl_config.h"

#include <assert.h>
#include <string.h>

static uint8_t wire[128];
static unsigned wire_length;
static unsigned fifo_budget;
static bool tx_interrupt_pending;

bool DL_UART_isTXFIFOFull(void *instance)
{ (void)instance; return fifo_budget == 0U; }
bool DL_UART_isRXFIFOEmpty(void *instance)
{ (void)instance; return true; }
void DL_UART_transmitData(void *instance, uint8_t data)
{ (void)instance; assert(wire_length < sizeof(wire)); wire[wire_length++] = data; fifo_budget--; }
uint32_t DL_UART_receiveData(void *instance)
{ (void)instance; return 0U; }
void DL_UART_enableInterrupt(void *instance, uint32_t mask)
{ (void)instance; (void)mask; }
void DL_UART_disableInterrupt(void *instance, uint32_t mask)
{ (void)instance; (void)mask; }
uint32_t DL_UART_getPendingInterrupt(void *instance)
{
    (void)instance;
    if (tx_interrupt_pending) { tx_interrupt_pending = false; return DL_UART_IIDX_TX; }
    return DL_UART_IIDX_NO_INTERRUPT;
}

void UART3_IRQHandler(void);

int main(void)
{
    static const uint8_t active[] = {0x01U, 0xF6U, 0x00U, 0x6BU};
    static const uint8_t dropped[] = {0x01U, 0x35U, 0x6BU};
    static const uint8_t stop[] = {0x01U, 0xFEU, 0x98U, 0x00U, 0x6BU};
    static const uint8_t disable[] = {0x01U, 0xF3U, 0xABU, 0x00U, 0x00U, 0x6BU};
    const uint8_t *urgent[] = {stop, disable};
    const uint8_t lengths[] = {(uint8_t)sizeof(stop), (uint8_t)sizeof(disable)};
    ZdtStepperUartStats_t stats;
    uint8_t expected[sizeof(active) + sizeof(stop) + sizeof(disable)];

    memset(wire, 0, sizeof(wire)); wire_length = 0U; fifo_budget = 2U;
    ZdtStepperUart_Init();
    assert(ZdtStepperUart_Write(active, sizeof(active)));
    assert(wire_length == 2U);
    assert(ZdtStepperUart_Write(dropped, sizeof(dropped)));
    assert(ZdtStepperUart_WriteEmergencyBatch(urgent, lengths, 2U));
    ZdtStepperUart_GetStats(&stats);
    assert(stats.preempted_frames == 1U);

    fifo_budget = 64U; tx_interrupt_pending = true; UART3_IRQHandler();
    memcpy(expected, active, sizeof(active));
    memcpy(expected + sizeof(active), stop, sizeof(stop));
    memcpy(expected + sizeof(active) + sizeof(stop), disable, sizeof(disable));
    assert(wire_length == sizeof(expected));
    assert(memcmp(wire, expected, sizeof(expected)) == 0);
    ZdtStepperUart_GetStats(&stats);
    assert(stats.tx_queued == 0U);
    assert(stats.tx_bytes == sizeof(expected));
    return 0;
}
