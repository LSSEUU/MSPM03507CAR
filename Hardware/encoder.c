#include "encoder.h"
#include "board.h"

#include <stddef.h>

static volatile int32_t encoder_count_a = 0;
static volatile int32_t encoder_count_b = 0;
static volatile uint32_t encoder_edges_a = 0;
static volatile uint32_t encoder_edges_b = 0;

void GROUP1_IRQHandler(void)
{
    uint32_t status;

    status = DL_GPIO_getEnabledInterruptStatus(ENC_A_PORT,
        ENC_A_E1A_PIN | ENC_A_E1B_PIN);
    if (status & ENC_A_E1A_PIN) {
        encoder_edges_a++;
        if (DL_GPIO_readPins(ENC_A_PORT, ENC_A_E1B_PIN))
            encoder_count_a++;
        else
            encoder_count_a--;
    }
    if (status & ENC_A_E1B_PIN) {
        encoder_edges_a++;
        if (DL_GPIO_readPins(ENC_A_PORT, ENC_A_E1A_PIN))
            encoder_count_a--;
        else
            encoder_count_a++;
    }
    DL_GPIO_clearInterruptStatus(ENC_A_PORT,
        ENC_A_E1A_PIN | ENC_A_E1B_PIN);

    status = DL_GPIO_getEnabledInterruptStatus(ENC_B_PORT,
        ENC_B_E2A_PIN | ENC_B_E2B_PIN);
    if (status & ENC_B_E2A_PIN) {
        encoder_edges_b++;
        if (DL_GPIO_readPins(ENC_B_PORT, ENC_B_E2B_PIN))
            encoder_count_b++;
        else
            encoder_count_b--;
    }
    if (status & ENC_B_E2B_PIN) {
        encoder_edges_b++;
        if (DL_GPIO_readPins(ENC_B_PORT, ENC_B_E2A_PIN))
            encoder_count_b--;
        else
            encoder_count_b++;
    }
    DL_GPIO_clearInterruptStatus(ENC_B_PORT,
        ENC_B_E2A_PIN | ENC_B_E2B_PIN);
}

void Encoder_Init(void)
{
    encoder_count_a = 0;
    encoder_count_b = 0;
    encoder_edges_a = 0;
    encoder_edges_b = 0;
}

void Encoder_GetSnapshot(EncoderSnapshot_t *snapshot)
{
    uint32_t primask;
    if (snapshot == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->count_a = encoder_count_a;
    snapshot->count_b = encoder_count_b;
    snapshot->edge_count_a = encoder_edges_a;
    snapshot->edge_count_b = encoder_edges_b;
    if (primask == 0U) __enable_irq();
}
