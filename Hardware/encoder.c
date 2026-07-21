
#include "encoder.h"
#include "board.h"

volatile int32_t EncoderA_Count = 0;
volatile int32_t EncoderB_Count = 0;
static volatile uint32_t encoder_edges_a = 0;
static volatile uint32_t encoder_edges_b = 0;
static uint32_t encoder_errors = 0;


void GROUP1_IRQHandler(void)
{
    uint32_t status;

    
    status = DL_GPIO_getEnabledInterruptStatus(ENC_A_PORT,
        ENC_A_E1A_PIN | ENC_A_E1B_PIN);
    if (status & ENC_A_E1A_PIN) {
        encoder_edges_a++;
        if (DL_GPIO_readPins(ENC_A_PORT, ENC_A_E1B_PIN))
            EncoderA_Count++;
        else
            EncoderA_Count--;
    }
    if (status & ENC_A_E1B_PIN) {
        encoder_edges_a++;
        if (DL_GPIO_readPins(ENC_A_PORT, ENC_A_E1A_PIN))
            EncoderA_Count--;
        else
            EncoderA_Count++;
    }
    DL_GPIO_clearInterruptStatus(ENC_A_PORT,
        ENC_A_E1A_PIN | ENC_A_E1B_PIN);

    
    status = DL_GPIO_getEnabledInterruptStatus(ENC_B_PORT,
        ENC_B_E2A_PIN | ENC_B_E2B_PIN);
    if (status & ENC_B_E2A_PIN) {
        encoder_edges_b++;
        if (DL_GPIO_readPins(ENC_B_PORT, ENC_B_E2B_PIN))
            EncoderB_Count++;
        else
            EncoderB_Count--;
    }
    if (status & ENC_B_E2B_PIN) {
        encoder_edges_b++;
        if (DL_GPIO_readPins(ENC_B_PORT, ENC_B_E2A_PIN))
            EncoderB_Count--;
        else
            EncoderB_Count++;
    }
    DL_GPIO_clearInterruptStatus(ENC_B_PORT,
        ENC_B_E2A_PIN | ENC_B_E2B_PIN);
}

void Encoder_Init(void)
{
    EncoderA_Count = 0;
    EncoderB_Count = 0;
    encoder_edges_a = 0;
    encoder_edges_b = 0;
    encoder_errors = 0;
}

void Encoder_Update(void)
{
    
}

int32_t Encoder_GetCount(uint8_t channel)
{
    if (channel == 0) return EncoderA_Count;
    if (channel == 1) return EncoderB_Count;
    encoder_errors++;
    return 0;
}

void Encoder_ClearCount(uint8_t channel)
{
    if (channel == 0) EncoderA_Count = 0;
    else if (channel == 1) EncoderB_Count = 0;
    else encoder_errors++;
}

int16_t Encoder_GetDelta(uint8_t channel)
{
    static int32_t last_a = 0, last_b = 0;
    int32_t cur, delta;

    if (channel == 0) {
        cur = EncoderA_Count;
        delta = cur - last_a;
        last_a = cur;
    } else if (channel == 1) {
        cur = EncoderB_Count;
        delta = cur - last_b;
        last_b = cur;
    } else { encoder_errors++; return 0; }
    return (int16_t)delta;
}

void Encoder_GetSnapshot(EncoderSnapshot_t *snapshot)
{
    uint32_t primask;
    if (snapshot == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->count_a = EncoderA_Count;
    snapshot->count_b = EncoderB_Count;
    snapshot->edge_count_a = encoder_edges_a;
    snapshot->edge_count_b = encoder_edges_b;
    if (primask == 0U) __enable_irq();
    snapshot->error_count = encoder_errors;
}
