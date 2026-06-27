
#include "encoder.h"

volatile int32_t EncoderA_Count = 0;
volatile int32_t EncoderB_Count = 0;


void GROUP1_IRQHandler(void)
{
    uint32_t status;

    
    status = DL_GPIO_getEnabledInterruptStatus(ENC_A_PORT,
        ENC_A_E1A_PIN | ENC_A_E1B_PIN);
    if (status & ENC_A_E1A_PIN) {
        if (DL_GPIO_readPins(ENC_A_PORT, ENC_A_E1B_PIN))
            EncoderA_Count++;
        else
            EncoderA_Count--;
    }
    if (status & ENC_A_E1B_PIN) {
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
        if (DL_GPIO_readPins(ENC_B_PORT, ENC_B_E2B_PIN))
            EncoderB_Count++;
        else
            EncoderB_Count--;
    }
    if (status & ENC_B_E2B_PIN) {
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
}

void Encoder_Update(void)
{
    
}

int32_t Encoder_GetCount(uint8_t channel)
{
    if (channel == 0) return EncoderA_Count;
    return EncoderB_Count;
}

void Encoder_ClearCount(uint8_t channel)
{
    if (channel == 0) EncoderA_Count = 0;
    else             EncoderB_Count = 0;
}

int16_t Encoder_GetDelta(uint8_t channel)
{
    static int32_t last_a = 0, last_b = 0;
    int32_t cur, delta;

    if (channel == 0) {
        cur = EncoderA_Count;
        delta = cur - last_a;
        last_a = cur;
    } else {
        cur = EncoderB_Count;
        delta = cur - last_b;
        last_b = cur;
    }
    return (int16_t)delta;
}
