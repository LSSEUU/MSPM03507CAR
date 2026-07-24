#include "ir_nec.h"

#include <string.h>

/* TIMG0 is clocked at 4 MHz: one tick is 0.25 us. */
#define IR_NEC_LEADER_MARK_MIN_TICKS      32000U
#define IR_NEC_LEADER_MARK_MAX_TICKS      40000U
#define IR_NEC_LEADER_SPACE_MIN_TICKS     16000U
#define IR_NEC_LEADER_SPACE_MAX_TICKS     20000U
#define IR_NEC_REPEAT_SPACE_MIN_TICKS      8000U
#define IR_NEC_REPEAT_SPACE_MAX_TICKS     10000U
#define IR_NEC_BIT_MARK_MIN_TICKS          1600U
#define IR_NEC_BIT_MARK_MAX_TICKS          2800U
#define IR_NEC_ZERO_SPACE_MIN_TICKS        1600U
#define IR_NEC_ZERO_SPACE_MAX_TICKS        3200U
#define IR_NEC_ONE_SPACE_MIN_TICKS         5200U
#define IR_NEC_ONE_SPACE_MAX_TICKS         7600U
#define IR_NEC_REPEAT_MAX_AGE_MS             180U

enum {
    IR_NEC_STATE_IDLE = 0,
    IR_NEC_STATE_LEADER_MARK,
    IR_NEC_STATE_LEADER_SPACE,
    IR_NEC_STATE_BIT_MARK,
    IR_NEC_STATE_BIT_SPACE,
    IR_NEC_STATE_FINAL_MARK,
    IR_NEC_STATE_REPEAT_MARK
};

static bool IrNec_InWindow(uint16_t ticks, uint16_t minimum,
                           uint16_t maximum)
{
    return (ticks >= minimum) && (ticks <= maximum);
}

static void IrNec_RestartOnFallingEdge(IrNecDecoder_t *decoder,
                                       bool new_level_high)
{
    decoder->state = new_level_high ? IR_NEC_STATE_IDLE :
                                      IR_NEC_STATE_LEADER_MARK;
    decoder->bit_index = 0U;
    decoder->raw = 0U;
}

static bool IrNec_TimingError(IrNecDecoder_t *decoder, bool new_level_high)
{
    decoder->stats.timing_errors++;
    IrNec_RestartOnFallingEdge(decoder, new_level_high);
    return false;
}

void IrNec_Init(IrNecDecoder_t *decoder)
{
    if (decoder == 0) return;
    memset(decoder, 0, sizeof(*decoder));
}

bool IrNec_ProcessEdge(IrNecDecoder_t *decoder, uint16_t elapsed_ticks,
                       bool new_level_high, uint32_t now_ms,
                       IrNecEvent_t *event)
{
    uint8_t address;
    uint8_t address_inverse;
    uint8_t command;
    uint8_t command_inverse;

    if ((decoder == 0) || (event == 0)) return false;
    memset(event, 0, sizeof(*event));

    if (decoder->state == IR_NEC_STATE_IDLE) {
        if (!new_level_high) decoder->state = IR_NEC_STATE_LEADER_MARK;
        return false;
    }

    if (decoder->state == IR_NEC_STATE_LEADER_MARK) {
        if (!new_level_high ||
            !IrNec_InWindow(elapsed_ticks,
                IR_NEC_LEADER_MARK_MIN_TICKS,
                IR_NEC_LEADER_MARK_MAX_TICKS))
            return IrNec_TimingError(decoder, new_level_high);
        decoder->state = IR_NEC_STATE_LEADER_SPACE;
        return false;
    }

    if (decoder->state == IR_NEC_STATE_LEADER_SPACE) {
        if (new_level_high)
            return IrNec_TimingError(decoder, new_level_high);
        if (IrNec_InWindow(elapsed_ticks,
                IR_NEC_LEADER_SPACE_MIN_TICKS,
                IR_NEC_LEADER_SPACE_MAX_TICKS)) {
            decoder->state = IR_NEC_STATE_BIT_MARK;
            decoder->bit_index = 0U;
            decoder->raw = 0U;
            return false;
        }
        if (IrNec_InWindow(elapsed_ticks,
                IR_NEC_REPEAT_SPACE_MIN_TICKS,
                IR_NEC_REPEAT_SPACE_MAX_TICKS)) {
            decoder->state = IR_NEC_STATE_REPEAT_MARK;
            return false;
        }
        return IrNec_TimingError(decoder, new_level_high);
    }

    if ((decoder->state == IR_NEC_STATE_BIT_MARK) ||
        (decoder->state == IR_NEC_STATE_FINAL_MARK) ||
        (decoder->state == IR_NEC_STATE_REPEAT_MARK)) {
        uint8_t completed_state = decoder->state;
        if (!new_level_high ||
            !IrNec_InWindow(elapsed_ticks,
                IR_NEC_BIT_MARK_MIN_TICKS,
                IR_NEC_BIT_MARK_MAX_TICKS))
            return IrNec_TimingError(decoder, new_level_high);

        if (completed_state == IR_NEC_STATE_BIT_MARK) {
            decoder->state = IR_NEC_STATE_BIT_SPACE;
            return false;
        }
        decoder->state = IR_NEC_STATE_IDLE;
        if (completed_state == IR_NEC_STATE_REPEAT_MARK) {
            if (!decoder->stats.last_full_valid ||
                ((uint32_t)(now_ms - decoder->repeat_reference_ms) >
                    IR_NEC_REPEAT_MAX_AGE_MS))
                return false;
            event->type = IR_NEC_EVENT_REPEAT;
            event->address = decoder->stats.last_address;
            event->command = decoder->stats.last_command;
            event->raw = decoder->stats.last_raw;
            event->timestamp_ms = now_ms;
            decoder->repeat_reference_ms = now_ms;
            decoder->stats.repeat_frames++;
            return true;
        }

        address = (uint8_t)(decoder->raw & 0xFFU);
        address_inverse = (uint8_t)((decoder->raw >> 8U) & 0xFFU);
        command = (uint8_t)((decoder->raw >> 16U) & 0xFFU);
        command_inverse = (uint8_t)((decoder->raw >> 24U) & 0xFFU);
        if (((uint8_t)(address ^ address_inverse) != 0xFFU) ||
            ((uint8_t)(command ^ command_inverse) != 0xFFU)) {
            decoder->stats.checksum_errors++;
            return false;
        }
        event->type = IR_NEC_EVENT_FULL;
        event->address = address;
        event->command = command;
        event->raw = decoder->raw;
        event->timestamp_ms = now_ms;
        decoder->stats.full_frames++;
        decoder->stats.last_address = address;
        decoder->stats.last_command = command;
        decoder->stats.last_raw = decoder->raw;
        decoder->stats.last_full_ms = now_ms;
        decoder->stats.last_full_valid = true;
        decoder->repeat_reference_ms = now_ms;
        return true;
    }

    if (decoder->state == IR_NEC_STATE_BIT_SPACE) {
        bool one;
        if (new_level_high)
            return IrNec_TimingError(decoder, new_level_high);
        if (IrNec_InWindow(elapsed_ticks,
                IR_NEC_ZERO_SPACE_MIN_TICKS,
                IR_NEC_ZERO_SPACE_MAX_TICKS)) one = false;
        else if (IrNec_InWindow(elapsed_ticks,
                IR_NEC_ONE_SPACE_MIN_TICKS,
                IR_NEC_ONE_SPACE_MAX_TICKS)) one = true;
        else return IrNec_TimingError(decoder, new_level_high);

        if (one) decoder->raw |= (1UL << decoder->bit_index);
        decoder->bit_index++;
        decoder->state = (decoder->bit_index == 32U) ?
            IR_NEC_STATE_FINAL_MARK : IR_NEC_STATE_BIT_MARK;
        return false;
    }

    return IrNec_TimingError(decoder, new_level_high);
}

void IrNec_Timeout(IrNecDecoder_t *decoder)
{
    if (decoder == 0) return;
    if (decoder->state != IR_NEC_STATE_IDLE) {
        decoder->stats.timing_errors++;
        decoder->state = IR_NEC_STATE_IDLE;
        decoder->bit_index = 0U;
        decoder->raw = 0U;
    }
}

void IrNec_GetStats(const IrNecDecoder_t *decoder, IrNecStats_t *stats)
{
    if ((decoder == 0) || (stats == 0)) return;
    *stats = decoder->stats;
}
