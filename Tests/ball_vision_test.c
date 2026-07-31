#include "ball_vision.h"

#include "m6_vision_uart.h"
#include "vision_uart.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static VisionUartStats_t uart_stats;
static uint8_t uart_rx[256];
static size_t uart_rx_length;
static uint32_t mirrored_bytes;

static uint8_t crc8_atm(const uint8_t *data, size_t length)
{
    uint8_t crc = 0U;
    size_t index;
    for (index = 0U; index < length; index++) {
        uint8_t bit;
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc & 0x80U) != 0U ?
                (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void frame(uint8_t output[9], uint8_t sequence, uint8_t flags,
                  int16_t x_mm_x10, int16_t vx_mm_s)
{
    output[0] = 0xA5U;
    output[1] = 0x5AU;
    output[2] = sequence;
    output[3] = flags;
    output[4] = (uint8_t)x_mm_x10;
    output[5] = (uint8_t)((uint16_t)x_mm_x10 >> 8);
    output[6] = (uint8_t)vx_mm_s;
    output[7] = (uint8_t)((uint16_t)vx_mm_s >> 8);
    output[8] = crc8_atm(output, 8U);
}

static BallVisionSnapshot_t snapshot(uint32_t now_ms)
{
    BallVisionSnapshot_t value;
    BallVision_GetSnapshot(now_ms, &value);
    return value;
}

size_t VisionUart_Read(uint8_t *data, size_t capacity)
{
    size_t length = uart_rx_length < capacity ? uart_rx_length : capacity;
    memcpy(data, uart_rx, length);
    memmove(uart_rx, uart_rx + length, uart_rx_length - length);
    uart_rx_length -= length;
    uart_stats.rx_queued = (uint16_t)uart_rx_length;
    return length;
}

void VisionUart_GetStats(VisionUartStats_t *stats)
{
    *stats = uart_stats;
}

void M6VisionUart_MirrorRx(const uint8_t *data, size_t length)
{
    (void)data;
    mirrored_bytes += (uint32_t)length;
}

static void reset(void)
{
    memset(&uart_stats, 0, sizeof(uart_stats));
    uart_rx_length = 0U;
    mirrored_bytes = 0U;
    BallVision_Init(0U);
}

static void test_frozen_vectors_fragment_concat_and_wrap(void)
{
    static const uint8_t golden[] = {
        0xA5U, 0x5AU, 0x2AU, 0x01U, 0xF4U,
        0x01U, 0xECU, 0xFFU, 0x73U
    };
    uint8_t next[9];
    uint8_t wrap[9];
    uint8_t joined[18];
    BallVisionSnapshot_t value;
    reset();
    BallVision_ProcessBytes(golden, 3U, 10U);
    BallVision_ProcessBytes(golden + 3U, 6U, 10U);
    value = snapshot(10U);
    assert(value.valid_frames == 1U && !value.valid && !value.fresh);
    assert(value.x_mm_x10 == 500 && value.vx_mm_s == -20);
    frame(next, 43U, 1U, 510, -10);
    BallVision_ProcessBytes(next, sizeof(next), 20U);
    value = snapshot(20U);
    assert(value.valid && value.fresh && value.seq == 43U);
    assert(value.received_ms == 20U && value.age_ms == 0U);

    frame(next, 255U, 1U, 100, 0);
    frame(wrap, 0U, 1U, 110, 1);
    memcpy(joined, next, 9U);
    memcpy(joined + 9U, wrap, 9U);
    BallVision_ProcessBytes(joined, sizeof(joined), 30U);
    value = snapshot(30U);
    assert(value.seq == 0U && value.valid && value.sequence_gaps != 0U);
}

static void test_noise_bad_crc_embedded_header_and_duplicate(void)
{
    uint8_t good1[9];
    uint8_t good2[9];
    uint8_t stream[20];
    BallVisionSnapshot_t value;
    reset();
    frame(good1, 1U, 1U, 10, 0);
    frame(good2, 2U, 1U, 20, 0);
    /* A corrupt candidate ends with the first five bytes of the next frame;
     * the parser must retain the embedded A5 5A and finish it. */
    stream[0] = 0x00U;
    stream[1] = 0xA5U;
    stream[2] = 0x5AU;
    stream[3] = 0x99U;
    stream[4] = 0x01U;
    memcpy(&stream[5], good1, 9U);
    stream[13] ^= 0x01U;
    memcpy(&stream[14], good2, 6U);
    BallVision_ProcessBytes(stream, sizeof(stream), 10U);
    BallVision_ProcessBytes(good2 + 6U, 3U, 10U);
    value = snapshot(10U);
    assert(value.crc_errors >= 1U);
    assert(value.valid_frames == 1U && value.seq == 2U);

    BallVision_ProcessBytes(good2, sizeof(good2), 100U);
    value = snapshot(100U);
    assert(value.duplicate_frames == 1U);
    assert(value.received_ms == 10U);
}

static void test_invalid_malformed_timeout_and_transport_fault(void)
{
    uint8_t valid[9];
    uint8_t invalid[9];
    uint8_t corrupt[9];
    BallVisionSnapshot_t value;
    reset();
    frame(valid, 10U, 1U, 0, 0);
    BallVision_ProcessBytes(valid, 9U, 10U);
    frame(valid, 11U, 1U, 0, 0);
    BallVision_ProcessBytes(valid, 9U, 20U);
    assert(snapshot(20U).fresh);

    frame(invalid, 12U, 0U, 0, 0);
    BallVision_ProcessBytes(invalid, 9U, 30U);
    value = snapshot(30U);
    assert(!value.valid && !value.fresh);
    assert(value.explicit_invalid_frames == 1U);

    frame(corrupt, 13U, 2U, 0, 0);
    BallVision_ProcessBytes(corrupt, 9U, 40U);
    frame(corrupt, 13U, 0U, 1, 0);
    BallVision_ProcessBytes(corrupt, 9U, 40U);
    frame(corrupt, 13U, 1U, BALL_VISION_POSITION_LIMIT_X10 + 1, 0);
    BallVision_ProcessBytes(corrupt, 9U, 40U);
    assert(snapshot(40U).malformed_frames >= 3U);

    frame(valid, 13U, 1U, 30, 0);
    BallVision_ProcessBytes(valid, 9U, 50U);
    frame(valid, 14U, 1U, 30, 0);
    BallVision_ProcessBytes(valid, 9U, 60U);
    assert(snapshot(60U).fresh);
    assert(!snapshot(211U).fresh);

    memcpy(uart_rx, valid, 9U);
    uart_rx_length = 9U;
    uart_stats.rx_queued = 9U;
    BallVision_Service(220U);
    assert(mirrored_bytes == 9U);
    uart_stats.rx_overflow++;
    BallVision_Service(221U);
    assert(!snapshot(221U).fresh);
    uart_stats.hardware_errors++;
    BallVision_Service(222U);
    assert(!snapshot(222U).fresh);
}

int main(void)
{
    test_frozen_vectors_fragment_concat_and_wrap();
    test_noise_bad_crc_embedded_header_and_duplicate();
    test_invalid_malformed_timeout_and_transport_fault();
    return 0;
}
