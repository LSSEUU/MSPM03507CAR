#include "m6_vision_uart.h"

#include "m6_link.h"
#include "vision_uart.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static VisionUartStats_t uart_stats;
static bool publish_accept = true;
static unsigned publish_count;
static uint8_t published_payload[96];
static uint16_t published_length;
static M6TxPriority_t published_priority;
static uint8_t published_type;
static uint16_t published_topic;
static uint16_t published_flags[8];
static uint32_t published_bridge_dropped[8];

static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void VisionUart_GetStats(VisionUartStats_t *stats)
{
    *stats = uart_stats;
}

bool M6Link_Publish(M6TxPriority_t priority, uint8_t frame_type,
                    uint16_t topic, const uint8_t *payload,
                    uint16_t payload_length)
{
    if (!publish_accept) return false;
    assert(payload_length <= sizeof(published_payload));
    published_priority = priority;
    published_type = frame_type;
    published_topic = topic;
    published_length = payload_length;
    memcpy(published_payload, payload, payload_length);
    published_flags[publish_count] = u16(&payload[2]);
    published_bridge_dropped[publish_count] = u32(&payload[28]);
    publish_count++;
    return true;
}

int main(void)
{
    uint8_t startup[70];
    uint8_t flood[200];
    uint8_t status[32];
    uint16_t status_length = 0U;
    unsigned index;

    memset(&uart_stats, 0, sizeof(uart_stats));
    for (index = 0U; index < sizeof(startup); index++) startup[index] = (uint8_t)index;
    startup[0] = 0x00U;
    startup[1] = 0x7DU;
    startup[2] = 0x7EU;
    startup[3] = 0xFFU;
    M6VisionUart_Init();
    M6VisionUart_SetEnabled(true);
    uart_stats.rx_bytes = sizeof(startup);
    uart_stats.rx_overflow = 2U;
    uart_stats.hardware_errors = 1U;
    M6VisionUart_MirrorRx(startup, sizeof(startup));
    M6VisionUart_Service(1234U);
    assert(publish_count == 1U);
    assert(published_priority == M6_TX_P3);
    assert(published_type == M6_FRAME_BULK);
    assert(published_topic == 2U);
    assert(published_length == 96U);
    assert(published_payload[0] == 2U && published_payload[1] == 1U);
    assert(u16(&published_payload[2]) == 0x000AU);
    assert(u32(&published_payload[4]) == 1234U);
    assert(u32(&published_payload[8]) == 70U);
    assert(u32(&published_payload[16]) == 2U);
    assert(u32(&published_payload[24]) == 1U);
    assert(memcmp(&published_payload[32], startup, 64U) == 0);

    M6VisionUart_Service(1250U);
    assert(publish_count == 2U);
    assert(published_length == 38U);
    assert((u16(&published_payload[2]) & 0x0001U) == 0U);
    assert(memcmp(&published_payload[32], startup + 64U, 6U) == 0);
    M6VisionUart_Service(1260U);
    assert(publish_count == 2U);

    for (index = 0U; index < sizeof(flood); index++) flood[index] = (uint8_t)(255U - index);
    uart_stats.rx_bytes += sizeof(flood);
    M6VisionUart_MirrorRx(flood, sizeof(flood));
    publish_accept = false;
    M6VisionUart_Service(1300U);
    assert(publish_count == 2U);
    M6VisionUart_GetStatus(status, &status_length);
    assert(status_length == 32U);
    assert(status[0] == 1U && status[1] == 1U);
    assert(u32(&status[24]) == 72U);
    assert(u16(&status[28]) == 0U);

    publish_accept = true;
    M6VisionUart_Service(1320U);
    assert(publish_count == 4U);
    assert((published_flags[2] & 0x0004U) != 0U);
    assert(published_bridge_dropped[2] == 72U);
    assert((published_flags[3] & 0x0004U) == 0U);
    assert(u32(&published_payload[28]) == 72U);
    assert(memcmp(&published_payload[32], flood + 64U, 64U) == 0);
    M6VisionUart_Service(1340U);
    assert(publish_count == 4U);

    M6VisionUart_SetEnabled(false);
    M6VisionUart_MirrorRx(startup, 1U);
    M6VisionUart_Service(1400U);
    assert(publish_count == 4U);
    M6VisionUart_GetStatus(status, &status_length);
    assert(status[0] == 0U);
    return 0;
}
