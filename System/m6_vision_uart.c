#include "m6_vision_uart.h"

#include "m6_link.h"
#include "m6_wire.h"
#include "vision_uart.h"

#include <string.h>

#define M6_VISION_BULK_KIND                 2U
#define M6_VISION_CHUNK_DATA_MAX            64U
#define M6_VISION_RECENT_BACKLOG_BYTES      128U
#define M6_VISION_FLAG_STARTUP_BACKLOG      (1U << 0)
#define M6_VISION_FLAG_RX_OVERFLOW          (1U << 1)
#define M6_VISION_FLAG_BRIDGE_DROP          (1U << 2)
#define M6_VISION_FLAG_HARDWARE_ERROR       (1U << 3)

typedef struct {
    bool enabled;
    bool pending;
    uint16_t startup_remaining;
    uint8_t pending_payload[M6_MAX_PAYLOAD];
    uint8_t pending_length;
    uint32_t bridge_dropped;
    uint32_t last_rx_overflow;
    uint32_t last_hardware_errors;
    uint32_t last_bridge_dropped;
} M6VisionUartState_t;

static M6VisionUartState_t s_monitor;

static void p16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void p32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void discard_old_backlog(void)
{
    VisionUartStats_t stats;
    uint8_t scratch[32];
    size_t discard;

    if (s_monitor.startup_remaining != 0U) return;
    VisionUart_GetStats(&stats);
    if (stats.rx_queued <= M6_VISION_RECENT_BACKLOG_BYTES) return;
    discard = (size_t)(stats.rx_queued - M6_VISION_RECENT_BACKLOG_BYTES);
    while (discard != 0U) {
        size_t request = discard > sizeof(scratch) ? sizeof(scratch) : discard;
        size_t consumed = VisionUart_Read(scratch, request);
        if (consumed == 0U) break;
        discard -= consumed;
        s_monitor.bridge_dropped += (uint32_t)consumed;
    }
}

static bool publish_pending(void)
{
    if (!s_monitor.pending) return true;
    if (!M6Link_Publish(M6_TX_P3, M6_FRAME_BULK,
                        M6_VISION_BULK_KIND,
                        s_monitor.pending_payload,
                        s_monitor.pending_length))
        return false;
    s_monitor.pending = false;
    return true;
}

void M6VisionUart_Init(void)
{
    memset(&s_monitor, 0, sizeof(s_monitor));
}

void M6VisionUart_SetEnabled(bool enabled)
{
    VisionUartStats_t stats;
    if (enabled == s_monitor.enabled) return;
    s_monitor.enabled = enabled;
    s_monitor.pending = false;
    if (!enabled) {
        s_monitor.startup_remaining = 0U;
        return;
    }
    VisionUart_GetStats(&stats);
    s_monitor.startup_remaining = stats.rx_queued;
    /* Report startup damage with the backlog instead of treating it as complete. */
    s_monitor.last_rx_overflow = 0U;
    s_monitor.last_hardware_errors = 0U;
    s_monitor.last_bridge_dropped = s_monitor.bridge_dropped;
}

void M6VisionUart_GetStatus(uint8_t *payload, uint16_t *payload_length)
{
    VisionUartStats_t stats;
    if ((payload == NULL) || (payload_length == NULL)) return;
    VisionUart_GetStats(&stats);
    memset(payload, 0, 32U);
    payload[0] = s_monitor.enabled ? 1U : 0U;
    payload[1] = 1U;
    p32(&payload[4], stats.rx_bytes);
    p32(&payload[8], stats.tx_bytes);
    p32(&payload[12], stats.rx_overflow);
    p32(&payload[16], stats.tx_overflow);
    p32(&payload[20], stats.hardware_errors);
    p32(&payload[24], s_monitor.bridge_dropped);
    p16(&payload[28], stats.rx_queued);
    p16(&payload[30], stats.tx_queued);
    *payload_length = 32U;
}

void M6VisionUart_Service(uint32_t now_ms)
{
    VisionUartStats_t stats;
    uint16_t flags = 0U;
    size_t length;

    if (!s_monitor.enabled) return;
    if (!publish_pending()) {
        discard_old_backlog();
        return;
    }
    discard_old_backlog();
    VisionUart_GetStats(&stats);
    if (stats.rx_queued == 0U) return;

    memset(s_monitor.pending_payload, 0, sizeof(s_monitor.pending_payload));
    length = VisionUart_Read(&s_monitor.pending_payload[32],
                             M6_VISION_CHUNK_DATA_MAX);
    if (length == 0U) return;
    if (s_monitor.startup_remaining != 0U) {
        flags |= M6_VISION_FLAG_STARTUP_BACKLOG;
        if (length >= s_monitor.startup_remaining)
            s_monitor.startup_remaining = 0U;
        else
            s_monitor.startup_remaining -= (uint16_t)length;
    }
    VisionUart_GetStats(&stats);
    if (stats.rx_overflow != s_monitor.last_rx_overflow)
        flags |= M6_VISION_FLAG_RX_OVERFLOW;
    if (stats.hardware_errors != s_monitor.last_hardware_errors)
        flags |= M6_VISION_FLAG_HARDWARE_ERROR;
    if (s_monitor.bridge_dropped != s_monitor.last_bridge_dropped)
        flags |= M6_VISION_FLAG_BRIDGE_DROP;

    s_monitor.pending_payload[0] = M6_VISION_BULK_KIND;
    s_monitor.pending_payload[1] = 1U;
    p16(&s_monitor.pending_payload[2], flags);
    p32(&s_monitor.pending_payload[4], now_ms);
    p32(&s_monitor.pending_payload[8], stats.rx_bytes);
    p32(&s_monitor.pending_payload[12], stats.tx_bytes);
    p32(&s_monitor.pending_payload[16], stats.rx_overflow);
    p32(&s_monitor.pending_payload[20], stats.tx_overflow);
    p32(&s_monitor.pending_payload[24], stats.hardware_errors);
    p32(&s_monitor.pending_payload[28], s_monitor.bridge_dropped);
    s_monitor.pending_length = (uint8_t)(32U + length);
    s_monitor.pending = true;
    s_monitor.last_rx_overflow = stats.rx_overflow;
    s_monitor.last_hardware_errors = stats.hardware_errors;
    s_monitor.last_bridge_dropped = s_monitor.bridge_dropped;
    (void)publish_pending();
}
