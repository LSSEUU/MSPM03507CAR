#include "zdt_stepper.h"
#include "zdt_stepper_uart.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static uint8_t tx_data[512];
static size_t tx_length;
static uint8_t rx_data[512];
static size_t rx_length;
static bool reject_write;
static ZdtStepperUartStats_t uart_stats;

void ZdtStepperUart_Init(void)
{
    tx_length = 0U;
    rx_length = 0U;
    reject_write = false;
    memset(&uart_stats, 0, sizeof(uart_stats));
}

bool ZdtStepperUart_Write(const uint8_t *data, size_t length)
{
    if (reject_write) {
        uart_stats.tx_overflow += (uint32_t)length;
        return false;
    }
    assert(tx_length + length <= sizeof(tx_data));
    memcpy(tx_data + tx_length, data, length);
    tx_length += length;
    uart_stats.tx_bytes += (uint32_t)length;
    uart_stats.tx_queued = (uint16_t)tx_length;
    if (uart_stats.tx_queued > uart_stats.tx_high_water)
        uart_stats.tx_high_water = uart_stats.tx_queued;
    return true;
}

bool ZdtStepperUart_WriteEmergencyBatch(const uint8_t *const frames[],
                                        const uint8_t lengths[],
                                        size_t count)
{
    size_t index;
    if (reject_write) return false;
    tx_length = 0U;
    for (index = 0U; index < count; index++) {
        assert(tx_length + lengths[index] <= sizeof(tx_data));
        memcpy(tx_data + tx_length, frames[index], lengths[index]);
        tx_length += lengths[index];
    }
    uart_stats.tx_bytes += (uint32_t)tx_length;
    return true;
}

size_t ZdtStepperUart_Read(uint8_t *data, size_t capacity)
{
    size_t length = rx_length < capacity ? rx_length : capacity;
    memcpy(data, rx_data, length);
    memmove(rx_data, rx_data + length, rx_length - length);
    rx_length -= length;
    uart_stats.rx_queued = (uint16_t)rx_length;
    return length;
}

void ZdtStepperUart_GetStats(ZdtStepperUartStats_t *stats)
{
    uart_stats.tx_queued = (uint16_t)tx_length;
    uart_stats.rx_queued = (uint16_t)rx_length;
    *stats = uart_stats;
}

static void reset_tx(void)
{
    tx_length = 0U;
    uart_stats.tx_queued = 0U;
}

static void assert_frame(const uint8_t *expected, size_t length)
{
    assert(tx_length == length);
    assert(memcmp(tx_data, expected, length) == 0);
    reset_tx();
}

static void feed(const uint8_t *data, size_t length)
{
    assert(rx_length + length <= sizeof(rx_data));
    memcpy(rx_data + rx_length, data, length);
    rx_length += length;
    uart_stats.rx_bytes += (uint32_t)length;
    uart_stats.rx_queued = (uint16_t)rx_length;
    if (uart_stats.rx_queued > uart_stats.rx_high_water)
        uart_stats.rx_high_water = uart_stats.rx_queued;
}

static void test_command_frames(void)
{
    const uint8_t enable[] = {0x01U, 0xF3U, 0xABU, 0x01U, 0x00U, 0x6BU};
    const uint8_t disable[] = {0x01U, 0xF3U, 0xABU, 0x00U, 0x00U, 0x6BU};
    const uint8_t stop[] = {0x01U, 0xFEU, 0x98U, 0x00U, 0x6BU};
    const uint8_t velocity[] = {
        0x01U, 0xF6U, 0x01U, 0x0BU, 0xB8U, 0xFFU, 0x00U, 0x6BU
    };
    const uint8_t position[] = {
        0x01U, 0xFDU, 0x00U, 0x03U, 0x20U, 0x64U, 0x12U,
        0x34U, 0x56U, 0x78U, 0x02U, 0x00U, 0x6BU
    };
    const uint8_t zero[] = {0x01U, 0x0AU, 0x6DU, 0x6BU};
    const uint8_t clear[] = {0x01U, 0x0EU, 0x52U, 0x6BU};
    const uint8_t home[] = {0x01U, 0x9AU, 0x05U, 0x00U, 0x6BU};
    const uint8_t abort_home[] = {0x01U, 0x9CU, 0x48U, 0x6BU};
    const uint8_t read_speed[] = {0x01U, 0x35U, 0x6BU};
    const uint8_t read_encoder[] = {0x01U, 0x31U, 0x6BU};
    const uint8_t read_position[] = {0x01U, 0x36U, 0x6BU};
    const uint8_t read_motor[] = {0x01U, 0x3AU, 0x6BU};
    const uint8_t read_home[] = {0x01U, 0x3BU, 0x6BU};
    const uint8_t read_config[] = {0x01U, 0x42U, 0x6CU, 0x6BU};

    ZdtStepper_Init();
    assert(tx_length == 0U);
    assert(ZdtStepper_Enable(true) == ZDT_STEPPER_OK);
    assert_frame(enable, sizeof(enable));
    assert(ZdtStepper_Enable(false) == ZDT_STEPPER_OK);
    assert_frame(disable, sizeof(disable));
    assert(ZdtStepper_Stop() == ZDT_STEPPER_OK);
    assert_frame(stop, sizeof(stop));
    assert(ZdtStepper_EmergencyStop(false) == ZDT_STEPPER_OK);
    assert_frame(stop, sizeof(stop));
    assert(ZdtStepper_EmergencyStop(true) == ZDT_STEPPER_OK);
    assert(tx_length == sizeof(stop) + sizeof(disable));
    assert(memcmp(tx_data, stop, sizeof(stop)) == 0);
    assert(memcmp(tx_data + sizeof(stop), disable, sizeof(disable)) == 0);
    reset_tx();
    assert(ZdtStepper_EmergencyDisable() == ZDT_STEPPER_OK);
    assert_frame(disable, sizeof(disable));
    assert(ZdtStepper_RunVelocity(ZDT_STEPPER_DIRECTION_CCW, 3000U, 255U) ==
           ZDT_STEPPER_OK);
    assert_frame(velocity, sizeof(velocity));
    assert(ZdtStepper_MovePosition(ZDT_STEPPER_DIRECTION_CW, 800U, 100U,
                                   0x12345678UL,
                                   ZDT_STEPPER_POSITION_RELATIVE_CURRENT) ==
           ZDT_STEPPER_OK);
    assert_frame(position, sizeof(position));
    assert(ZdtStepper_ZeroCurrentPosition() == ZDT_STEPPER_OK);
    assert_frame(zero, sizeof(zero));
    assert(ZdtStepper_ClearProtection() == ZDT_STEPPER_OK);
    assert_frame(clear, sizeof(clear));
    assert(ZdtStepper_StartHoming(ZDT_STEPPER_HOME_POWER_LOSS_POSITION) ==
           ZDT_STEPPER_OK);
    assert_frame(home, sizeof(home));
    assert(ZdtStepper_AbortHoming() == ZDT_STEPPER_OK);
    assert_frame(abort_home, sizeof(abort_home));
    assert(ZdtStepper_ReadSpeed() == ZDT_STEPPER_OK);
    assert_frame(read_speed, sizeof(read_speed));
    assert(ZdtStepper_ReadEncoder() == ZDT_STEPPER_OK);
    assert_frame(read_encoder, sizeof(read_encoder));
    assert(ZdtStepper_ReadPosition() == ZDT_STEPPER_OK);
    assert_frame(read_position, sizeof(read_position));
    assert(ZdtStepper_ReadMotorStatus() == ZDT_STEPPER_OK);
    assert_frame(read_motor, sizeof(read_motor));
    assert(ZdtStepper_ReadHomingStatus() == ZDT_STEPPER_OK);
    assert_frame(read_home, sizeof(read_home));
    assert(ZdtStepper_ReadDriverConfig() == ZDT_STEPPER_OK);
    assert_frame(read_config, sizeof(read_config));

    assert(ZdtStepper_RunVelocity((ZdtStepperDirection_t)2, 10U, 0U) ==
           ZDT_STEPPER_INVALID_ARGUMENT);
    assert(ZdtStepper_RunVelocity(ZDT_STEPPER_DIRECTION_CW, 3001U, 0U) ==
           ZDT_STEPPER_INVALID_ARGUMENT);
    assert(ZdtStepper_MovePosition(ZDT_STEPPER_DIRECTION_CW, 1U, 0U, 1U,
                                   (ZdtStepperPositionMode_t)3) ==
           ZDT_STEPPER_INVALID_ARGUMENT);
    assert(ZdtStepper_StartHoming((ZdtStepperHomeMode_t)6) ==
           ZDT_STEPPER_INVALID_ARGUMENT);
    assert(tx_length == 0U);

    reject_write = true;
    assert(ZdtStepper_ReadSpeed() == ZDT_STEPPER_TX_FULL);
    assert(tx_length == 0U);
    reject_write = false;
}

static void test_response_codes(void)
{
    static const uint8_t wire[] = {0x02U, 0x12U, 0x22U, 0x9FU, 0xE2U, 0xEEU};
    static const ZdtStepperResponse_t decoded[] = {
        ZDT_STEPPER_RESPONSE_ACCEPTED,
        ZDT_STEPPER_RESPONSE_AT_ORIGIN,
        ZDT_STEPPER_RESPONSE_LIMIT_ACTIVE,
        ZDT_STEPPER_RESPONSE_COMPLETED,
        ZDT_STEPPER_RESPONSE_PARAMETER_OR_STATE_ERROR,
        ZDT_STEPPER_RESPONSE_FORMAT_ERROR
    };
    ZdtStepperSnapshot_t snapshot;
    size_t index;

    ZdtStepper_Init();
    for (index = 0U; index < sizeof(wire); index++) {
        uint8_t frame[] = {0x01U, 0xF3U, wire[index], 0x6BU};
        feed(frame, 2U);
        ZdtStepper_Poll();
        ZdtStepper_GetSnapshot(&snapshot);
        assert(snapshot.response_sequence == (uint32_t)index);
        feed(frame + 2U, 2U);
        ZdtStepper_Poll();
        ZdtStepper_GetSnapshot(&snapshot);
        assert(snapshot.response_sequence == (uint32_t)index + 1U);
        assert(snapshot.last_function == 0xF3U);
        assert(snapshot.last_response == decoded[index]);
    }
}

static void test_data_and_recovery(void)
{
    const uint8_t data_frames[] = {
        0x01U, 0x35U, 0x01U, 0x04U, 0xD2U, 0x6BU,
        0x01U, 0x36U, 0x01U, 0x00U, 0x01U, 0x00U, 0x00U, 0x6BU,
        0x01U, 0x3AU, 0xBDU, 0x6BU,
        0x01U, 0x3BU, 0x3FU, 0x6BU
    };
    const uint8_t short_error[] = {0x01U, 0x35U, 0xE2U, 0x6BU};
    const uint8_t motor_reached[] = {0x01U, 0x3AU, 0x02U, 0x6BU};
    const uint8_t damaged_stream[] = {
        0x55U, 0x00U,
        0x02U, 0xF3U, 0x02U, 0x6BU,
        0x01U, 0xF3U, 0x02U, 0x00U,
        0x01U, 0x44U, 0x6BU,
        0x01U, 0x35U, 0x02U, 0x00U, 0x01U, 0x6BU,
        0x01U, 0xFEU, 0x02U, 0x6BU
    };
    ZdtStepperSnapshot_t snapshot;
    uint32_t before;

    ZdtStepper_Init();
    feed(data_frames, sizeof(data_frames));
    ZdtStepper_Poll();
    ZdtStepper_GetSnapshot(&snapshot);
    assert(snapshot.response_sequence == 4U);
    assert(snapshot.speed_valid && snapshot.speed_negative);
    assert(snapshot.speed_rpm == 1234U);
    assert(snapshot.position_valid && snapshot.position_negative);
    assert(snapshot.position_raw == 65536UL);
    assert(snapshot.motor_status_valid && snapshot.motor_status == 0xBDU);
    assert(snapshot.homing_status_valid && snapshot.homing_status == 0x3FU);
    assert(snapshot.last_response == ZDT_STEPPER_RESPONSE_DATA);

    feed(short_error, sizeof(short_error));
    ZdtStepper_Poll();
    ZdtStepper_GetSnapshot(&snapshot);
    assert(snapshot.last_function == 0x35U);
    assert(snapshot.last_response ==
           ZDT_STEPPER_RESPONSE_PARAMETER_OR_STATE_ERROR);

    feed(motor_reached, sizeof(motor_reached));
    ZdtStepper_Poll();
    ZdtStepper_GetSnapshot(&snapshot);
    assert(snapshot.last_function == 0x3AU);
    assert(snapshot.last_response == ZDT_STEPPER_RESPONSE_DATA);
    assert(snapshot.motor_status == ZDT_STEPPER_MOTOR_POSITION_REACHED);

    before = snapshot.response_sequence;
    feed(damaged_stream, sizeof(damaged_stream));
    ZdtStepper_Poll();
    ZdtStepper_GetSnapshot(&snapshot);
    assert(snapshot.response_sequence == before + 1U);
    assert(snapshot.last_function == 0xFEU);
    assert(snapshot.last_response == ZDT_STEPPER_RESPONSE_ACCEPTED);
    assert(snapshot.stats.checksum_errors >= 1U);
    assert(snapshot.stats.invalid_frames >= 2U);
    assert(snapshot.stats.discarded_rx_bytes >= 1U);
    assert(snapshot.stats.rx_bytes ==
           sizeof(data_frames) + sizeof(short_error) +
           sizeof(motor_reached) + sizeof(damaged_stream));
}

static void test_encoder_wrap_and_variable_config(void)
{
    const uint8_t encoder[] = {0x01U, 0x31U, 0xFFU, 0xF8U, 0x6BU};
    const uint8_t encoder_old_error[] = {0x01U, 0x31U, 0xE2U, 0x6BU};
    const uint8_t config_old_error[] = {0x01U, 0x42U, 0xE2U, 0x6BU};
    const uint8_t config[] = {
        0x01U, 0x42U, 0x21U, 0x15U, 0x01U, 0x02U, 0x03U, 0x04U,
        0x01U, 0x10U, 0x01U, 0x00U, 0x03U, 0x20U, 0x04U, 0x00U,
        0x18U, 0x00U, 0x04U, 0x01U, 0x01U, 0x00U, 0x01U, 0x01U,
        0x00U, 0x64U, 0x01U, 0x2CU, 0x00U, 0x32U, 0x00U, 0x0AU,
        0x6BU
    };
    uint8_t damaged[sizeof(config)];
    ZdtStepperSnapshot_t snapshot;
    uint32_t before;

    ZdtStepper_Init();
    assert(sizeof(config) == 0x21U);
    feed(encoder, sizeof(encoder));
    ZdtStepper_Poll();
    ZdtStepper_GetSnapshot(&snapshot);
    assert(snapshot.encoder_valid && snapshot.encoder_raw == 65528U);
    assert(snapshot.encoder_sequence == 1U);
    assert(ZdtStepper_CircularDelta(4U, 65528U) == 12);
    assert(ZdtStepper_CircularDelta(65528U, 4U) == -12);

    feed(config, 7U);
    ZdtStepper_Poll();
    ZdtStepper_GetSnapshot(&snapshot);
    assert(!snapshot.driver_config_valid);
    feed(config + 7U, sizeof(config) - 7U);
    ZdtStepper_Poll();
    ZdtStepper_GetSnapshot(&snapshot);
    assert(snapshot.driver_config_valid);
    assert(snapshot.driver_config_sequence == 1U);
    assert(snapshot.driver_config.length == 0x21U);
    assert(snapshot.driver_config.communication_mode == 0x03U);
    assert(snapshot.driver_config.direction == 0x01U);
    assert(snapshot.driver_config.subdivision == 0x10U);
    assert(snapshot.driver_config.stall_protection == 0x01U);
    assert(snapshot.driver_config_fingerprint != 0U);

    before = snapshot.response_sequence;
    feed(config_old_error, 3U);
    ZdtStepper_Poll();
    ZdtStepper_GetSnapshot(&snapshot);
    assert(snapshot.response_sequence == before);
    feed(config_old_error + 3U, 1U);
    ZdtStepper_Poll();
    ZdtStepper_GetSnapshot(&snapshot);
    assert(snapshot.response_sequence == before + 1U);
    assert(snapshot.last_function == 0x42U);
    assert(snapshot.last_response ==
           ZDT_STEPPER_RESPONSE_PARAMETER_OR_STATE_ERROR);

    before = snapshot.response_sequence;
    memcpy(damaged, config, sizeof(config));
    damaged[sizeof(damaged) - 1U] = 0x00U;
    feed(damaged, sizeof(damaged));
    feed(encoder_old_error, sizeof(encoder_old_error));
    ZdtStepper_Poll();
    ZdtStepper_GetSnapshot(&snapshot);
    assert(snapshot.response_sequence == before + 1U);
    assert(snapshot.last_function == 0x31U);
    assert(snapshot.last_response ==
           ZDT_STEPPER_RESPONSE_PARAMETER_OR_STATE_ERROR);
    assert(snapshot.stats.checksum_errors >= 1U);
}

int main(void)
{
    test_command_frames();
    test_response_codes();
    test_data_and_recovery();
    test_encoder_wrap_and_variable_config();
    return 0;
}
