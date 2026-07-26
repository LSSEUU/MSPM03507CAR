#ifndef RESET_DIAG_H
#define RESET_DIAG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RESET_DIAG_CLASS_NONE = 0,
    RESET_DIAG_CLASS_POWER,
    RESET_DIAG_CLASS_EXTERNAL,
    RESET_DIAG_CLASS_SOFTWARE,
    RESET_DIAG_CLASS_WATCHDOG,
    RESET_DIAG_CLASS_DEBUG,
    RESET_DIAG_CLASS_HARDWARE_FAULT,
    RESET_DIAG_CLASS_BSL,
    RESET_DIAG_CLASS_UNKNOWN
} ResetDiagClass_t;

/* Values are persistent wire/storage IDs. Append only; never reorder. */
typedef enum {
    RESET_DIAG_CHECKPOINT_NONE = 0,
    RESET_DIAG_CHECKPOINT_BOOT = 1,
    RESET_DIAG_CHECKPOINT_INIT = 2,
    RESET_DIAG_CHECKPOINT_MAIN_LOOP = 3,
    RESET_DIAG_CHECKPOINT_TASK_5MS = 4,
    RESET_DIAG_CHECKPOINT_TASK_10MS = 5,
    RESET_DIAG_CHECKPOINT_TASK_20MS = 6,
    RESET_DIAG_CHECKPOINT_TASK_100MS = 7,
    RESET_DIAG_CHECKPOINT_CONTROL_RX = 8,
    RESET_DIAG_CHECKPOINT_TX_ENCODE = 9,
    RESET_DIAG_CHECKPOINT_TX_SUBMIT = 10,
    RESET_DIAG_CHECKPOINT_TELEMETRY = 11,
    RESET_DIAG_CHECKPOINT_GRAYSCALE = 12,
    RESET_DIAG_CHECKPOINT_OLED = 13,
    RESET_DIAG_CHECKPOINT_PARAMETER_FLASH = 14,
    RESET_DIAG_CHECKPOINT_WWDT_SERVICE = 15,
    RESET_DIAG_CHECKPOINT_SAFETY_FAULT = 16,
    RESET_DIAG_CHECKPOINT_EMERGENCY = 17,
    RESET_DIAG_CHECKPOINT_HARDFAULT = 18,
    RESET_DIAG_CHECKPOINT_IDLE = 19,
    RESET_DIAG_CHECKPOINT_COUNT
} ResetDiagCheckpoint_t;

typedef struct {
    uint32_t now_ms;
    uint32_t last_watchdog_service_ms;
    uint32_t scheduler_missed_total;
    uint32_t safety_fault_bits;
    uint32_t uart_tx_dropped;
    uint32_t uart_rx_dropped;
    uint32_t telemetry_dropped;
    uint32_t scheduler_pending_mask;
    uint32_t scheduler_missed_mask;
    ResetDiagCheckpoint_t checkpoint;
    uint8_t system_state;
    uint8_t telemetry_queued;
    uint8_t scheduler_health_mask;
} ResetDiagHealth_t;

typedef struct {
    uint32_t raw_cause;
    ResetDiagClass_t reset_class;
    uint32_t boot_id;
    ResetDiagCheckpoint_t checkpoint;

    bool previous_coarse_valid;
    bool previous_detail_valid;
    bool previous_hardfault;
    uint8_t previous_boot_id_low;
    ResetDiagCheckpoint_t previous_checkpoint;

    uint32_t previous_sequence;
    uint32_t previous_boot_id;
    uint32_t previous_time_ms;
    uint32_t previous_last_watchdog_service_ms;
    uint32_t previous_scheduler_missed_total;
    uint32_t previous_safety_fault_bits;
    uint32_t previous_uart_tx_dropped;
    uint32_t previous_uart_rx_dropped;
    uint32_t previous_telemetry_dropped;
    uint32_t previous_scheduler_pending_mask;
    uint32_t previous_scheduler_missed_mask;
    uint32_t previous_hardfault_pc;
    uint32_t previous_hardfault_lr;
    uint32_t previous_hardfault_xpsr;
    uint8_t previous_detail_checkpoint;
    uint8_t previous_system_state;
    uint8_t previous_telemetry_queued;
    uint8_t previous_scheduler_health_mask;
} ResetDiagSnapshot_t;

/* Call immediately after the sole read-to-clear RSTCAUSE read. */
void ResetDiag_EarlyInit(uint32_t raw_cause);
void ResetDiag_Checkpoint(ResetDiagCheckpoint_t checkpoint);
void ResetDiag_RecordHealthyWindow(const ResetDiagHealth_t *health);
void ResetDiag_GetSnapshot(ResetDiagSnapshot_t *snapshot);

ResetDiagClass_t ResetDiag_Classify(uint32_t raw_cause);
const char *ResetDiag_RawName(uint32_t raw_cause);
const char *ResetDiag_ClassName(ResetDiagClass_t reset_class);
const char *ResetDiag_CheckpointName(ResetDiagCheckpoint_t checkpoint);

#ifndef RESET_DIAG_HOST_TEST
void ResetDiag_HardFaultReset(void) __attribute__((noreturn));
#else
void ResetDiag_TestClearRetained(void);
uint8_t ResetDiag_TestGetShutdownByte(uint8_t index);
void ResetDiag_TestSetShutdownByte(uint8_t index, uint8_t value);
void ResetDiag_TestCorruptSlot(uint8_t slot, uint8_t offset, uint8_t xor_mask);
void ResetDiag_TestForceSequence(uint32_t sequence);
void ResetDiag_TestSimulateHardFault(void);
void ResetDiag_TestSimulateEarlyHardFault(void);
#endif

#endif
