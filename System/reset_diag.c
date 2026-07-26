#include "reset_diag.h"

#include <stddef.h>
#include <string.h>

#ifndef RESET_DIAG_HOST_TEST
#include "ti_msp_dl_config.h"
#endif

#define RESET_DIAG_SHUTDOWN_MAGIC       0xB6U
#define RESET_DIAG_SHUTDOWN_COMMIT      0x49U
#define RESET_DIAG_RECORD_MAGIC         0x52364447UL
#define RESET_DIAG_RECORD_VERSION       1U
#define RESET_DIAG_RECORD_COMMIT        0xC63A91E7UL
#define RESET_DIAG_HARDFAULT_MAGIC      0x48465236UL
#define RESET_DIAG_HARDFAULT_VERSION    1UL
#define RESET_DIAG_HARDFAULT_COMMIT     0xA731CE4DUL
#define RESET_DIAG_RETAINED_SIZE        256U

enum {
    RESET_DIAG_SHUTDOWN_MAGIC_INDEX = 0,
    RESET_DIAG_SHUTDOWN_COMMIT_INDEX = 1,
    RESET_DIAG_SHUTDOWN_CHECKPOINT_INDEX = 2,
    RESET_DIAG_SHUTDOWN_BOOT_INDEX = 3
};

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    uint32_t boot_id;
    uint32_t time_ms;
    uint32_t last_watchdog_service_ms;
    uint32_t scheduler_missed_total;
    uint32_t safety_fault_bits;
    uint32_t uart_tx_dropped;
    uint32_t uart_rx_dropped;
    uint32_t telemetry_dropped;
    uint32_t scheduler_pending_mask;
    uint32_t scheduler_missed_mask;
    uint8_t checkpoint;
    uint8_t system_state;
    uint8_t telemetry_queued;
    uint8_t scheduler_health_mask;
    uint32_t crc32;
    uint32_t commit_marker;
} ResetDiagRecordV1_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t boot_id;
    uint32_t record_sequence;
    uint32_t stacked_pc;
    uint32_t stacked_lr;
    uint32_t stacked_xpsr;
    uint32_t crc32;
    uint32_t commit_marker;
} ResetDiagHardFaultScratch_t;

typedef struct {
    ResetDiagRecordV1_t slots[2];
    ResetDiagHardFaultScratch_t hardfault;
    uint8_t reserved[RESET_DIAG_RETAINED_SIZE -
        (2U * sizeof(ResetDiagRecordV1_t)) -
        sizeof(ResetDiagHardFaultScratch_t)];
} ResetDiagRetained_t;

_Static_assert(sizeof(ResetDiagRecordV1_t) == 64U,
               "reset breadcrumb record must remain 64 bytes");
_Static_assert(offsetof(ResetDiagRecordV1_t, crc32) == 56U,
               "reset breadcrumb CRC coverage changed");
_Static_assert(offsetof(ResetDiagRecordV1_t, commit_marker) == 60U,
               "reset breadcrumb commit must be last");
_Static_assert(offsetof(ResetDiagHardFaultScratch_t, crc32) == 28U,
               "HardFault scratch CRC coverage changed");
_Static_assert(offsetof(ResetDiagHardFaultScratch_t, commit_marker) == 32U,
               "HardFault scratch commit must be last");
_Static_assert(sizeof(ResetDiagRetained_t) == RESET_DIAG_RETAINED_SIZE,
               "reset retained region must match the scatter reservation");

#ifdef RESET_DIAG_HOST_TEST
static volatile ResetDiagRetained_t s_retained;
static uint8_t s_shutdown_storage[4];
#else
__attribute__((section(".bss.noinit.reset_diag"), used, aligned(8)))
static volatile ResetDiagRetained_t s_retained;
#endif

static uint32_t s_raw_cause;
static ResetDiagClass_t s_reset_class;
static uint32_t s_boot_id;
static volatile ResetDiagCheckpoint_t s_checkpoint;
static uint32_t s_sequence;
static uint8_t s_active_slot;
static bool s_previous_coarse_valid;
static bool s_previous_detail_valid;
static bool s_previous_hardfault;
static uint32_t s_previous_hardfault_pc;
static uint32_t s_previous_hardfault_lr;
static uint32_t s_previous_hardfault_xpsr;
static uint8_t s_previous_boot_id_low;
static ResetDiagCheckpoint_t s_previous_checkpoint;
static ResetDiagRecordV1_t s_previous_record;

static void reset_diag_barrier(void)
{
#ifdef RESET_DIAG_HOST_TEST
    __sync_synchronize();
#else
    __DMB();
#endif
}

static uint32_t reset_diag_enter_critical(void)
{
#ifdef RESET_DIAG_HOST_TEST
    return 0U;
#else
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#endif
}

static void reset_diag_exit_critical(uint32_t primask)
{
#ifdef RESET_DIAG_HOST_TEST
    (void)primask;
#else
    if (primask == 0U) __enable_irq();
#endif
}

static uint8_t shutdown_get(uint8_t index)
{
#ifdef RESET_DIAG_HOST_TEST
    return s_shutdown_storage[index];
#else
    return DL_SYSCTL_getShutdownStorageByte(
        (DL_SYSCTL_SHUTDOWN_STORAGE_BYTE)index);
#endif
}

static void shutdown_set(uint8_t index, uint8_t value)
{
#ifdef RESET_DIAG_HOST_TEST
    s_shutdown_storage[index] = value;
#else
    DL_SYSCTL_setShutdownStorageByte(
        (DL_SYSCTL_SHUTDOWN_STORAGE_BYTE)index, value);
#endif
}

static uint32_t reset_diag_crc32(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;
    size_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++) {
        crc ^= bytes[index];
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
    return crc ^ 0xFFFFFFFFUL;
}

static void copy_from_volatile(void *destination, const volatile void *source,
                               size_t length)
{
    uint8_t *dst = (uint8_t *)destination;
    const volatile uint8_t *src = (const volatile uint8_t *)source;
    size_t index;
    for (index = 0U; index < length; index++) dst[index] = src[index];
}

static void copy_to_volatile(volatile void *destination, const void *source,
                             size_t length)
{
    volatile uint8_t *dst = (volatile uint8_t *)destination;
    const uint8_t *src = (const uint8_t *)source;
    size_t index;
    for (index = 0U; index < length; index++) dst[index] = src[index];
}

static bool checkpoint_valid(uint8_t checkpoint)
{
    return checkpoint > (uint8_t)RESET_DIAG_CHECKPOINT_NONE &&
           checkpoint < (uint8_t)RESET_DIAG_CHECKPOINT_COUNT;
}

static bool cause_invalidates_retained(uint32_t raw_cause)
{
    return raw_cause == 0x00U || raw_cause == 0x01U ||
           raw_cause == 0x02U || raw_cause == 0x03U;
}

static bool record_valid(uint8_t slot, uint8_t expected_boot_id_low,
                         ResetDiagRecordV1_t *record)
{
    ResetDiagRecordV1_t candidate;
    if (slot >= 2U || record == NULL) return false;
    copy_from_volatile(&candidate, &s_retained.slots[slot],
                       sizeof(candidate));
    if (candidate.magic != RESET_DIAG_RECORD_MAGIC ||
        candidate.version != RESET_DIAG_RECORD_VERSION ||
        candidate.length != sizeof(candidate) ||
        candidate.commit_marker != RESET_DIAG_RECORD_COMMIT ||
        (uint8_t)candidate.boot_id != expected_boot_id_low ||
        !checkpoint_valid(candidate.checkpoint) ||
        candidate.crc32 != reset_diag_crc32(
            &candidate, offsetof(ResetDiagRecordV1_t, crc32))) return false;
    *record = candidate;
    return true;
}

static bool sequence_newer(uint32_t left, uint32_t right)
{
    return (int32_t)(left - right) > 0;
}

static bool select_previous_record(uint8_t expected_boot_id_low,
                                   ResetDiagRecordV1_t *selected,
                                   uint8_t *selected_slot)
{
    ResetDiagRecordV1_t first;
    ResetDiagRecordV1_t second;
    bool first_valid = record_valid(0U, expected_boot_id_low, &first);
    bool second_valid = record_valid(1U, expected_boot_id_low, &second);

    if (!first_valid && !second_valid) return false;
    if (first_valid && (!second_valid ||
        sequence_newer(first.sequence, second.sequence))) {
        *selected = first;
        *selected_slot = 0U;
    } else {
        *selected = second;
        *selected_slot = 1U;
    }
    return true;
}

static bool hardfault_scratch_valid(ResetDiagHardFaultScratch_t *scratch)
{
    ResetDiagHardFaultScratch_t candidate;
    if (scratch == NULL) return false;
    copy_from_volatile(&candidate, &s_retained.hardfault,
                       sizeof(candidate));
    if (candidate.magic != RESET_DIAG_HARDFAULT_MAGIC ||
        candidate.version != RESET_DIAG_HARDFAULT_VERSION ||
        candidate.commit_marker != RESET_DIAG_HARDFAULT_COMMIT ||
        candidate.crc32 != reset_diag_crc32(
            &candidate, offsetof(ResetDiagHardFaultScratch_t, crc32)))
        return false;
    *scratch = candidate;
    return true;
}

static void invalidate_retained_records(void)
{
    s_retained.slots[0].commit_marker = 0U;
    s_retained.slots[1].commit_marker = 0U;
    s_retained.hardfault.commit_marker = 0U;
    reset_diag_barrier();
}

static void commit_record(const ResetDiagHealth_t *health)
{
    ResetDiagRecordV1_t record;
    uint8_t target = (uint8_t)(s_active_slot ^ 1U);

    memset(&record, 0, sizeof(record));
    record.magic = RESET_DIAG_RECORD_MAGIC;
    record.version = RESET_DIAG_RECORD_VERSION;
    record.length = (uint16_t)sizeof(record);
    record.sequence = s_sequence + 1U;
    record.boot_id = s_boot_id;
    if (health != NULL) {
        record.time_ms = health->now_ms;
        record.last_watchdog_service_ms =
            health->last_watchdog_service_ms;
        record.scheduler_missed_total = health->scheduler_missed_total;
        record.safety_fault_bits = health->safety_fault_bits;
        record.uart_tx_dropped = health->uart_tx_dropped;
        record.uart_rx_dropped = health->uart_rx_dropped;
        record.telemetry_dropped = health->telemetry_dropped;
        record.scheduler_pending_mask = health->scheduler_pending_mask;
        record.scheduler_missed_mask = health->scheduler_missed_mask;
        record.checkpoint = (uint8_t)health->checkpoint;
        record.system_state = health->system_state;
        record.telemetry_queued = health->telemetry_queued;
        record.scheduler_health_mask = health->scheduler_health_mask;
    } else {
        record.checkpoint = (uint8_t)RESET_DIAG_CHECKPOINT_BOOT;
    }
    if (!checkpoint_valid(record.checkpoint))
        record.checkpoint = (uint8_t)s_checkpoint;
    record.crc32 = reset_diag_crc32(
        &record, offsetof(ResetDiagRecordV1_t, crc32));
    record.commit_marker = RESET_DIAG_RECORD_COMMIT;

    s_retained.slots[target].commit_marker = 0U;
    reset_diag_barrier();
    copy_to_volatile(&s_retained.slots[target], &record,
                     offsetof(ResetDiagRecordV1_t, commit_marker));
    reset_diag_barrier();
    s_retained.slots[target].commit_marker = RESET_DIAG_RECORD_COMMIT;
    reset_diag_barrier();
    s_active_slot = target;
    s_sequence = record.sequence;
}

static void write_current_shutdown_record(void)
{
    shutdown_set(RESET_DIAG_SHUTDOWN_COMMIT_INDEX, 0U);
    shutdown_set(RESET_DIAG_SHUTDOWN_MAGIC_INDEX,
                 RESET_DIAG_SHUTDOWN_MAGIC);
    shutdown_set(RESET_DIAG_SHUTDOWN_BOOT_INDEX, (uint8_t)s_boot_id);
    shutdown_set(RESET_DIAG_SHUTDOWN_CHECKPOINT_INDEX,
                 (uint8_t)RESET_DIAG_CHECKPOINT_BOOT);
    reset_diag_barrier();
    shutdown_set(RESET_DIAG_SHUTDOWN_COMMIT_INDEX,
                 RESET_DIAG_SHUTDOWN_COMMIT);
}

static void write_hardfault_scratch(uint32_t stacked_pc,
                                    uint32_t stacked_lr,
                                    uint32_t stacked_xpsr)
{
    ResetDiagHardFaultScratch_t scratch;

    memset(&scratch, 0, sizeof(scratch));
    scratch.magic = RESET_DIAG_HARDFAULT_MAGIC;
    scratch.version = RESET_DIAG_HARDFAULT_VERSION;
    scratch.boot_id = s_boot_id;
    scratch.record_sequence = s_sequence;
    scratch.stacked_pc = stacked_pc;
    scratch.stacked_lr = stacked_lr;
    scratch.stacked_xpsr = stacked_xpsr;
    scratch.crc32 = reset_diag_crc32(
        &scratch, offsetof(ResetDiagHardFaultScratch_t, crc32));
    scratch.commit_marker = RESET_DIAG_HARDFAULT_COMMIT;

    s_retained.hardfault.commit_marker = 0U;
    reset_diag_barrier();
    copy_to_volatile(&s_retained.hardfault, &scratch,
                     offsetof(ResetDiagHardFaultScratch_t, commit_marker));
    reset_diag_barrier();
    s_retained.hardfault.commit_marker = RESET_DIAG_HARDFAULT_COMMIT;
    reset_diag_barrier();

    /* Rebuild the complete coarse record as well. This remains self-validating
     * even if the fault happened before ResetDiag_EarlyInit established the
     * current boot record (boot_id is then the reserved value zero). */
    shutdown_set(RESET_DIAG_SHUTDOWN_COMMIT_INDEX, 0U);
    shutdown_set(RESET_DIAG_SHUTDOWN_MAGIC_INDEX,
                 RESET_DIAG_SHUTDOWN_MAGIC);
    shutdown_set(RESET_DIAG_SHUTDOWN_BOOT_INDEX, (uint8_t)s_boot_id);
    shutdown_set(RESET_DIAG_SHUTDOWN_CHECKPOINT_INDEX,
                 (uint8_t)RESET_DIAG_CHECKPOINT_HARDFAULT);
    reset_diag_barrier();
    shutdown_set(RESET_DIAG_SHUTDOWN_COMMIT_INDEX,
                 RESET_DIAG_SHUTDOWN_COMMIT);
}

ResetDiagClass_t ResetDiag_Classify(uint32_t raw_cause)
{
    switch (raw_cause) {
        case 0x00U: return RESET_DIAG_CLASS_NONE;
        case 0x01U:
        case 0x04U:
        case 0x05U: return RESET_DIAG_CLASS_POWER;
        case 0x02U:
        case 0x0CU: return RESET_DIAG_CLASS_EXTERNAL;
        case 0x03U:
        case 0x0DU:
        case 0x1BU:
        case 0x1DU: return RESET_DIAG_CLASS_SOFTWARE;
        case 0x0EU:
        case 0x13U: return RESET_DIAG_CLASS_WATCHDOG;
        case 0x1AU:
        case 0x1CU: return RESET_DIAG_CLASS_DEBUG;
        case 0x08U:
        case 0x09U:
        case 0x14U:
        case 0x15U: return RESET_DIAG_CLASS_HARDWARE_FAULT;
        case 0x10U:
        case 0x11U: return RESET_DIAG_CLASS_BSL;
        default: return RESET_DIAG_CLASS_UNKNOWN;
    }
}

const char *ResetDiag_RawName(uint32_t raw_cause)
{
    switch (raw_cause) {
        case 0x00U: return "none";
        case 0x01U: return "por_or_parity";
        case 0x02U: return "external_por";
        case 0x03U: return "software_por";
        case 0x04U: return "brownout";
        case 0x05U: return "shutdown_wake";
        case 0x08U: return "non_pmu_parity";
        case 0x09U: return "clock_fault";
        case 0x0CU: return "external_boot";
        case 0x0DU: return "software_boot";
        case 0x0EU: return "wwdt0";
        case 0x10U: return "bsl_exit";
        case 0x11U: return "bsl_entry";
        case 0x13U: return "wwdt1";
        case 0x14U: return "flash_ecc";
        case 0x15U: return "cpu_lockup";
        case 0x1AU: return "debug_sys";
        case 0x1BU: return "software_sys";
        case 0x1CU: return "debug_cpu";
        case 0x1DU: return "software_cpu";
        default: return "unknown";
    }
}

const char *ResetDiag_ClassName(ResetDiagClass_t reset_class)
{
    switch (reset_class) {
        case RESET_DIAG_CLASS_NONE: return "none";
        case RESET_DIAG_CLASS_POWER: return "power";
        case RESET_DIAG_CLASS_EXTERNAL: return "external";
        case RESET_DIAG_CLASS_SOFTWARE: return "software";
        case RESET_DIAG_CLASS_WATCHDOG: return "watchdog";
        case RESET_DIAG_CLASS_DEBUG: return "debug";
        case RESET_DIAG_CLASS_HARDWARE_FAULT: return "hardware";
        case RESET_DIAG_CLASS_BSL: return "bsl";
        default: return "unknown";
    }
}

const char *ResetDiag_CheckpointName(ResetDiagCheckpoint_t checkpoint)
{
    switch (checkpoint) {
        case RESET_DIAG_CHECKPOINT_NONE: return "none";
        case RESET_DIAG_CHECKPOINT_BOOT: return "boot";
        case RESET_DIAG_CHECKPOINT_INIT: return "init";
        case RESET_DIAG_CHECKPOINT_MAIN_LOOP: return "main";
        case RESET_DIAG_CHECKPOINT_TASK_5MS: return "task5";
        case RESET_DIAG_CHECKPOINT_TASK_10MS: return "task10";
        case RESET_DIAG_CHECKPOINT_TASK_20MS: return "task20";
        case RESET_DIAG_CHECKPOINT_TASK_100MS: return "task100";
        case RESET_DIAG_CHECKPOINT_CONTROL_RX: return "control_rx";
        case RESET_DIAG_CHECKPOINT_TX_ENCODE: return "tx_encode";
        case RESET_DIAG_CHECKPOINT_TX_SUBMIT: return "tx_submit";
        case RESET_DIAG_CHECKPOINT_TELEMETRY: return "telemetry";
        case RESET_DIAG_CHECKPOINT_GRAYSCALE: return "grayscale";
        case RESET_DIAG_CHECKPOINT_OLED: return "oled";
        case RESET_DIAG_CHECKPOINT_PARAMETER_FLASH: return "param_flash";
        case RESET_DIAG_CHECKPOINT_WWDT_SERVICE: return "wwdt_service";
        case RESET_DIAG_CHECKPOINT_SAFETY_FAULT: return "safety_fault";
        case RESET_DIAG_CHECKPOINT_EMERGENCY: return "emergency";
        case RESET_DIAG_CHECKPOINT_HARDFAULT: return "hardfault";
        case RESET_DIAG_CHECKPOINT_IDLE: return "idle";
        default: return "unknown";
    }
}

void ResetDiag_EarlyInit(uint32_t raw_cause)
{
    uint8_t selected_slot = 0U;
    bool may_recover = !cause_invalidates_retained(raw_cause);
    bool scratch_valid = false;
    ResetDiagHardFaultScratch_t hardfault_scratch;

    s_raw_cause = raw_cause;
    s_reset_class = ResetDiag_Classify(raw_cause);
    s_checkpoint = RESET_DIAG_CHECKPOINT_BOOT;
    s_previous_coarse_valid = false;
    s_previous_detail_valid = false;
    s_previous_hardfault = false;
    s_previous_hardfault_pc = 0U;
    s_previous_hardfault_lr = 0U;
    s_previous_hardfault_xpsr = 0U;
    s_previous_boot_id_low = 0U;
    s_previous_checkpoint = RESET_DIAG_CHECKPOINT_NONE;
    memset(&s_previous_record, 0, sizeof(s_previous_record));

    memset(&hardfault_scratch, 0, sizeof(hardfault_scratch));
    if (may_recover)
        scratch_valid = hardfault_scratch_valid(&hardfault_scratch);

    if (may_recover &&
        shutdown_get(RESET_DIAG_SHUTDOWN_MAGIC_INDEX) ==
            RESET_DIAG_SHUTDOWN_MAGIC &&
        shutdown_get(RESET_DIAG_SHUTDOWN_COMMIT_INDEX) ==
            RESET_DIAG_SHUTDOWN_COMMIT &&
        checkpoint_valid(shutdown_get(
            RESET_DIAG_SHUTDOWN_CHECKPOINT_INDEX))) {
        s_previous_coarse_valid = true;
        s_previous_boot_id_low = shutdown_get(
            RESET_DIAG_SHUTDOWN_BOOT_INDEX);
        s_previous_checkpoint = (ResetDiagCheckpoint_t)shutdown_get(
            RESET_DIAG_SHUTDOWN_CHECKPOINT_INDEX);
    }

    if (scratch_valid &&
        (!s_previous_coarse_valid || hardfault_scratch.boot_id == 0U ||
         (uint8_t)hardfault_scratch.boot_id == s_previous_boot_id_low)) {
        if (s_previous_coarse_valid && hardfault_scratch.boot_id == 0U &&
            s_previous_boot_id_low != 0U)
            s_previous_coarse_valid = false;
        s_previous_hardfault = true;
        s_previous_boot_id_low = (uint8_t)hardfault_scratch.boot_id;
        s_previous_checkpoint = RESET_DIAG_CHECKPOINT_HARDFAULT;
        s_previous_hardfault_pc = hardfault_scratch.stacked_pc;
        s_previous_hardfault_lr = hardfault_scratch.stacked_lr;
        s_previous_hardfault_xpsr = hardfault_scratch.stacked_xpsr;
    }

    if (s_previous_coarse_valid || s_previous_hardfault) {
        s_previous_detail_valid = select_previous_record(
            s_previous_boot_id_low, &s_previous_record, &selected_slot);
        if (s_previous_detail_valid && s_previous_hardfault &&
            s_previous_record.boot_id != hardfault_scratch.boot_id)
            s_previous_detail_valid = false;
    }

    if (s_previous_detail_valid) {
        s_boot_id = s_previous_record.boot_id + 1U;
        s_sequence = s_previous_record.sequence;
        s_active_slot = selected_slot;
    } else {
        if (s_previous_hardfault) {
            s_boot_id = hardfault_scratch.boot_id + 1U;
            s_sequence = hardfault_scratch.record_sequence;
        } else {
            s_boot_id = s_previous_coarse_valid ?
                (uint32_t)(uint8_t)(s_previous_boot_id_low + 1U) : 1U;
            s_sequence = 0U;
        }
        s_active_slot = 1U;
        invalidate_retained_records();
    }

    s_retained.hardfault.commit_marker = 0U;
    reset_diag_barrier();
    write_current_shutdown_record();
    commit_record(NULL);
}

void ResetDiag_Checkpoint(ResetDiagCheckpoint_t checkpoint)
{
    uint32_t primask;
    if (!checkpoint_valid((uint8_t)checkpoint)) return;
    primask = reset_diag_enter_critical();
    if (s_checkpoint != checkpoint) {
        s_checkpoint = checkpoint;
        shutdown_set(RESET_DIAG_SHUTDOWN_CHECKPOINT_INDEX,
                     (uint8_t)checkpoint);
    }
    reset_diag_exit_critical(primask);
}

void ResetDiag_RecordHealthyWindow(const ResetDiagHealth_t *health)
{
    if (health == NULL) return;
    commit_record(health);
}

void ResetDiag_GetSnapshot(ResetDiagSnapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->raw_cause = s_raw_cause;
    snapshot->reset_class = s_reset_class;
    snapshot->boot_id = s_boot_id;
    snapshot->checkpoint = s_checkpoint;
    snapshot->previous_coarse_valid = s_previous_coarse_valid;
    snapshot->previous_detail_valid = s_previous_detail_valid;
    snapshot->previous_hardfault = s_previous_hardfault;
    snapshot->previous_boot_id_low = s_previous_boot_id_low;
    snapshot->previous_checkpoint = s_previous_checkpoint;
    snapshot->previous_hardfault_pc = s_previous_hardfault_pc;
    snapshot->previous_hardfault_lr = s_previous_hardfault_lr;
    snapshot->previous_hardfault_xpsr = s_previous_hardfault_xpsr;
    if (!s_previous_detail_valid) return;
    snapshot->previous_sequence = s_previous_record.sequence;
    snapshot->previous_boot_id = s_previous_record.boot_id;
    snapshot->previous_time_ms = s_previous_record.time_ms;
    snapshot->previous_last_watchdog_service_ms =
        s_previous_record.last_watchdog_service_ms;
    snapshot->previous_scheduler_missed_total =
        s_previous_record.scheduler_missed_total;
    snapshot->previous_safety_fault_bits =
        s_previous_record.safety_fault_bits;
    snapshot->previous_uart_tx_dropped =
        s_previous_record.uart_tx_dropped;
    snapshot->previous_uart_rx_dropped =
        s_previous_record.uart_rx_dropped;
    snapshot->previous_telemetry_dropped =
        s_previous_record.telemetry_dropped;
    snapshot->previous_scheduler_pending_mask =
        s_previous_record.scheduler_pending_mask;
    snapshot->previous_scheduler_missed_mask =
        s_previous_record.scheduler_missed_mask;
    snapshot->previous_detail_checkpoint = s_previous_record.checkpoint;
    snapshot->previous_system_state = s_previous_record.system_state;
    snapshot->previous_telemetry_queued = s_previous_record.telemetry_queued;
    snapshot->previous_scheduler_health_mask =
        s_previous_record.scheduler_health_mask;
}

#ifndef RESET_DIAG_HOST_TEST
__attribute__((noreturn))
void ResetDiag_HardFaultCapture(const uint32_t *stacked_frame)
{
    uint32_t stacked_pc = 0U;
    uint32_t stacked_lr = 0U;
    uint32_t stacked_xpsr = 0U;
    uintptr_t address = (uintptr_t)stacked_frame;

    if ((address >= 0x20200000UL) &&
        (address <= (0x20208000UL - (8UL * sizeof(uint32_t)))) &&
        ((address & 0x3U) == 0U)) {
        stacked_lr = stacked_frame[5];
        stacked_pc = stacked_frame[6];
        stacked_xpsr = stacked_frame[7];
    }
    write_hardfault_scratch(stacked_pc, stacked_lr, stacked_xpsr);
    __DSB();
    __ISB();
    DL_SYSCTL_resetDevice(DL_SYSCTL_RESET_SYSRST);
    __DSB();
    __ISB();
    while (1) { }
}

void ResetDiag_HardFaultReset(void)
{
    ResetDiag_HardFaultCapture(NULL);
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile (
        "mrs r0, msp\n"
        "bl ResetDiag_HardFaultCapture\n"
        "b .\n");
}
#else
void ResetDiag_TestClearRetained(void)
{
    memset((void *)&s_retained, 0, sizeof(s_retained));
    memset(s_shutdown_storage, 0, sizeof(s_shutdown_storage));
}

uint8_t ResetDiag_TestGetShutdownByte(uint8_t index)
{
    return (index < 4U) ? s_shutdown_storage[index] : 0U;
}

void ResetDiag_TestSetShutdownByte(uint8_t index, uint8_t value)
{
    if (index < 4U) s_shutdown_storage[index] = value;
}

void ResetDiag_TestCorruptSlot(uint8_t slot, uint8_t offset, uint8_t xor_mask)
{
    volatile uint8_t *bytes;
    if (slot >= 2U || offset >= sizeof(ResetDiagRecordV1_t)) return;
    bytes = (volatile uint8_t *)&s_retained.slots[slot];
    bytes[offset] ^= xor_mask;
}

void ResetDiag_TestForceSequence(uint32_t sequence)
{
    s_sequence = sequence;
}

void ResetDiag_TestSimulateHardFault(void)
{
    write_hardfault_scratch(0x08001235UL, 0x08004567UL, 0x21000000UL);
}

void ResetDiag_TestSimulateEarlyHardFault(void)
{
    s_boot_id = 0U;
    s_sequence = 0U;
    write_hardfault_scratch(0x0800ABC1UL, 0x0800DEF3UL, 0x61000000UL);
}
#endif
