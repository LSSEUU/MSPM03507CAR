#include "reset_diag.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

enum {
    RAW_NONE = 0x00U,
    RAW_POR_HW = 0x01U,
    RAW_POR_EXTERNAL = 0x02U,
    RAW_POR_SOFTWARE = 0x03U,
    RAW_BOR = 0x04U,
    RAW_SHUTDOWN_WAKE = 0x05U,
    RAW_PARITY = 0x08U,
    RAW_CLOCK = 0x09U,
    RAW_BOOT_EXTERNAL = 0x0CU,
    RAW_BOOT_SOFTWARE = 0x0DU,
    RAW_WWDT0 = 0x0EU,
    RAW_BSL_EXIT = 0x10U,
    RAW_BSL_ENTRY = 0x11U,
    RAW_WWDT1 = 0x13U,
    RAW_FLASH_ECC = 0x14U,
    RAW_CPU_LOCKUP = 0x15U,
    RAW_DEBUG_SYS = 0x1AU,
    RAW_SOFTWARE_SYS = 0x1BU,
    RAW_DEBUG_CPU = 0x1CU,
    RAW_SOFTWARE_CPU = 0x1DU
};

static ResetDiagHealth_t health(uint32_t now_ms)
{
    ResetDiagHealth_t result;
    memset(&result, 0, sizeof(result));
    result.now_ms = now_ms;
    result.last_watchdog_service_ms = now_ms - 1U;
    result.scheduler_missed_total = 7U;
    result.safety_fault_bits = 0x1234U;
    result.uart_tx_dropped = 3U;
    result.uart_rx_dropped = 4U;
    result.telemetry_dropped = 5U;
    result.scheduler_pending_mask = 0x02U;
    result.scheduler_missed_mask = 0x04U;
    result.checkpoint = RESET_DIAG_CHECKPOINT_TASK_20MS;
    result.system_state = 3U;
    result.telemetry_queued = 6U;
    result.scheduler_health_mask = 0x07U;
    return result;
}

static void test_reset_classification(void)
{
    assert(ResetDiag_Classify(RAW_NONE) == RESET_DIAG_CLASS_NONE);
    assert(ResetDiag_Classify(RAW_POR_HW) == RESET_DIAG_CLASS_POWER);
    assert(ResetDiag_Classify(RAW_POR_EXTERNAL) == RESET_DIAG_CLASS_EXTERNAL);
    assert(ResetDiag_Classify(RAW_POR_SOFTWARE) == RESET_DIAG_CLASS_SOFTWARE);
    assert(ResetDiag_Classify(RAW_BOR) == RESET_DIAG_CLASS_POWER);
    assert(ResetDiag_Classify(RAW_SHUTDOWN_WAKE) == RESET_DIAG_CLASS_POWER);
    assert(ResetDiag_Classify(RAW_PARITY) == RESET_DIAG_CLASS_HARDWARE_FAULT);
    assert(ResetDiag_Classify(RAW_CLOCK) == RESET_DIAG_CLASS_HARDWARE_FAULT);
    assert(ResetDiag_Classify(RAW_BOOT_EXTERNAL) == RESET_DIAG_CLASS_EXTERNAL);
    assert(ResetDiag_Classify(RAW_BOOT_SOFTWARE) == RESET_DIAG_CLASS_SOFTWARE);
    assert(ResetDiag_Classify(RAW_WWDT0) == RESET_DIAG_CLASS_WATCHDOG);
    assert(ResetDiag_Classify(RAW_WWDT1) == RESET_DIAG_CLASS_WATCHDOG);
    assert(ResetDiag_Classify(RAW_BSL_EXIT) == RESET_DIAG_CLASS_BSL);
    assert(ResetDiag_Classify(RAW_BSL_ENTRY) == RESET_DIAG_CLASS_BSL);
    assert(ResetDiag_Classify(RAW_FLASH_ECC) ==
           RESET_DIAG_CLASS_HARDWARE_FAULT);
    assert(ResetDiag_Classify(RAW_CPU_LOCKUP) ==
           RESET_DIAG_CLASS_HARDWARE_FAULT);
    assert(ResetDiag_Classify(RAW_DEBUG_SYS) == RESET_DIAG_CLASS_DEBUG);
    assert(ResetDiag_Classify(RAW_DEBUG_CPU) == RESET_DIAG_CLASS_DEBUG);
    assert(ResetDiag_Classify(RAW_SOFTWARE_SYS) == RESET_DIAG_CLASS_SOFTWARE);
    assert(ResetDiag_Classify(RAW_SOFTWARE_CPU) == RESET_DIAG_CLASS_SOFTWARE);
    assert(ResetDiag_Classify(0xAAU) == RESET_DIAG_CLASS_UNKNOWN);
    assert(strcmp(ResetDiag_RawName(RAW_POR_HW), "por_or_parity") == 0);
    assert(strcmp(ResetDiag_ClassName(RESET_DIAG_CLASS_WATCHDOG),
                  "watchdog") == 0);
}

static void test_por_invalidates_retained_history(void)
{
    ResetDiagSnapshot_t snapshot;

    ResetDiag_TestClearRetained();
    ResetDiag_EarlyInit(RAW_POR_HW);
    ResetDiag_GetSnapshot(&snapshot);
    assert(snapshot.raw_cause == RAW_POR_HW);
    assert(snapshot.boot_id == 1U);
    assert(snapshot.checkpoint == RESET_DIAG_CHECKPOINT_BOOT);
    assert(!snapshot.previous_coarse_valid);
    assert(!snapshot.previous_detail_valid);
    assert(ResetDiag_TestGetShutdownByte(0U) == 0xB6U);
    assert(ResetDiag_TestGetShutdownByte(1U) == 0x49U);
    assert(ResetDiag_TestGetShutdownByte(2U) ==
           (uint8_t)RESET_DIAG_CHECKPOINT_BOOT);
    assert(ResetDiag_TestGetShutdownByte(3U) == 1U);
}

static void test_reset_recovers_coarse_and_detailed_context(void)
{
    ResetDiagHealth_t summary;
    ResetDiagSnapshot_t snapshot;

    ResetDiag_TestClearRetained();
    ResetDiag_EarlyInit(RAW_POR_HW);
    summary = health(40U);
    ResetDiag_RecordHealthyWindow(&summary);
    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TASK_5MS);

    ResetDiag_EarlyInit(RAW_SOFTWARE_SYS);
    ResetDiag_GetSnapshot(&snapshot);
    assert(snapshot.raw_cause == RAW_SOFTWARE_SYS);
    assert(snapshot.reset_class == RESET_DIAG_CLASS_SOFTWARE);
    assert(snapshot.boot_id == 2U);
    assert(snapshot.previous_coarse_valid);
    assert(snapshot.previous_boot_id_low == 1U);
    assert(snapshot.previous_checkpoint == RESET_DIAG_CHECKPOINT_TASK_5MS);
    assert(snapshot.previous_detail_valid);
    assert(snapshot.previous_boot_id == 1U);
    assert(snapshot.previous_time_ms == 40U);
    assert(snapshot.previous_last_watchdog_service_ms == 39U);
    assert(snapshot.previous_scheduler_missed_total == 7U);
    assert(snapshot.previous_safety_fault_bits == 0x1234U);
    assert(snapshot.previous_uart_tx_dropped == 3U);
    assert(snapshot.previous_uart_rx_dropped == 4U);
    assert(snapshot.previous_telemetry_dropped == 5U);
    assert(snapshot.previous_scheduler_pending_mask == 0x02U);
    assert(snapshot.previous_scheduler_missed_mask == 0x04U);
    assert(snapshot.previous_system_state == 3U);
    assert(snapshot.previous_telemetry_queued == 6U);
    assert(snapshot.previous_scheduler_health_mask == 0x07U);

    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_IDLE);
    ResetDiag_GetSnapshot(&snapshot);
    assert(snapshot.raw_cause == RAW_SOFTWARE_SYS);
    assert(snapshot.checkpoint == RESET_DIAG_CHECKPOINT_IDLE);
}

static void test_corrupt_newest_slot_falls_back(void)
{
    ResetDiagHealth_t summary;
    ResetDiagSnapshot_t snapshot;

    ResetDiag_TestClearRetained();
    ResetDiag_EarlyInit(RAW_POR_HW);
    summary = health(20U);
    ResetDiag_RecordHealthyWindow(&summary);
    summary = health(40U);
    ResetDiag_RecordHealthyWindow(&summary);
    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_IDLE);

    /* Initial boot record uses slot 0; two commits use slot 1 then slot 0. */
    ResetDiag_TestCorruptSlot(0U, 16U, 0x01U);
    ResetDiag_EarlyInit(RAW_WWDT0);
    ResetDiag_GetSnapshot(&snapshot);
    assert(snapshot.previous_detail_valid);
    assert(snapshot.previous_time_ms == 20U);
    assert(snapshot.previous_checkpoint == RESET_DIAG_CHECKPOINT_IDLE);
}

static void test_sequence_wrap_selects_newest(void)
{
    ResetDiagHealth_t summary;
    ResetDiagSnapshot_t snapshot;

    ResetDiag_TestClearRetained();
    ResetDiag_EarlyInit(RAW_POR_HW);
    ResetDiag_TestForceSequence(UINT32_MAX - 1U);
    summary = health(60U);
    ResetDiag_RecordHealthyWindow(&summary);
    summary = health(80U);
    ResetDiag_RecordHealthyWindow(&summary);
    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_IDLE);
    ResetDiag_EarlyInit(RAW_WWDT1);
    ResetDiag_GetSnapshot(&snapshot);
    assert(snapshot.previous_detail_valid);
    assert(snapshot.previous_sequence == 0U);
    assert(snapshot.previous_time_ms == 80U);
}

static void test_boot_id_mismatch_rejects_detail(void)
{
    ResetDiagHealth_t summary;
    ResetDiagSnapshot_t snapshot;

    ResetDiag_TestClearRetained();
    ResetDiag_EarlyInit(RAW_POR_HW);
    summary = health(20U);
    ResetDiag_RecordHealthyWindow(&summary);
    ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_TASK_10MS);
    ResetDiag_TestSetShutdownByte(3U, 7U);

    ResetDiag_EarlyInit(RAW_SOFTWARE_CPU);
    ResetDiag_GetSnapshot(&snapshot);
    assert(snapshot.previous_coarse_valid);
    assert(snapshot.previous_boot_id_low == 7U);
    assert(!snapshot.previous_detail_valid);
    assert(snapshot.boot_id == 8U);
}

static void test_hardfault_scratch_survives_software_reset(void)
{
    ResetDiagSnapshot_t snapshot;

    ResetDiag_TestClearRetained();
    ResetDiag_EarlyInit(RAW_POR_HW);
    ResetDiag_TestSimulateHardFault();
    ResetDiag_EarlyInit(RAW_SOFTWARE_SYS);
    ResetDiag_GetSnapshot(&snapshot);
    assert(snapshot.previous_coarse_valid);
    assert(snapshot.previous_checkpoint == RESET_DIAG_CHECKPOINT_HARDFAULT);
    assert(snapshot.previous_hardfault);
    assert(snapshot.previous_hardfault_pc == 0x08001235UL);
    assert(snapshot.previous_hardfault_lr == 0x08004567UL);
    assert(snapshot.previous_hardfault_xpsr == 0x21000000UL);
}

static void test_hardfault_scratch_survives_without_detail_slots(void)
{
    ResetDiagSnapshot_t snapshot;

    ResetDiag_TestClearRetained();
    ResetDiag_EarlyInit(RAW_POR_HW);
    ResetDiag_TestSimulateHardFault();
    ResetDiag_TestCorruptSlot(0U, 0U, 0x01U);
    ResetDiag_TestCorruptSlot(1U, 0U, 0x01U);
    ResetDiag_EarlyInit(RAW_SOFTWARE_SYS);
    ResetDiag_GetSnapshot(&snapshot);
    assert(!snapshot.previous_detail_valid);
    assert(snapshot.previous_hardfault);
    assert(snapshot.previous_hardfault_pc == 0x08001235UL);
    assert(snapshot.previous_hardfault_lr == 0x08004567UL);
    assert(snapshot.previous_hardfault_xpsr == 0x21000000UL);
}

static void test_early_hardfault_builds_self_contained_record(void)
{
    ResetDiagSnapshot_t snapshot;

    ResetDiag_TestClearRetained();
    ResetDiag_TestSimulateEarlyHardFault();
    assert(ResetDiag_TestGetShutdownByte(0U) == 0xB6U);
    assert(ResetDiag_TestGetShutdownByte(1U) == 0x49U);
    assert(ResetDiag_TestGetShutdownByte(2U) ==
           (uint8_t)RESET_DIAG_CHECKPOINT_HARDFAULT);
    assert(ResetDiag_TestGetShutdownByte(3U) == 0U);

    ResetDiag_EarlyInit(RAW_SOFTWARE_SYS);
    ResetDiag_GetSnapshot(&snapshot);
    assert(snapshot.previous_coarse_valid);
    assert(!snapshot.previous_detail_valid);
    assert(snapshot.previous_hardfault);
    assert(snapshot.previous_checkpoint == RESET_DIAG_CHECKPOINT_HARDFAULT);
    assert(snapshot.previous_hardfault_pc == 0x0800ABC1UL);
    assert(snapshot.previous_hardfault_lr == 0x0800DEF3UL);
    assert(snapshot.previous_hardfault_xpsr == 0x61000000UL);
    assert(snapshot.boot_id == 1U);
}

static void test_hardfault_scratch_recovers_if_coarse_commit_tears(void)
{
    ResetDiagSnapshot_t snapshot;

    ResetDiag_TestClearRetained();
    ResetDiag_TestSimulateEarlyHardFault();
    ResetDiag_TestSetShutdownByte(1U, 0U);
    ResetDiag_EarlyInit(RAW_SOFTWARE_SYS);
    ResetDiag_GetSnapshot(&snapshot);
    assert(!snapshot.previous_coarse_valid);
    assert(snapshot.previous_hardfault);
    assert(snapshot.previous_checkpoint == RESET_DIAG_CHECKPOINT_HARDFAULT);
    assert(snapshot.previous_hardfault_pc == 0x0800ABC1UL);
    assert(snapshot.boot_id == 1U);
}

int main(void)
{
    test_reset_classification();
    test_por_invalidates_retained_history();
    test_reset_recovers_coarse_and_detailed_context();
    test_corrupt_newest_slot_falls_back();
    test_sequence_wrap_selects_newest();
    test_boot_id_mismatch_rejects_detail();
    test_hardfault_scratch_survives_software_reset();
    test_hardfault_scratch_survives_without_detail_slots();
    test_early_hardfault_builds_self_contained_record();
    test_hardfault_scratch_recovers_if_coarse_commit_tears();
    return 0;
}
