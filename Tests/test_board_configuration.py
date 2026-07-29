from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "Source" / "ti_msp_dl_config.h"
CONFIG_SOURCE = ROOT / "Source" / "ti_msp_dl_config.c"
PROJECT = ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"


class FrozenBoardConfigurationTests(unittest.TestCase):
    def test_pb11_nec_capture_is_frozen_without_uart_or_motor_remap(self):
        header = (ROOT / "Source" / "ti_msp_dl_config.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "Source" / "ti_msp_dl_config.c").read_text(
            encoding="utf-8"
        )
        project = (
            ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
        ).read_text(encoding="utf-8")
        receiver = (ROOT / "Hardware" / "ir_receiver.c").read_text(
            encoding="utf-8"
        )
        remote_config = (ROOT / "Config" / "remote_config.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("IR_CAPTURE_INST                                                   TIMG0", header)
        self.assertIn("IR_CAPTURE_PIN                                           DL_GPIO_PIN_11", header)
        self.assertIn("IOMUX_PINCM28_PF_TIMG0_CCP1", header)
        self.assertIn("DL_TIMER_CLOCK_DIVIDE_8", source)
        self.assertIn("DL_TIMER_CAPTURE_MODE_EDGE_TIME", source)
        self.assertIn("DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_EDGE", source)
        self.assertIn("DL_TIMER_INPUT_CHAN_1", source)
        self.assertIn("../Hardware/ir_receiver.c", project)
        self.assertIn("../Hardware/ir_nec.c", project)
        self.assertIn("../App/app_remote.c", project)
        self.assertIn("IOMUX_PINCM22_PF_UART0_RX", header)
        self.assertIn("IOMUX_PINCM47_PF_UART2_RX", header)
        self.assertIn("IOMUX_PINCM15_PF_TIMA1_CCP0", header)
        self.assertIn("#define IR_REMOTE_KEYMAP_VERIFIED              1", remote_config)
        measured_keys = {
            "UP": "0x18U", "DOWN": "0x52U", "LEFT": "0x08U",
            "RIGHT": "0x5AU", "OK": "0x1CU", "STAR": "0x16U",
            "HASH": "0x0DU", "0": "0x19U", "1": "0x45U",
            "2": "0x46U", "3": "0x47U", "4": "0x44U",
            "5": "0x40U", "6": "0x43U", "7": "0x07U",
            "8": "0x15U", "9": "0x09U",
        }
        for key, command in measured_keys.items():
            self.assertRegex(
                remote_config,
                rf"(?m)^#define\s+IR_REMOTE_KEY_{re.escape(key)}\s+{re.escape(command)}$",
            )
        self.assertIn("ir_phase_level_high = !ir_phase_level_high", receiver)
        self.assertNotIn(
            "bool level_high = (DL_GPIO_readPins", receiver
        )

    def test_no_stale_sysconfig_source_can_overwrite_frozen_configuration(self):
        syscfg_sources = list((ROOT / "Source").glob("*.syscfg"))
        syscfg_sources += list((ROOT / "syscfg").glob("*.syscfg"))
        self.assertEqual(syscfg_sources, [])
        self.assertIn("FROZEN_BOARD_CONFIG_REVISION", HEADER.read_text(encoding="utf-8"))

    def test_verified_pin_invariants_are_recorded(self):
        header = HEADER.read_text(encoding="utf-8")
        expected = [
            "GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10",
            "GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11",
            "GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6",
            "GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7",
            "GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_21",
            "GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_22",
            "GPIO_UART_3_TX_PIN                                        DL_GPIO_PIN_12",
            "GPIO_UART_3_RX_PIN                                        DL_GPIO_PIN_13",
            "ENC_A_E1A_PIN                                            (DL_GPIO_PIN_8)",
            "ENC_A_E1B_PIN                                            (DL_GPIO_PIN_9)",
            "ENC_B_E2A_PIN                                            (DL_GPIO_PIN_9)",
            "ENC_B_E2B_PIN                                            (DL_GPIO_PIN_8)",
            "OLED_SDA_PIN                                            (DL_GPIO_PIN_28)",
            "OLED_SCL_PIN                                            (DL_GPIO_PIN_31)",
            "KEY1_PIN                                                (DL_GPIO_PIN_18)",
            "KEY1_IOMUX                                              (IOMUX_PINCM44)",
            "BUZZER_PIN                                              (DL_GPIO_PIN_12)",
            "BUZZER_IOMUX                                            (IOMUX_PINCM34)",
            "BOARD_STATUS_LED_COUNT                                             (0U)",
        ]
        for invariant in expected:
            self.assertIn(invariant, header)

    def test_bsl_and_swd_pins_are_not_reconfigured_by_application(self):
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        header = HEADER.read_text(encoding="utf-8")
        combined = config + "\n" + header
        # PA18 is the default BSL invocation pin; PA19/PA20 are SWDIO/SWCLK.
        for pincm in ("IOMUX_PINCM40", "IOMUX_PINCM41", "IOMUX_PINCM42"):
            self.assertNotIn(pincm, combined)

    def test_grayscale_uses_pb1_address_and_pa25_adc_with_bounded_scanning(self):
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        driver = (ROOT / "Hardware" / "grayscale.c").read_text(encoding="utf-8")
        header = HEADER.read_text(encoding="utf-8")
        self.assertIn("DL_ADC12_initSingleSample(GRAY_ADC_INST", config)
        self.assertIn("DL_ADC12_enablePower(GRAY_ADC_INST)", config)
        self.assertIn("DL_ADC12_INPUT_CHAN_2", header)
        self.assertIn("DL_ADC12_getRawInterruptStatus", driver)
        self.assertIn("GRAY_ADC_TIMEOUT_LOOPS", driver)
        self.assertIn("static bool Gray_ReadADC", driver)
        self.assertNotIn("DL_GPIO_initDigitalInputFeatures(GRAY_OUT_IOMUX", driver)
        self.assertRegex(header, r"#define\s+GRAY_AD0_PORT\s+\(GPIOB\)")
        self.assertRegex(header, r"#define\s+GRAY_AD0_PIN\s+\(DL_GPIO_PIN_1\)")
        self.assertRegex(header, r"#define\s+GRAY_AD0_IOMUX\s+\(IOMUX_PINCM13\)")
        self.assertIn("DL_GPIO_initDigitalOutput(GRAY_AD0_IOMUX)", driver)
        self.assertIn("DL_GPIO_clearPins(GRAY_AD0_PORT, GRAY_AD0_PIN)", driver)
        self.assertIn("DL_GPIO_enableOutput(GRAY_AD0_PORT, GRAY_AD0_PIN)", driver)
        self.assertNotIn("GRAY_AD0_PIN | GRAY_AD1_PIN", driver)
        self.assertNotIn("DL_GPIO_enableHiZ(GRAY_AD0_IOMUX)", driver)
        self.assertNotIn("DL_GPIO_RESISTOR_PULL_UP", driver)
        self.assertNotIn("GRAY_ASYNC_WAIT_AD0", driver)
        self.assertNotIn("GRAY_AD0_RISE_TIMEOUT_MS", driver)
        self.assertNotIn("IOMUX_PINCM53", header + config + driver)
        self.assertIn("0U, 2U, 4U, 6U, 1U, 3U, 5U, 7U", driver)
        self.assertNotIn("while (false ==", driver)

    def test_uart2_is_receive_only_to_release_pa21_vref_minus(self):
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        self.assertIn(".direction   = DL_UART_MAIN_DIRECTION_RX", config)
        self.assertIn("DL_GPIO_initDigitalInputFeatures(GPIO_UART_2_TX_IOMUX", config)
        self.assertNotIn(
            "DL_GPIO_initPeripheralOutputFunction(\n"
            "        GPIO_UART_2_TX_IOMUX",
            config,
        )

    def test_keil_target_uses_diagnostic_main_and_system_sources(self):
        root = ET.fromstring(PROJECT.read_text(encoding="utf-8"))
        paths = [node.text for node in root.findall(".//FilePath")]
        include_paths = root.findtext(".//Cads/VariousControls/IncludePath")
        self.assertIn("../Source/main.c", paths)
        self.assertIn("../System/diagnostics.c", paths)
        self.assertIn("../System/diagnostics_health.c", paths)
        self.assertIn("../System/system_time.c", paths)
        self.assertIn("../System/scheduler.c", paths)
        self.assertIn("../System/system_state.c", paths)
        self.assertIn("../System/safety.c", paths)
        self.assertIn("../System/m6_wire.c", paths)
        self.assertIn("../System/m6_link.c", paths)
        self.assertIn("../System/m6_rpc.c", paths)
        self.assertIn("../System/m6_vision_uart.c", paths)
        self.assertIn("../System/m6_action_validate.c", paths)
        self.assertIn("../Hardware/watchdog.c", paths)
        self.assertIn("../Hardware/vision_uart.c", paths)
        self.assertIn("../Hardware/zdt_stepper_uart.c", paths)
        self.assertIn("../Hardware/zdt_stepper.c", paths)
        self.assertIn("../Hardware/key.c", paths)
        self.assertIn("../Hardware/buzzer.c", paths)
        self.assertIn("../App/app_manual_drive.c", paths)
        self.assertIn("../Control/pid.c", paths)
        self.assertIn("../Control/filter.c", paths)
        self.assertIn("../Control/wheel_speed.c", paths)
        self.assertIn("../Config", include_paths)
        self.assertIn("../Control", include_paths)
        self.assertNotIn("../Source/empty.c", paths)
        self.assertFalse((ROOT / "Source" / "empty.c").exists())
        self.assertEqual(root.findtext(".//pCCUsed"), "6220000::V6.22::ARMCLANG")

    def test_uart1_is_reserved_for_camera_transport_only(self):
        header = HEADER.read_text(encoding="utf-8")
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        driver = (ROOT / "Hardware" / "vision_uart.c").read_text(encoding="utf-8")
        self.assertIn("IOMUX_PINCM23_PF_UART1_TX", header)
        self.assertIn("IOMUX_PINCM24_PF_UART1_RX", header)
        self.assertIn("UART_1_BAUD_RATE", header)
        self.assertIn("DL_UART_MAIN_DIRECTION_TX_RX", config)
        self.assertIn("DL_UART_RX_FIFO_LEVEL_1_2_FULL", config)
        self.assertIn("void UART1_IRQHandler(void)", driver)
        self.assertIn("DL_UART_INTERRUPT_RX_TIMEOUT_ERROR", driver)
        self.assertIn("VISION_UART_RX_BUFFER_SIZE 256U", driver)
        self.assertIn("VISION_UART_TX_BUFFER_SIZE 256U", driver)
        self.assertNotIn("AppManualDrive", driver)

    def test_uart3_is_reserved_for_zdt_stepper_transport(self):
        header = HEADER.read_text(encoding="utf-8")
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        transport = (ROOT / "Hardware" / "zdt_stepper_uart.c").read_text(
            encoding="utf-8")
        protocol = (ROOT / "Hardware" / "zdt_stepper.c").read_text(
            encoding="utf-8")
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")

        self.assertIn("FROZEN_BOARD_CONFIG_REVISION                         (20260729U)", header)
        self.assertIn("IOMUX_PINCM29_PF_UART3_TX", header)
        self.assertIn("IOMUX_PINCM30_PF_UART3_RX", header)
        self.assertIn("UART_3_BAUD_RATE", header)
        self.assertIn("UART_3_IBRD_32_MHZ_115200_BAUD", header)
        self.assertIn("DL_UART_Main_reset(UART_3_INST)", config)
        self.assertIn("DL_UART_Main_enablePower(UART_3_INST)", config)
        self.assertIn("DL_UART_MAIN_DIRECTION_TX_RX", config)
        self.assertIn("DL_UART_RX_FIFO_LEVEL_1_2_FULL", config)
        self.assertIn("void UART3_IRQHandler(void)", transport)
        self.assertIn("DL_UART_INTERRUPT_RX_TIMEOUT_ERROR", transport)
        self.assertNotIn("while (DL_UART_isBusy", transport)
        self.assertIn("ZdtStepperUart_WriteEmergencyBatch", transport)
        self.assertIn("ZdtStepperUart_Write(frame, length)", protocol)
        self.assertIn("ZdtStepperTest_Init(SystemTime_Now());", main)
        self.assertIn("ZdtStepperTest_Poll(SystemTime_Now());", main)
        self.assertIn("ZdtStepper_EmergencyStop(true)",
                      (ROOT / "System" / "zdt_stepper_test.c").read_text(
                          encoding="utf-8"))

    def test_key1_drives_active_high_buzzer_for_board_validation(self):
        header = HEADER.read_text(encoding="utf-8")
        key_header = (ROOT / "Hardware" / "key.h").read_text(encoding="utf-8")
        key_driver = (ROOT / "Hardware" / "key.c").read_text(encoding="utf-8")
        buzzer_header = (ROOT / "Hardware" / "buzzer.h").read_text(
            encoding="utf-8")
        buzzer_driver = (ROOT / "Hardware" / "buzzer.c").read_text(
            encoding="utf-8")
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")

        self.assertIn("FROZEN_BOARD_CONFIG_REVISION                         (20260729U)", header)
        self.assertRegex(header, r"#define\s+KEY1_PORT\s+\(GPIOB\)")
        self.assertRegex(header, r"#define\s+KEY1_PIN\s+\(DL_GPIO_PIN_18\)")
        self.assertRegex(header, r"#define\s+KEY1_IOMUX\s+\(IOMUX_PINCM44\)")
        self.assertRegex(header, r"#define\s+BUZZER_PORT\s+\(GPIOA\)")
        self.assertRegex(header, r"#define\s+BUZZER_PIN\s+\(DL_GPIO_PIN_12\)")
        self.assertRegex(header, r"#define\s+BUZZER_IOMUX\s+\(IOMUX_PINCM34\)")

        self.assertIn("void Key_Init(void);", key_header)
        self.assertIn("bool Key1_IsPressed(void);", key_header)
        self.assertIn("DL_GPIO_initDigitalInput(KEY1_IOMUX)", key_driver)
        self.assertIn("DL_GPIO_readPins(KEY1_PORT, KEY1_PIN)", key_driver)
        self.assertNotIn("DL_GPIO_RESISTOR", key_driver)

        for declaration in (
            "void Buzzer_Init(void);", "void Buzzer_On(void);",
            "void Buzzer_Off(void);",
        ):
            self.assertIn(declaration, buzzer_header)
        preload = buzzer_driver.index(
            "DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);")
        configure = buzzer_driver.index(
            "DL_GPIO_initDigitalOutput(BUZZER_IOMUX);")
        enable = buzzer_driver.index(
            "DL_GPIO_enableOutput(BUZZER_PORT, BUZZER_PIN);")
        self.assertLess(preload, configure)
        self.assertLess(configure, enable)
        self.assertIn("DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);",
                      buzzer_driver)

        syscfg = main.index("SYSCFG_DL_init();")
        buzzer_init = main.index("Buzzer_Init();")
        key_init = main.index("Key_Init();")
        checkpoint = main.index(
            "ResetDiag_Checkpoint(RESET_DIAG_CHECKPOINT_INIT);")
        self.assertLess(syscfg, buzzer_init)
        self.assertLess(buzzer_init, key_init)
        self.assertLess(key_init, checkpoint)
        self.assertIn("if (Key1_IsPressed()) Buzzer_On();", main)
        self.assertIn("else                  Buzzer_Off();", main)

    def test_watchdog_and_scheduler_clock_start_after_boot_diagnostics(self):
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        diagnostics = main.index("Diagnostics_RunBootSelfTest();")
        watchdog = main.index("Watchdog_Init();")
        systick = main.index("DL_SYSTICK_enable();")
        loop = main.index("while (1)")
        self.assertLess(diagnostics, watchdog)
        self.assertLess(watchdog, systick)
        self.assertLess(systick, loop)

    def test_grayscale_api_exposes_adc_scans_without_stale_line_state(self):
        header = (ROOT / "Hardware" / "grayscale.h").read_text(encoding="utf-8")
        self.assertIn("bool Grayscale_Scan(void)", header)
        self.assertIn("uint16_t adc[GRAYSCALE_CHANNEL_COUNT]", header)
        self.assertIn("bool adc_powered", header)
        for stale_name in (
            "raw_state", "filtered_state", "position", "debounce_cnt",
            "min_bits", "Grayscale_SetMinBits", "extern Grayscale_t Gray",
        ):
            self.assertNotIn(stale_name, header)

    def test_scripts_and_vscode_have_no_machine_specific_legacy_paths(self):
        files = list((ROOT / "scripts").glob("*")) + list((ROOT / ".vscode").glob("*"))
        combined = "\n".join(
            path.read_text(encoding="utf-8", errors="ignore") for path in files if path.is_file()
        )
        self.assertNotIn(r"C:\Keil_v5", combined)
        self.assertNotIn("D:/keil_5", combined)
        self.assertNotIn("openocd", combined.lower())

        build_script = (ROOT / "scripts" / "build.ps1").read_text(encoding="utf-8")
        self.assertIn("Remove-Item -LiteralPath $buildLog", build_script)
        self.assertIn("-Wait -PassThru", build_script)


if __name__ == "__main__":
    unittest.main()
