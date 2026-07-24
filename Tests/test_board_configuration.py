from pathlib import Path
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
        self.assertIn("#define IR_REMOTE_KEYMAP_VERIFIED              0", remote_config)

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
            "GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_21",
            "GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_22",
            "ENC_A_E1A_PIN                                            (DL_GPIO_PIN_8)",
            "ENC_A_E1B_PIN                                            (DL_GPIO_PIN_9)",
            "ENC_B_E2A_PIN                                            (DL_GPIO_PIN_9)",
            "ENC_B_E2B_PIN                                            (DL_GPIO_PIN_8)",
            "OLED_SDA_PIN                                            (DL_GPIO_PIN_28)",
            "OLED_SCL_PIN                                            (DL_GPIO_PIN_31)",
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

    def test_grayscale_uses_pa25_digital_input_with_bounded_scanning(self):
        config = CONFIG_SOURCE.read_text(encoding="utf-8")
        driver = (ROOT / "Hardware" / "grayscale.c").read_text(encoding="utf-8")
        combined = config + "\n" + HEADER.read_text(encoding="utf-8")
        self.assertNotIn("GRAY_ADC", combined)
        self.assertNotIn("DL_ADC12_", driver)
        self.assertIn("DL_GPIO_initDigitalInputFeatures(GRAY_OUT_IOMUX", driver)
        self.assertIn("DL_GPIO_RESISTOR_NONE", driver)
        self.assertIn("GRAY_CH_SWITCH_DELAY_CYCLES 1600U", driver)
        self.assertIn("static uint8_t Gray_ReadDigital", driver)
        self.assertIn("DL_GPIO_initDigitalInputFeatures(GRAY_AD0_IOMUX", driver)
        self.assertIn("DL_GPIO_enableHiZ(GRAY_AD0_IOMUX)", driver)
        self.assertIn("DL_GPIO_RESISTOR_PULL_UP", driver)
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
        self.assertIn("../Hardware/watchdog.c", paths)
        self.assertIn("../Control/pid.c", paths)
        self.assertIn("../Control/filter.c", paths)
        self.assertIn("../Control/wheel_speed.c", paths)
        self.assertIn("../Config", include_paths)
        self.assertIn("../Control", include_paths)
        self.assertNotIn("../Source/empty.c", paths)
        self.assertFalse((ROOT / "Source" / "empty.c").exists())
        self.assertEqual(root.findtext(".//pCCUsed"), "6220000::V6.22::ARMCLANG")

    def test_grayscale_api_exposes_digital_scans_without_stale_line_state(self):
        header = (ROOT / "Hardware" / "grayscale.h").read_text(encoding="utf-8")
        self.assertIn("bool Grayscale_Scan(void)", header)
        self.assertIn("uint8_t level[GRAYSCALE_CHANNEL_COUNT]", header)
        self.assertIn("uint8_t high_mask", header)
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
