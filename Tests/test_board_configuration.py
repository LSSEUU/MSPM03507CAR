from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "Source" / "ti_msp_dl_config.h"
PROJECT = ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"


class FrozenBoardConfigurationTests(unittest.TestCase):
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

    def test_keil_target_uses_diagnostic_main_and_system_sources(self):
        root = ET.fromstring(PROJECT.read_text(encoding="utf-8"))
        paths = [node.text for node in root.findall(".//FilePath")]
        self.assertIn("../Source/main.c", paths)
        self.assertIn("../System/diagnostics.c", paths)
        self.assertIn("../System/diagnostics_health.c", paths)
        self.assertNotIn("../Source/empty.c", paths)
        self.assertEqual(root.findtext(".//pCCUsed"), "6220000::V6.22::ARMCLANG")

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
