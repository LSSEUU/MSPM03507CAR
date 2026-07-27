from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PROJECT_FILE = ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
SCATTER_FILE = ROOT / "keil" / "mspm0g3507.sct"


class KeilProjectPortabilityTests(unittest.TestCase):
    def test_hex_is_generated_without_a_mandatory_after_build_command(self):
        project = PROJECT_FILE.read_text(encoding="utf-8")
        root = ET.fromstring(project)
        after_make = root.find(".//AfterMake")

        self.assertNotIn(r"E:\MyFile", project)
        self.assertNotIn(r"D:\keil_5", project)
        self.assertEqual(root.findtext(".//CreateHexFile"), "1")
        self.assertIsNotNone(after_make)
        self.assertEqual(after_make.findtext("RunUserProg1"), "0")
        self.assertFalse(after_make.findtext("UserProg1Name"))

    def test_scatter_file_has_no_empty_padding_region(self):
        scatter = SCATTER_FILE.read_text(encoding="utf-8")

        self.assertNotIn("ER_IROM1_PAD", scatter)
        self.assertIn("LR_IROM1 0x00000000 0x0001F800", scatter)
        self.assertIn("ER_IROM1 0x00000000 ALIGNALL 8 0x0001F800", scatter)

    def test_project_describes_flash_and_reserves_parameter_sectors(self):
        root = ET.fromstring(PROJECT_FILE.read_text(encoding="utf-8"))

        # Keil refreshes the physical device memory from the installed DFP.
        # Keep the full 128 KiB here; OCR_RVCT4 and the scatter file define
        # the smaller application link region that excludes the final 2 KiB.
        self.assertEqual(
            int(root.findtext(".//OnChipMemories/IROM/Size"), 0), 0x20000
        )
        self.assertEqual(
            int(root.findtext(".//OnChipMemories/OCR_RVCT4/Size"), 0), 0x1F800
        )

    def test_project_has_no_machine_specific_tool_path(self):
        project = PROJECT_FILE.read_text(encoding="utf-8")
        self.assertNotIn(r"C:\Keil_v5", project)
        self.assertNotIn("D:/keil_5", project)


if __name__ == "__main__":
    unittest.main()
