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

    def test_project_has_no_machine_specific_tool_path(self):
        project = PROJECT_FILE.read_text(encoding="utf-8")
        self.assertNotIn(r"C:\Keil_v5", project)
        self.assertNotIn("D:/keil_5", project)


if __name__ == "__main__":
    unittest.main()
