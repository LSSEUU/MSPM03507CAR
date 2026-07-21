from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROJECT_FILE = ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
SCATTER_FILE = ROOT / "keil" / "mspm0g3507.sct"


class KeilProjectPortabilityTests(unittest.TestCase):
    def test_after_build_command_has_no_machine_specific_paths(self):
        project = PROJECT_FILE.read_text(encoding="utf-8")

        self.assertNotIn(r"E:\MyFile", project)
        self.assertNotIn(r"D:\keil_5", project)
        self.assertIn(
            'python ".\\pad_hex.py" '
            '".\\Objects\\empty_LP_MSPM0G3507_nortos_keil.axf"',
            project,
        )

    def test_scatter_file_has_no_empty_padding_region(self):
        scatter = SCATTER_FILE.read_text(encoding="utf-8")

        self.assertNotIn("ER_IROM1_PAD", scatter)


if __name__ == "__main__":
    unittest.main()
