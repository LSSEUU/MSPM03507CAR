from hashlib import sha256
from pathlib import Path
import unittest
import zipfile


ROOT = Path(__file__).resolve().parents[1]
DEMO_UART = ROOT / "demo_uart"
MANIFEST = DEMO_UART / "app.yaml"
PACKAGE = DEMO_UART / "dist" / "maix-balls-v1.3.0.zip"

MODEL_HASHES = {
    "models/yolo26_all_maixcam_yolo26_480_160/yolo26_all.cvimodel": (
        "4D8EA9568FAC2536EA125B0301CFDA3B272DF6E933F0D331CECEB505F2D40D3F"
    ),
    "models/yolo26_all_maixcam_yolo26_480_160/yolo26_all.mud": (
        "2CB951F25AAF1760C2CCDD68E63CA4235155F78DE454CE63091C2020135EF13E"
    ),
    "models/yolo26_all_maixcam_yolo26_480_160/yolo26_all_tensor_info.txt": (
        "799D9A18510548546B7E5C18302C7B28AB9427301D73C7811638C92B6CAC27BD"
    ),
}


def manifest_files(text):
    files = []
    in_files = False
    for line in text.splitlines():
        if line.strip() == "files:":
            in_files = True
            continue
        if in_files and line.startswith("  - "):
            files.append(line[4:].strip().replace("\\", "/"))
    return files


class DemoUartPackageTests(unittest.TestCase):
    def test_manifest_is_v130_and_every_declared_file_exists(self):
        text = MANIFEST.read_text(encoding="utf-8")
        self.assertIn("id: balls", text)
        self.assertIn("version: 1.3.0", text)
        files = manifest_files(text)
        self.assertIn("hybrid_tracker.py", files)
        self.assertNotIn("README.md", files)
        self.assertFalse(any("maixcam2" in name.lower() for name in files))
        for name in files:
            with self.subTest(name=name):
                self.assertTrue((DEMO_UART / Path(name)).is_file())

    def test_frozen_maixcam_model_hashes_are_unchanged(self):
        for name, expected in MODEL_HASHES.items():
            with self.subTest(name=name):
                digest = sha256((DEMO_UART / Path(name)).read_bytes()).hexdigest()
                self.assertEqual(digest.upper(), expected)

    def test_v130_archive_matches_manifest_and_source_bytes(self):
        declared = set(manifest_files(MANIFEST.read_text(encoding="utf-8")))
        self.assertTrue(PACKAGE.is_file())
        with zipfile.ZipFile(PACKAGE, "r") as archive:
            archived = set(archive.namelist())
            self.assertEqual(archived, declared)
            self.assertIsNone(archive.testzip())
            for name in sorted(declared):
                with self.subTest(name=name):
                    self.assertEqual(
                        archive.read(name),
                        (DEMO_UART / Path(name)).read_bytes(),
                    )


if __name__ == "__main__":
    unittest.main()
