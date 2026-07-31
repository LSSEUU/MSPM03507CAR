from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class AppH3Tests(unittest.TestCase):
    def test_single_and_double_target_flows_and_guards(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        for single_target in (1, 0):
            with self.subTest(single_target=single_target), tempfile.TemporaryDirectory() as directory:
                executable = Path(directory) / "app_h3_test.exe"
                command = [
                    compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-DH3_DEVELOPMENT_ENTRY=1",
                    f"-DH3_SINGLE_TARGET_ONLY={single_target}",
                    "-I", str(ROOT / "App"), "-I", str(ROOT / "Config"),
                    "-I", str(ROOT / "Control"), "-I", str(ROOT / "Hardware"),
                    "-I", str(ROOT / "System"),
                    str(ROOT / "Tests" / "app_h3_test.c"),
                    str(ROOT / "App" / "app_h3.c"),
                    str(ROOT / "Control" / "ball_pd.c"),
                    "-o", str(executable),
                ]
                compiled = subprocess.run(command, capture_output=True, text=True)
                self.assertEqual(compiled.returncode, 0, compiled.stderr)
                result = subprocess.run([str(executable)], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
