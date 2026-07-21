from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class DiagnosticsHealthTests(unittest.TestCase):
    def test_health_model_compiles_and_passes(self):
        compiler = shutil.which("gcc")
        self.assertIsNotNone(compiler, "gcc is required for host C regression tests")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "diagnostics_health_test.exe"
            compile_result = subprocess.run(
                [
                    compiler,
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(ROOT / "System"),
                    str(ROOT / "System" / "diagnostics_health.c"),
                    str(ROOT / "Tests" / "diagnostics_health_test.c"),
                    "-o",
                    str(executable),
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(compile_result.returncode, 0, compile_result.stderr)
            run_result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run_result.returncode, 0, run_result.stderr)


if __name__ == "__main__":
    unittest.main()
