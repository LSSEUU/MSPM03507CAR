import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class GrayscaleI2CDriverTests(unittest.TestCase):
    def _compile_and_run(self, orientation):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "grayscale_i2c_test.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-DGRAYSCALE_IR1_ON_RIGHT={orientation}U",
                "-I",
                str(ROOT / "Tests" / "mocks"),
                "-I",
                str(ROOT / "Hardware"),
                "-I",
                str(ROOT / "Config"),
                str(ROOT / "Tests" / "grayscale_i2c_driver_test.c"),
                str(ROOT / "Hardware" / "grayscale.c"),
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_ir1_on_right_mapping_and_transport_failures(self):
        self._compile_and_run(1)

    def test_orientation_switch_reverses_mapping(self):
        self._compile_and_run(0)


if __name__ == "__main__":
    unittest.main()
