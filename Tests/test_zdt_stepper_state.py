import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ZdtStepperStateTests(unittest.TestCase):
    def test_controller_c_suite(self) -> None:
        compiler = os.environ.get("CC", "gcc")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / "zdt-stepper-state-test.exe"
            subprocess.run([
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-IHardware", "-ISystem", "Tests/zdt_stepper_test_state_test.c",
                "System/zdt_stepper_test.c", "-o", str(executable),
            ], cwd=ROOT, check=True)
            subprocess.run([str(executable)], cwd=ROOT, check=True)


if __name__ == "__main__":
    unittest.main()
