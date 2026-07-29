from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def test_vision_uart_monitor_preserves_bytes_and_drops_old_backlog() -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"
    with tempfile.TemporaryDirectory() as directory:
        executable = Path(directory) / "m6_vision_uart_test.exe"
        command = [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I", str(ROOT / "System"),
            "-I", str(ROOT / "Hardware"),
            str(ROOT / "Tests" / "m6_vision_uart_test.c"),
            str(ROOT / "System" / "m6_vision_uart.c"),
            "-o", str(executable),
        ]
        compiled = subprocess.run(command, capture_output=True, text=True)
        assert compiled.returncode == 0, compiled.stderr
        result = subprocess.run([str(executable)], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
