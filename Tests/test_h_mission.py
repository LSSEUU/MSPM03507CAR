import hashlib
import json
import pathlib
import shutil
import struct
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
EXPECTED = "973cb83992eaa81bd4a021391e8c98498e78ddf261c7c13949c163338bab9b67"


class HMissionTests(unittest.TestCase):
    def test_ram_state_machine(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler)
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "h_mission_test.exe"
            command = [
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "App"),
                str(ROOT / "Tests" / "h_mission_test.c"),
                str(ROOT / "App" / "h_mission.c"),
                "-o", str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_extension_registry_is_frozen_and_bound(self):
        path = ROOT / "Tests" / "vectors" / "m6_h_mission_ext_v1.json"
        registry = json.loads(path.read_text(encoding="utf-8"))
        digest = hashlib.sha256(json.dumps(
            registry, sort_keys=True, separators=(",", ":"), ensure_ascii=True
        ).encode("ascii")).hexdigest()
        self.assertEqual(digest, EXPECTED)
        self.assertEqual(registry["capability"]["bit"], 29)
        self.assertEqual(registry["rpc_opcodes"]["H_MISSION_STATUS_GET"], 15)
        self.assertEqual(registry["rpc_opcodes"]["H_MISSION_COMMAND"], 0x8309)
        header = (ROOT / "System" / "m6_h_mission_ext.h").read_text(encoding="utf-8")
        binding = (ROOT / "python" / "src" / "m6link" / "h_mission_registry.py").read_text(encoding="utf-8")
        self.assertIn(EXPECTED, header)
        self.assertIn(EXPECTED, binding)

    def test_no_flash_save_surface(self):
        controller = (ROOT / "python" / "src" / "m6link" / "dashboard_control.py").read_text(encoding="utf-8")
        dashboard = (ROOT / "python" / "src" / "m6link" / "dashboard.py").read_text(encoding="utf-8")
        self.assertIn('"/api/h-mission/stage"', dashboard)
        self.assertIn('"/api/h-mission/apply"', dashboard)
        self.assertIn('"/api/h-mission/revert"', dashboard)
        self.assertNotIn('"/api/h-mission/save"', dashboard)
        self.assertIn('"save_supported": False', controller)

    def test_host_snapshot_layout_and_signed_targets(self):
        from m6link.session import Session

        body = bytearray(64)
        struct.pack_into("<III4B", body, 0, 1234, 7, 9, 4, 4, 3, 2)
        struct.pack_into("<3H", body, 18, 260, 80, 40)
        struct.pack_into("<5H", body, 24, 220, 30, 250, 210, 0)
        struct.pack_into("<5H", body, 34, 200, 20, 230, 190, 500)
        struct.pack_into("<IIIIhh", body, 44, 15305, 1000, 8400, 7400, -1, 220)
        snapshot = Session._decode_h_mission(bytes(body))
        self.assertEqual(snapshot.active.h4_straight_speed_mm_s, 220)
        self.assertEqual(snapshot.staged.development_end_distance_mm, 500)
        self.assertEqual(snapshot.distance_mm, 1530.5)
        self.assertEqual(snapshot.final_left_target_mm_s, -1)
        self.assertEqual(struct.calcsize("<IIIBBH5H6x"), 32)


if __name__ == "__main__":
    unittest.main()
