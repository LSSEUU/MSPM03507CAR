import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class HMissionControlProfileTests(unittest.TestCase):
    def test_h4_h5_profiles_are_offline_exact_and_restore_external_config(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "h_mission_control_profile_test.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "App"),
                "-I",
                str(ROOT / "Control"),
                "-I",
                str(ROOT / "Config"),
                str(ROOT / "Tests" / "h_mission_control_profile_test.c"),
                str(ROOT / "App" / "h_mission_control_profile.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_h4_h5_profile_is_bound_to_start_and_all_idle_exits(self):
        app_line = (ROOT / "App" / "app_line.c").read_text(encoding="utf-8")
        mission = (ROOT / "App" / "h_mission.c").read_text(encoding="utf-8")
        project = (
            ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
        ).read_text(encoding="utf-8")
        self.assertIn("HMissionControlProfile_Activate(mission_mode)", app_line)
        self.assertIn("mission_mode == H_MISSION_MODE_H4", app_line)
        self.assertIn("mission_mode == H_MISSION_MODE_H5", app_line)
        self.assertIn("HMissionControlProfile_Restore()", app_line)
        self.assertIn("HMissionControlProfile_Init()", app_line)
        self.assertIn("config.h5_straight_speed_mm_s = 230.0f;", mission)
        self.assertIn("../App/h_mission_control_profile.c", project)


if __name__ == "__main__":
    unittest.main()
