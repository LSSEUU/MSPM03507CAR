import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class M5LineFollowTests(unittest.TestCase):
    def _compile_and_run(self, source, defines=()):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / f"{source.stem}.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                *defines,
                "-I",
                str(ROOT / "Control"),
                "-I",
                str(ROOT / "Config"),
                str(source),
                str(ROOT / "Control" / "line_follow.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_line_follow_model(self):
        values_white = "{3000U,3000U,3000U,3000U,3000U,3000U,3000U,3000U}"
        values_black = "{800U,800U,800U,800U,800U,800U,800U,800U}"
        self._compile_and_run(
            ROOT / "Tests" / "m5_line_follow_test.c",
            (
                "-DGRAYSCALE_CALIBRATION_VERIFIED=1U",
                f"-DGRAYSCALE_WHITE_ADC_VALUES={values_white}",
                f"-DGRAYSCALE_BLACK_ADC_VALUES={values_black}",
            ),
        )

    def test_digital_line_follow_root_fix(self):
        values_white = "{3000U,3000U,3000U,3000U,3000U,3000U,3000U,3000U}"
        values_black = "{800U,800U,800U,800U,800U,800U,800U,800U}"
        self._compile_and_run(
            ROOT / "Tests" / "m5_line_follow_digital_test.c",
            (
                "-DGRAYSCALE_CALIBRATION_VERIFIED=1U",
                f"-DGRAYSCALE_WHITE_ADC_VALUES={values_white}",
                f"-DGRAYSCALE_BLACK_ADC_VALUES={values_black}",
            ),
        )

    def test_line_yaw_rate_assist(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "line_yaw_assist_test.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "Control"),
                "-I",
                str(ROOT / "Config"),
                str(ROOT / "Tests" / "line_yaw_assist_test.c"),
                str(ROOT / "Control" / "line_yaw_assist.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_fixed_h_course_profile(self):
        compiler = shutil.which("gcc") or shutil.which("clang")
        self.assertIsNotNone(compiler, "A host C compiler is required")
        with tempfile.TemporaryDirectory() as temporary:
            executable = pathlib.Path(temporary) / "h_course_profile_test.exe"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "Control"),
                "-I",
                str(ROOT / "Config"),
                str(ROOT / "Tests" / "h_course_profile_test.c"),
                str(ROOT / "Control" / "h_course_profile.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            compiled = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_unverified_compile_time_calibration_locks_motion(self):
        self._compile_and_run(
            ROOT / "Tests" / "m5_line_lock_test.c",
            ("-DGRAYSCALE_CALIBRATION_VERIFIED=0U",),
        )

    def test_verified_production_calibration_is_valid(self):
        self._compile_and_run(
            ROOT / "Tests" / "m5_line_production_calibration_test.c"
        )

    def test_runtime_routes_line_app_through_control_and_safety(self):
        main = (ROOT / "Source" / "main.c").read_text(encoding="utf-8")
        app = (ROOT / "App" / "app_line.c").read_text(encoding="utf-8")
        safety = (ROOT / "System" / "safety.h").read_text(encoding="utf-8")
        self.assertIn("AppLine_ProcessGrayscale(&gray)", main)
        self.assertIn("AppLine_Task20ms(started)", main)
        self.assertIn("Grayscale_Task(SystemTime_Now())", main)
        self.assertIn("WheelSpeed_SetTarget", app)
        self.assertIn("AppLine_StartTimed", app)
        self.assertIn("AppLine_StartH2", app)
        self.assertIn("H2ControlProfile_Activate", app)
        self.assertIn("H2ControlProfile_Restore", app)
        self.assertIn("LineFollow_ProcessDigitalMask(gray->conditioned_mask", app)
        self.assertIn("LineYawAssist_Update", app)
        self.assertIn("HCourse_Update", app)
        self.assertIn("HCourse_SelectBaseSpeed", app)
        self.assertIn("HCourse_SetFinishEnabled", app)
        self.assertNotIn("HCourse_SetFixedCourseStart", app)
        self.assertIn("#define APP_H4_MISSION_STOP_TIMEOUT_MS   1500U", app)
        self.assertIn("#define APP_H4_HEADING_HOLD_DISTANCE_MM 1400.0f", app)
        self.assertIn("#define APP_H_MISSION_STOP_DISTANCE_MM    250.0f", app)
        self.assertIn("#define APP_H4_STOP_CORRECTION_RATIO         0.20f", app)
        self.assertIn("mission_stop_heading_valid", app)
        self.assertIn("LINE_YAW_ASSIST_MODE_STRAIGHT", app)
        self.assertIn("mission_h4_heading_hold_active = true;", app)
        self.assertIn("course.total_distance_mm >= APP_H4_HEADING_HOLD_DISTANCE_MM", app)
        self.assertIn("mission_path_guard_active = guarded_mission &&", app)
        self.assertIn("mission_white_ms = mission_path_guard_active &&", app)
        self.assertIn("mission_outer_ms = mission_path_guard_active &&", app)
        self.assertIn("float geometric_curve_correction =", app)
        self.assertIn("assisted_control.dynamic_base_mm_s * CHASSIS_DRIVE_TRACK_MM", app)
        self.assertIn("(2.0f * LINE_CURVE_RADIUS_MM)", app)
        self.assertIn("#define APP_H5_CURVE_CORRECTION_MARGIN        1.15f", app)
        self.assertIn("assisted_control.correction_mm_s = 0.0f;", app)
        self.assertIn("H_MISSION_STOP_DISTANCE", app)
        self.assertIn("H2_FINISH_STOP_OFFSET_MM", app)
        self.assertIn("AppLine_CompleteFinish", app)
        self.assertIn("LineYawAssist_SetMode", app)
        self.assertIn("heading.rate_source == HEADING_RATE_GYRO_FRAME", app)
        self.assertIn("WheelSpeed_SetTarget(app_line.final_left_target_mm_s", app)
        self.assertIn("snapshot->control.correction_mm_s =", app)
        self.assertIn("snapshot->control.dynamic_base_mm_s =", app)
        self.assertIn("snapshot->final_left_target_mm_s", app)
        self.assertIn("snapshot->final_right_target_mm_s", app)
        self.assertIn("#define APP_LINE_TIMED_SAFETY_GRACE_MS 250U", app)
        self.assertIn("duration_ms + APP_LINE_TIMED_SAFETY_GRACE_MS", app)
        self.assertIn("SAFETY_SENSOR_BIT(SAFETY_SENSOR_GRAYSCALE)", app)
        self.assertIn("SAFETY_SENSOR_BIT(SAFETY_SENSOR_GYRO)", app)
        self.assertIn("SAFETY_FAULT_LINE_LOST", app)
        self.assertIn("SAFETY_FAULT_LINE_LOST", safety)
        self.assertNotIn("Motor_SetPWM", app)

    def test_project_and_diagnostics_expose_guarded_line_application(self):
        project = (
            ROOT / "keil" / "empty_LP_MSPM0G3507_nortos_keil.uvprojx"
        ).read_text(encoding="utf-8")
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("../Control/line_follow.c", project)
        self.assertIn("../Control/line_yaw_assist.c", project)
        self.assertIn("../Control/h_course_profile.c", project)
        self.assertIn("../App/app_line.c", project)
        self.assertIn("r=line 1=calibration 3=arm line A=start 0=stop", diagnostics)
        self.assertIn("complete module calibration and compatibility qualification first", diagnostics)
        self.assertIn("OUT.8IR.B mask uses 1=line", diagnostics)
        self.assertNotIn("AppLine_StartCalibration", diagnostics)
        self.assertIn("AppLine_Arm", diagnostics)
        self.assertIn("AppLine_ConfirmStart", diagnostics)
        self.assertIn("Diagnostics_QueueLineCalibrationReport", diagnostics)
        self.assertIn("Diagnostics_ServiceLineCalibrationReport", diagnostics)
        self.assertNotIn("Diagnostics_PrintLineCalibration", diagnostics)
        self.assertIn("Diagnostics_QueueGrayReport", diagnostics)
        self.assertIn("Diagnostics_ServiceGrayReport", diagnostics)
        self.assertNotIn("Diagnostics_PrintGraySnapshot", diagnostics)
        self.assertIn("CSV: M5,time_ms", diagnostics)

    def test_runtime_status_queries_are_bounded_during_line_action(self):
        diagnostics = (ROOT / "System" / "diagnostics.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("Diagnostics_QueueSafetyReport", diagnostics)
        self.assertIn("Diagnostics_ServiceSafetyReport", diagnostics)
        self.assertIn("Diagnostics_ActionTelemetryActive", diagnostics)
        self.assertIn("M5 running: use CSV telemetry; stop before r.", diagnostics)
        self.assertNotIn(
            "case 'v': case 'V': Diagnostics_PrintSafety()", diagnostics
        )

    def test_first_landing_tuning_is_frozen(self):
        config = (ROOT / "Config" / "control_config.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("LINE_FOLLOW_BASE_SPEED_MM_S             160.0f", config)
        self.assertIn("LINE_FOLLOW_KP_MM_S                     150.0f", config)
        self.assertIn("LINE_FOLLOW_KD_MM_S_PER_UNIT_S            1.0f", config)
        self.assertIn("LINE_FOLLOW_MAX_CORRECTION_MM_S         150.0f", config)


if __name__ == "__main__":
    unittest.main()
