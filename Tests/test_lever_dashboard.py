from pathlib import Path
from types import SimpleNamespace
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python" / "src"))

from m6link.dashboard_control import DashboardController  # noqa: E402
from m6link.lever_calibration_registry import LEVER_CALIBRATION  # noqa: E402
from m6link.models import LeverCalibrationSnapshot  # noqa: E402


def snapshot(*, state: int = 0, flags: int = 0) -> LeverCalibrationSnapshot:
    return LeverCalibrationSnapshot(
        time_ms=1, boot_id=2, generation=7, expected_generation=7,
        candidate_crc32=0x11223344, driver_config_fingerprint=0x55667788,
        reject_detail=0, state=state, result=0, flags=flags, page=0,
        required=54, angles_mdeg=tuple(-8000 + i * 2000 for i in range(9)),
        increasing_counts=tuple(-800 + i * 200 for i in range(9)),
        decreasing_counts=tuple(-798 + i * 200 for i in range(9)),
    )


class FakeLeverSession:
    def __init__(self):
        self.capabilities = LEVER_CALIBRATION
        self.current = snapshot()
        self.calls = []

    def get_lever_calibration_full(self):
        self.calls.append(("get-full",))
        return self.current

    def get_lever_calibration(self):
        self.calls.append(("get",))
        return self.current

    def start_lever_calibration(self, **kwargs):
        self.calls.append(("start", kwargs))
        self.current = snapshot(state=1, flags=0)
        return self.current

    def cancel_lever_calibration(self, **kwargs):
        self.calls.append(("cancel", kwargs))
        self.current = snapshot(state=8, flags=0)
        return self.current


class LeverDashboardTests(unittest.TestCase):
    def test_controller_exposes_read_only_wizard(self):
        session = FakeLeverSession()
        controller = DashboardController(
            session, SimpleNamespace(enabled_capabilities=LEVER_CALIBRATION)
        )
        try:
            state = controller.lever_calibration_state(force=True)
            self.assertTrue(state["available"])
            self.assertEqual(state["angles_mdeg"][4], 0)
            started = controller.lever_calibration_command("start", {
                "expected_generation": 7, "phone_zero_mdeg": 100,
                "force_full": True,
            })
            self.assertEqual(started["state_name"], "WAIT_REFERENCE")
            self.assertEqual(started["evidence"][0]["kind"], "phone_zero_check")
        finally:
            controller.close()

    def test_assets_hide_raw_motion_and_offer_exports(self):
        web = ROOT / "python" / "src" / "m6link" / "web"
        html = (web / "dashboard.html").read_text(encoding="utf-8")
        js = (web / "dashboard.js").read_text(encoding="utf-8")
        dashboard = (ROOT / "python/src/m6link/dashboard.py").read_text(encoding="utf-8")
        self.assertIn("本页面不会驱动电机", html)
        self.assertIn('id="leverExportJsonButton"', html)
        self.assertIn('id="leverExportCsvButton"', html)
        self.assertIn("54 点严格检查通过", js)
        self.assertIn("downloadLeverEvidence", js)
        self.assertNotIn('"/api/stepper/enable":', dashboard)
        self.assertNotIn('"/api/stepper/velocity":', dashboard)
        self.assertNotIn('"/api/stepper/position":', dashboard)


if __name__ == "__main__":
    unittest.main()
