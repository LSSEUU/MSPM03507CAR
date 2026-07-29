from pathlib import Path
import shutil
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "python/src/m6link/web"


def test_stepper_panel_accessibility_and_safety_copy() -> None:
    html = (WEB / "dashboard.html").read_text(encoding="utf-8")
    assert 'id="stepperTest"' in html
    assert 'id="stepperStopButton"' in html
    assert "车辆急停和车辆 Safety 故障不会停止此步进电机" in html
    assert "车辆立即急停（不含步进）" in html
    assert 'aria-live="polite"' in html
    for identifier in (
        "stepperVelocitySpeed", "stepperVelocityDuration",
        "stepperPositionPulses", "stepperPositionTimeout",
        "stepperSpeedChart", "stepperPositionChart",
    ):
        assert f'id="{identifier}"' in html


def test_stepper_web_limits_routes_and_responsive_rules() -> None:
    html = (WEB / "dashboard.html").read_text(encoding="utf-8")
    css = (WEB / "dashboard.css").read_text(encoding="utf-8")
    javascript = (WEB / "dashboard.js").read_text(encoding="utf-8")
    assert 'min="1" max="300"' in html
    assert 'min="100" max="5000"' in html
    assert 'min="1" max="3200"' in html
    assert 'min="500" max="10000"' in html
    assert "@media (max-width: 500px)" in css
    assert ".stepper-grid { grid-template-columns: 1fr; }" in css
    assert "min-height: 44px" in css
    for route in ("enable", "stop", "disable", "velocity", "position"):
        assert f'"/api/stepper/{route}"' in javascript
    assert "setInterval(pollStepper, 250)" in javascript
    assert "[1, 2].includes(numeric(stepperView.snapshot?.state))" in javascript


def test_dashboard_javascript_syntax() -> None:
    node = shutil.which("node")
    if node is None:
        return
    result = subprocess.run([node, "--check", str(WEB / "dashboard.js")], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def load_tests(_loader, _tests, _pattern):
    suite = unittest.TestSuite()
    for function in (
        test_stepper_panel_accessibility_and_safety_copy,
        test_stepper_web_limits_routes_and_responsive_rules,
        test_dashboard_javascript_syntax,
    ):
        suite.addTest(unittest.FunctionTestCase(function))
    return suite
