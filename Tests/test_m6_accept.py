from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "m6_accept.py"
SPEC = importlib.util.spec_from_file_location("m6_accept", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class M6AcceptanceWindowTests(unittest.TestCase):
    def test_ground_steps_use_frozen_arc_ratio_and_fresh_specs(self):
        steps = MODULE.build_steps(160, 0.7, 1.5)
        self.assertEqual(
            [(step.left_mm_s, step.right_mm_s, step.duration_ms) for step in steps],
            [(160, 160, 1500), (112, 160, 1500), (160, 112, 1500)],
        )

    def test_script_never_auto_confirms_or_imports_pyserial(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("import serial", source)
        self.assertEqual(source.count(".confirm_action("), 1)
        confirm_method = source.index("    def confirm_current(")
        confirm_call = source.index(".confirm_action(")
        self.assertLess(confirm_method, confirm_call)
        self.assertIn('command=self.confirm_current', source)

    def test_bounds_reject_unsafe_or_invalid_steps(self):
        for arguments in ((49, 0.7, 1.5), (401, 0.7, 1.5), (160, 0.1, 1.5), (160, 0.7, 0.05)):
            with self.assertRaises(ValueError):
                MODULE.build_steps(*arguments)


if __name__ == "__main__":
    unittest.main()
