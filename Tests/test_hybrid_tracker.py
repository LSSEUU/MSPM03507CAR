from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEMO_UART = ROOT / "demo_uart"
sys.path.insert(0, str(DEMO_UART))

from hybrid_tracker import HybridBallTracker  # noqa: E402


def detection(x=40, y=70, w=20, h=20, score=0.8, class_id=0):
    return SimpleNamespace(
        x=x,
        y=y,
        w=w,
        h=h,
        score=score,
        class_id=class_id,
    )


def result(source, x=40, y=70, w=20, h=20, score=0.8):
    return {
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "cx": x + w * 0.5,
        "cy": y + h * 0.5,
        "score": score,
        "source": source,
        "class_id": 0,
    }


class ScriptedDetector:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def detect(self, frame, conf_th, iou_th):
        self.calls.append((frame, conf_th, iou_th))
        return self.responses.pop(0) if self.responses else []


def host_frame_conversion(_frame):
    row = np.arange(480, dtype=np.uint16) % 256
    gray = np.tile(row.astype(np.uint8), (160, 1))
    rgb = np.repeat(gray[:, :, None], 3, axis=2)
    return rgb, gray


class HybridTrackerSelectionTests(unittest.TestCase):
    def test_validator_filters_before_acquisition_and_temporal_selection(self):
        tracker = HybridBallTracker(
            ScriptedDetector([]),
            480,
            160,
            candidate_validator=lambda item: item["cx"] < 100.0,
        )

        selected = tracker._select_yolo(
            [
                detection(x=200, score=0.99),
                detection(x=40, score=0.30),
            ],
            0,
            "Y",
        )
        self.assertIsNotNone(selected)
        self.assertEqual(selected["cx"], 50.0)
        self.assertEqual(selected["source"], "Y")

        self.assertIsNone(
            tracker._select_yolo([detection(score=0.27)], 0, "Y")
        )

        tracker._commit(selected, 0)
        continued = tracker._select_yolo(
            [detection(x=44, score=0.18)], 40, "Y"
        )
        self.assertIsNotNone(continued)
        self.assertAlmostEqual(continued["score"], 0.18)

    def test_template_result_is_rejected_by_application_validator(self):
        tracker = HybridBallTracker(
            ScriptedDetector([]),
            480,
            160,
            candidate_validator=lambda _item: False,
        )
        _, gray = host_frame_conversion(None)
        tracker.last_result = result("Y")
        tracker.last_seen_ms = 0
        tracker.template = gray[68:92, 38:62].copy()
        tracker.template_ball_offset = (2, 2, 20, 20)

        matched, score = tracker._track_template(gray, 40)
        self.assertIsNone(matched)
        self.assertGreater(score, tracker.template_threshold)


class HybridTrackerRoutingTests(unittest.TestCase):
    @staticmethod
    def make_tracker(raw_responses):
        tracker = HybridBallTracker(
            ScriptedDetector(raw_responses),
            480,
            160,
            candidate_validator=lambda _item: True,
        )
        tracker._frame_to_rgb_and_gray = host_frame_conversion
        return tracker

    def test_yolo_normal_path_returns_y_and_refreshes_template(self):
        tracker = self.make_tracker([[detection()]])
        ball, debug = tracker.process(object(), 10)

        self.assertEqual(ball["source"], "Y")
        self.assertEqual(debug["source"], "Y")
        self.assertEqual(debug["raw_objects"], 1)
        self.assertIs(tracker.last_result, ball)
        self.assertIsNotNone(tracker.template)
        self.assertEqual(tracker.detector.calls[0][1], 0.18)

    def test_local_template_recovery_returns_t(self):
        tracker = self.make_tracker([[]])
        tracker.last_result = result("Y")
        tracker.last_seen_ms = 0
        tracker.template = np.ones((12, 12), dtype=np.uint8)
        tracker.template_ball_offset = (0, 0, 20, 20)
        recovered = result("T", x=43, score=0.72)
        tracker._track_template = lambda gray, now_ms, full_frame=False: (
            (recovered, 0.65) if not full_frame else (None, 0.0)
        )

        ball, debug = tracker.process(object(), 40)
        self.assertIs(ball, recovered)
        self.assertEqual(debug["source"], "T")
        self.assertAlmostEqual(debug["template_score"], 0.65)

    def test_template_failure_uses_brightness_enhanced_yolo(self):
        tracker = self.make_tracker([[]])
        tracker._track_template = lambda gray, now_ms, full_frame=False: (
            None,
            0.20,
        )
        enhanced = result("E", x=45, score=0.35)
        tracker._detect_enhanced = lambda rgb, gray, now_ms: (
            enhanced,
            2,
            70.0,
        )

        ball, debug = tracker.process(object(), 40)
        self.assertIs(ball, enhanced)
        self.assertEqual(debug["source"], "E")
        self.assertEqual(debug["enhanced_objects"], 2)
        self.assertEqual(debug["brightness"], 70.0)

    def test_periodic_full_frame_relocation_returns_g(self):
        tracker = self.make_tracker([[]])
        tracker.last_result = result("Y")
        tracker.last_seen_ms = 0
        tracker.template = np.ones((12, 12), dtype=np.uint8)
        tracker.template_ball_offset = (0, 0, 20, 20)
        tracker.missing_frames = 2
        relocated = result("G", x=80, score=0.75)

        def track(_gray, _now_ms, full_frame=False):
            return (relocated, 0.70) if full_frame else (None, 0.25)

        tracker._track_template = track
        tracker._detect_enhanced = lambda rgb, gray, now_ms: (
            None,
            0,
            120.0,
        )

        ball, debug = tracker.process(object(), 120)
        self.assertIs(ball, relocated)
        self.assertEqual(debug["source"], "G")
        self.assertAlmostEqual(debug["global_template_score"], 0.70)

    def test_tracking_state_resets_after_900_ms_without_result(self):
        tracker = self.make_tracker([[]])
        tracker.last_result = result("Y")
        tracker.last_seen_ms = 0
        tracker.template = np.ones((12, 12), dtype=np.uint8)
        tracker.template_ball_offset = (0, 0, 20, 20)
        tracker._track_template = lambda gray, now_ms, full_frame=False: (
            None,
            0.0,
        )
        tracker._detect_enhanced = lambda rgb, gray, now_ms: (
            None,
            0,
            120.0,
        )

        ball, debug = tracker.process(object(), 901)
        self.assertIsNone(ball)
        self.assertEqual(debug["source"], "-")
        self.assertIsNone(tracker.last_result)
        self.assertIsNone(tracker.template)
        self.assertEqual(tracker.velocity_x, 0.0)


if __name__ == "__main__":
    unittest.main()
