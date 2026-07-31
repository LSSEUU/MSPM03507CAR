"""Hybrid steel-ball recognition and calibrated UART output for MaixCAM Pro."""

import math

from maix import app, camera, display, image, nn, sys, time, uart

from ball_position import (
    AdaptiveAlphaBetaFilter,
    CalibrationTable,
    MULTIPOINT_CALIBRATION_MM,
    THREE_POINT_CALIBRATION_MM,
    axis_point,
    measurement_is_valid,
    project_onto_axis,
)
from ball_wire import encode_frame
from hybrid_tracker import HybridBallTracker


# Keep the detector-to-UART path low latency.  Streaming and extra drawing are
# available for diagnosis but remain disabled in the competition defaults.
DEBUG_LOG = False
DRAW_DEBUG = False
LOW_LATENCY_MODE = True
USE_RTSP = False
USE_JPEG = False
USE_WEBRTC = False

LENS_CORR_ENABLE = False
LENS_CORR_STRENGTH = 0.6

# "three" is the current mounted-camera calibration.  "multi" retains the
# legacy 25-point table and is valid only if that earlier camera pose is used.
CALIBRATION_MODE = "three"

# HybridBallTracker owns appearance and per-source confidence decisions.  The
# application keeps the calibrated geometry, size, and physical-range gates.
MAX_AXIS_ERROR_PX = 20.0
MIN_BOX_SIZE_PX = 6.0
MAX_BOX_SIZE_PX = 64.0
MIN_BOX_ASPECT_RATIO = 0.50
MAX_BOX_ASPECT_RATIO = 2.0
MIN_POSITION_MM = -115.0
MAX_POSITION_MM = 115.0
MAX_VELOCITY_MM_S = 4000.0

MAIXCAM_MODEL_PATH = (
    "models/yolo26_all_maixcam_yolo26_480_160/yolo26_all.mud"
)


def print_debug(message):
    if DEBUG_LOG:
        print(message)


def model_path_for_device(device_name):
    """Return the packaged model for the current MaixCAM/Pro hardware."""
    normalized_name = device_name.strip().lower()
    if normalized_name in ("maixcam", "maixcam-pro", "maixcam_pro"):
        return MAIXCAM_MODEL_PATH
    raise ValueError("unsupported device: {}".format(device_name))


def round_half_away_from_zero(value):
    """Round a physical measurement without Python's ties-to-even behavior."""
    return int(value + 0.5) if value >= 0.0 else int(value - 0.5)


if CALIBRATION_MODE == "multi":
    calibration_points = MULTIPOINT_CALIBRATION_MM
elif CALIBRATION_MODE == "three":
    calibration_points = THREE_POINT_CALIBRATION_MM
else:
    raise ValueError("CALIBRATION_MODE must be 'multi' or 'three'")

calibration = CalibrationTable(calibration_points)
zero_pixels = [
    pixel for pixel, position_mm in calibration_points if position_mm == 0.0
]
if len(zero_pixels) != 1:
    raise ValueError("calibration must contain exactly one O=0 point")
zero_pixel_x = zero_pixels[0]

model = model_path_for_device(sys.device_name())
detector = nn.YOLO26(model=model, dual_buff=not LOW_LATENCY_MODE)

frame_width = detector.input_width()
frame_height = detector.input_height()
axis_y = frame_height // 2
AXIS_START_PX = (5, axis_y)
AXIS_END_PX = (frame_width - 5, axis_y)
if calibration_points[0][0] < 0 or calibration_points[-1][0] >= frame_width:
    raise ValueError("calibration pixels are outside the detector image")


def evaluate_candidate(candidate):
    """Apply the mounted-camera geometry policy to one tracker candidate."""
    try:
        measured_mm = calibration.to_mm(candidate["cx"])
        valid, axis_error_px = measurement_is_valid(
            candidate["score"],
            candidate["w"],
            candidate["h"],
            (candidate["cx"], candidate["cy"]),
            AXIS_START_PX,
            AXIS_END_PX,
            measured_mm,
            # Scores from Y/E and T/G have different meanings.  The tracker
            # has already applied the appropriate confidence policy.
            min_confidence=0.0,
            max_axis_error_px=MAX_AXIS_ERROR_PX,
            min_box_size_px=MIN_BOX_SIZE_PX,
            max_box_size_px=MAX_BOX_SIZE_PX,
            min_aspect_ratio=MIN_BOX_ASPECT_RATIO,
            max_aspect_ratio=MAX_BOX_ASPECT_RATIO,
            min_position_mm=MIN_POSITION_MM,
            max_position_mm=MAX_POSITION_MM,
        )
    except (ArithmeticError, KeyError, TypeError, ValueError):
        return False, 0.0, float("inf")
    return valid, measured_mm, axis_error_px


def candidate_is_valid(candidate):
    return evaluate_candidate(candidate)[0]


cam = camera.Camera(frame_width, frame_height, detector.input_format())
disp = display.Display()
serial = uart.UART("/dev/ttyS0", 115200)
tracker = HybridBallTracker(
    detector,
    frame_width,
    frame_height,
    candidate_validator=candidate_is_valid,
)
position_filter = AdaptiveAlphaBetaFilter()

if USE_RTSP:
    from maix import rtsp

    rtsp_camera = cam.add_channel(320, 180, image.Format.FMT_YVU420SP)
    rtsp_server = rtsp.Rtsp()
    rtsp_server.bind_camera(rtsp_camera)
    rtsp_server.start()
    print(rtsp_server.get_url())

if USE_JPEG:
    from maix import http

    jpeg_server = http.JpegStreamer()
    jpeg_server.start()

if USE_WEBRTC:
    from maix import webrtc

    webrtc_camera = cam.add_channel(320, 180, image.Format.FMT_YVU420SP)
    webrtc_server = webrtc.WebRTC()
    webrtc_server.bind_camera(webrtc_camera)
    webrtc_server.start()
    print(webrtc_server.get_url())

print(
    "Hybrid detector v1.3: {}x{}, model={}".format(
        frame_width, frame_height, model
    )
)

axis_color = image.Color.from_rgb(0, 220, 255)
zero_color = image.Color.from_rgb(255, 220, 0)
panel_color = image.Color.from_rgb(0, 0, 0)
source_colors = {
    "Y": image.COLOR_GREEN,
    "T": image.Color.from_rgb(255, 220, 0),
    "E": image.Color.from_rgb(0, 220, 255),
    "G": image.Color.from_rgb(255, 140, 0),
}


def draw_calibration_axis(img):
    img.draw_line(
        AXIS_START_PX[0],
        AXIS_START_PX[1],
        AXIS_END_PX[0],
        AXIS_END_PX[1],
        axis_color,
        2,
    )
    img.draw_cross(AXIS_START_PX[0], AXIS_START_PX[1], axis_color, 7, 2)
    img.draw_cross(AXIS_END_PX[0], AXIS_END_PX[1], axis_color, 7, 2)
    img.draw_cross(int(zero_pixel_x), axis_y, zero_color, 9, 2)


sequence = 0
last_ms = time.ticks_ms()
loop_ms = 1000

while not app.need_exit():
    loop_ms = time.ticks_ms() - last_ms
    last_ms = time.ticks_ms()

    img = cam.read()
    if LENS_CORR_ENABLE:
        img = img.lens_corr(strength=LENS_CORR_STRENGTH)

    detect_started_ms = time.ticks_ms()
    now_ms = time.ticks_ms()
    ball, tracker_debug = tracker.process(img, now_ms)
    detect_cost_ms = time.ticks_ms() - detect_started_ms

    valid = False
    position_mm = 0.0
    velocity_mm_s = 0.0
    measured_mm = 0.0
    axis_error_px = 0.0

    if ball is not None:
        valid, measured_mm, axis_error_px = evaluate_candidate(ball)
        if valid:
            position_mm, velocity_mm_s = position_filter.update(
                measured_mm, now_ms
            )
            x_mm_x10 = round_half_away_from_zero(position_mm * 10.0)
            vx_mm_s = round_half_away_from_zero(velocity_mm_s)
            valid = (
                math.isfinite(position_mm)
                and math.isfinite(velocity_mm_s)
                and MIN_POSITION_MM <= position_mm <= MAX_POSITION_MM
                and abs(velocity_mm_s) <= MAX_VELOCITY_MM_S
                and -32768 <= x_mm_x10 <= 32767
                and -32768 <= vx_mm_s <= 32767
            )
            if not valid:
                position_filter.reset()
        else:
            position_filter.mark_missing(now_ms)
    else:
        position_filter.mark_missing(now_ms)

    if valid:
        tx_frame = encode_frame(sequence, True, x_mm_x10, vx_mm_s)
    else:
        tx_frame = encode_frame(sequence, False)
    serial.write(tx_frame)
    sequence = (sequence + 1) & 0xFF

    draw_calibration_axis(img)
    img.draw_rect(0, 0, frame_width, 35, panel_color, -1)

    if ball is not None:
        source = ball["source"]
        box_color = source_colors.get(source, image.COLOR_GREEN)
        if not valid:
            box_color = image.COLOR_RED
        draw_x = max(0, int(ball["x"]))
        draw_y = max(0, int(ball["y"]))
        draw_w = max(1, min(frame_width - draw_x, int(ball["w"])))
        draw_h = max(1, min(frame_height - draw_y, int(ball["h"])))
        img.draw_rect(draw_x, draw_y, draw_w, draw_h, box_color, 2)

        if DRAW_DEBUG:
            ratio, _ = project_onto_axis(
                (ball["cx"], ball["cy"]), AXIS_START_PX, AXIS_END_PX
            )
            projected_x, projected_y = axis_point(
                ratio, AXIS_START_PX, AXIS_END_PX
            )
            img.draw_cross(int(ball["cx"]), int(ball["cy"]), box_color, 12, 2)
            img.draw_cross(
                int(projected_x), int(projected_y), zero_color, 7, 2
            )
            label = "{} Q{:.2f} A{:.1f}px {}ms".format(
                source, ball["score"], axis_error_px, detect_cost_ms
            )
            img.draw_string(draw_x, draw_y, label, color=box_color)

    if valid:
        status_text = "BALL {:+.1f} mm {} V {:+.0f}".format(
            position_mm, ball["source"], velocity_mm_s
        )
        status_color = image.COLOR_WHITE
    elif ball is not None:
        status_text = "BALL INVALID"
        status_color = image.COLOR_RED
    else:
        status_text = "BALL LOST"
        status_color = image.COLOR_RED
    img.draw_string(
        8, 5, status_text, color=status_color, scale=1.4, thickness=2
    )

    fps = 0 if loop_ms <= 0 else 1000 // loop_ms
    fps_str = "FPS:{}".format(fps)
    fps_width = image.string_size(fps_str, scale=1.4, thickness=2).width()
    img.draw_string(
        img.width() - fps_width,
        5,
        fps_str,
        color=image.COLOR_GREEN,
        scale=1.4,
        thickness=2,
    )

    print_debug(
        "source={} detect={}ms loop={}ms raw={} template={:.2f}".format(
            tracker_debug["source"],
            detect_cost_ms,
            loop_ms,
            tracker_debug["raw_objects"],
            tracker_debug["template_score"],
        )
    )

    if USE_JPEG:
        jpeg_server.write(img)
    disp.show(img)
