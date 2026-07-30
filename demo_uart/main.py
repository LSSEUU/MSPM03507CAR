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


# Defaults for the H3 sprint sender. Streaming and debug output stay off so the
# detector-to-UART path has the smallest practical latency.
DEBUG_LOG = False
LOW_LATENCY_MODE = True
USE_RTSP = False
USE_JPEG = False
USE_WEBRTC = False

LENS_CORR_ENABLE = False
LENS_CORR_STRENGTH = 0.6

# "multi" reuses the current 25-point installation table. Change to "three"
# to use only the measured -50, O, and +50 mm points.
CALIBRATION_MODE = "multi"

DETECTION_CONFIDENCE = 0.50
MAX_AXIS_ERROR_PX = 20.0
MIN_BOX_SIZE_PX = 6.0
MAX_BOX_SIZE_PX = 64.0
MIN_BOX_ASPECT_RATIO = 0.50
MAX_BOX_ASPECT_RATIO = 2.0
MIN_POSITION_MM = -115.0
MAX_POSITION_MM = 115.0

MAIXCAM_MODEL_PATH = (
    "models/yolo26_all_maixcam_yolo26_480_160/yolo26_all.mud"
)
MAIXCAM2_MODEL_PATH = (
    "models/yolo26_all_maixcam2_yolo26_640_160/yolo26_all.mud"
)


def print_debug(message):
    if DEBUG_LOG:
        print(message)


def model_path_for_device(device_name):
    """Return the checked-in YOLO26 model matching a Maix device name."""
    normalized_name = device_name.strip().lower()
    if normalized_name == "maixcam2":
        return MAIXCAM2_MODEL_PATH
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
zero_pixels = [pixel for pixel, position_mm in calibration_points if position_mm == 0.0]
if len(zero_pixels) != 1:
    raise ValueError("calibration must contain exactly one O=0 point")
zero_pixel_x = zero_pixels[0]

model = model_path_for_device(sys.device_name())
detector = nn.YOLO26(model=model, dual_buff=not LOW_LATENCY_MODE)

axis_y = detector.input_height() // 2
AXIS_START_PX = (5, axis_y)
AXIS_END_PX = (detector.input_width() - 5, axis_y)
if calibration_points[0][0] < 0 or calibration_points[-1][0] >= detector.input_width():
    raise ValueError("calibration pixels are outside the detector image")

cam = camera.Camera(
    detector.input_width(), detector.input_height(), detector.input_format()
)
disp = display.Display()
serial = uart.UART("/dev/ttyS0", 115200)
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

print_debug("detector {}x{}".format(detector.input_width(), detector.input_height()))

axis_color = image.Color.from_rgb(0, 220, 255)
zero_color = image.Color.from_rgb(255, 220, 0)
panel_color = image.Color.from_rgb(0, 0, 0)


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
    print_debug("loop cost {}ms".format(loop_ms))
    last_ms = time.ticks_ms()

    img = cam.read()
    started_ms = time.ticks_ms()
    if LENS_CORR_ENABLE:
        img = img.lens_corr(strength=LENS_CORR_STRENGTH)
    print_debug("lens_corr cost {}ms".format(time.ticks_ms() - started_ms))

    started_ms = time.ticks_ms()
    objs = detector.detect(img, conf_th=DETECTION_CONFIDENCE, iou_th=0.45)
    print_debug("detect cost {}ms".format(time.ticks_ms() - started_ms))

    now_ms = time.ticks_ms()
    ball = max(objs, key=lambda obj: obj.score) if objs else None
    valid = False
    position_mm = 0.0
    velocity_mm_s = 0.0
    measured_mm = 0.0
    axis_error_px = 0.0
    raw_x = 0.0
    raw_y = 0.0

    if ball is not None:
        raw_x = ball.x + ball.w * 0.5
        raw_y = ball.y + ball.h * 0.5
        try:
            measured_mm = calibration.to_mm(raw_x)
        except (TypeError, ValueError):
            measured_mm = float("nan")
        valid, axis_error_px = measurement_is_valid(
            ball.score,
            ball.w,
            ball.h,
            (raw_x, raw_y),
            AXIS_START_PX,
            AXIS_END_PX,
            measured_mm,
            min_confidence=DETECTION_CONFIDENCE,
            max_axis_error_px=MAX_AXIS_ERROR_PX,
            min_box_size_px=MIN_BOX_SIZE_PX,
            max_box_size_px=MAX_BOX_SIZE_PX,
            min_aspect_ratio=MIN_BOX_ASPECT_RATIO,
            max_aspect_ratio=MAX_BOX_ASPECT_RATIO,
            min_position_mm=MIN_POSITION_MM,
            max_position_mm=MAX_POSITION_MM,
        )

        if valid:
            position_mm, velocity_mm_s = position_filter.update(measured_mm, now_ms)
            x_mm_x10 = round_half_away_from_zero(position_mm * 10.0)
            vx_mm_s = round_half_away_from_zero(velocity_mm_s)
            valid = (
                math.isfinite(position_mm)
                and math.isfinite(velocity_mm_s)
                and MIN_POSITION_MM <= position_mm <= MAX_POSITION_MM
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

    draw_started_ms = time.ticks_ms()
    draw_calibration_axis(img)
    img.draw_rect(0, 0, detector.input_width(), 35, panel_color, -1)

    if ball is not None:
        box_color = image.COLOR_GREEN if valid else image.COLOR_RED
        img.draw_rect(ball.x, ball.y, ball.w, ball.h, color=box_color, thickness=2)

        if DEBUG_LOG:
            ratio, _ = project_onto_axis(
                (raw_x, raw_y), AXIS_START_PX, AXIS_END_PX
            )
            projected_x, projected_y = axis_point(
                ratio, AXIS_START_PX, AXIS_END_PX
            )
            img.draw_cross(int(raw_x), int(raw_y), box_color, 12, 2)
            img.draw_cross(int(projected_x), int(projected_y), zero_color, 7, 2)
            label = (
                "{} {:.2f} raw={:+.1f}mm axis={:.1f}px".format(
                    detector.labels[ball.class_id],
                    ball.score,
                    measured_mm,
                    axis_error_px,
                )
            )
            img.draw_string(ball.x, ball.y, label, color=image.COLOR_RED)

    if valid:
        status_text = "BALL {:+.1f} mm  V {:+.0f}".format(
            position_mm, velocity_mm_s
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
    print_debug("draw cost {}ms".format(time.ticks_ms() - draw_started_ms))

    if USE_JPEG:
        jpeg_server.write(img)
    disp.show(img)
