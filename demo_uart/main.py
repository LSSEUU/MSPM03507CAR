from maix import camera, display, image, nn, app, time, sys, uart

from ball_position import (
    AdaptiveAlphaBetaFilter,
    CalibrationTable,
    axis_point,
    position_from_pixel,
    validate_calibration,
)

# 输入打印. 关掉后提升帧率
DEBUG_LOG=False

def print_debug(s):
    if DEBUG_LOG:
        print(s)

# 位置传感器更重视“采集到结果”的延迟，而不是最高吞吐帧率。
# 低延迟模式下 dual_buff=False，每次检测结果都对应本次传入的图像。
# 只有在测试最高检测帧率时，才建议把该开关改为 False。
LOW_LATENCY_MODE = True

# RTSP流
USE_RTSP=False

# HTTP JPEG流
USE_JPEG=False

# WEBRTC流
USE_WEBRTC=False    #True

# 使能畸变校准
LENS_CORR_ENABLE=False

# 畸变校准强度
LENS_CORR_STRENGTH=0.6

# 钢珠位置两点标定。固定摄像头后，把钢珠分别放在两个已知刻度上，
# 将检测框中心像素和实际刻度填写到这里。起点为负方向，终点为正方向。
# 以下像素值仅为初始示例，使用前必须实测。
AXIS_START_PX = (10, 112)
AXIS_END_PX = (300, 112)
AXIS_START_CM = -12.5   # cm (左端，负值)
AXIS_END_CM = 12.5        # cm (右端，正值)

# 多点标定表。在水管水平时实测的 (像素, cm) 对应关系。
# 启用后优先使用查表插值，精度远超两点线性标定。
# 当摄像头与水管相对位置发生变化时，可重新实测更新。
USE_CALIBRATION_TABLE = True

CALIBRATION_POINTS = [
    ( 16, -12.0),
    ( 27, -11.0),
    ( 39, -10.0),
    ( 53,  -9.0),
    ( 71,  -8.0),
    ( 83,  -7.0),
    (108,  -6.0),
    (127,  -5.0),
    (145,  -4.0),
    (165,  -3.0),
    (190,  -2.0),
    (205,  -1.0),
    (218,   0.0),
    (248,   1.0),
    (267,   2.0),
    (294,   3.0),
    (318,   4.0),
    (331,   5.0),
    (357,   6.0),
    (383,   7.0),
    (403,   8.0),
    (422,   9.0),
    (436,  10.0),
    (454,  11.0),
    (465,  12.0),
]

# 非线性校正系数（仅两点标定模式有效）。
# 0.0 = 关闭校正。正值压缩右侧读数。推荐范围: 0.5 ~ 1.2
CORRECTION_K = 0.0

DETECTION_CONFIDENCE = 0.50

# 根据设备选择对应模型；不支持的设备直接报错，避免误加载模型。
# MAIXCAM_MODEL_PATH = "models/yolo26_ball_my_maixcam_maixcam_yolo26/yolo26_ball_my_maixcam.mud"
# MAIXCAM_MODEL_PATH = "models/yolo26_ball_my_maixcam_maixcam_yolo26_480_256/yolo26_ball_my_maixcam.mud"
MAIXCAM_MODEL_PATH = "models/yolo26_all_maixcam_yolo26_480_160/yolo26_all.mud"

MAIXCAM2_MODEL_PATH = "models/yolo26_all_maixcam2_yolo26_640_160/yolo26_all.mud"

def model_path_for_device(device_name):
    """Return the checked-in YOLO26 model matching a Maix device name."""
    normalized_name = device_name.strip().lower()
    if normalized_name == "maixcam2":
        return MAIXCAM2_MODEL_PATH
    if normalized_name in ("maixcam", "maixcam-pro", "maixcam_pro"):
        return MAIXCAM_MODEL_PATH
    raise ValueError("unsupported device: {}".format(device_name))


model = model_path_for_device(sys.device_name())

detector = nn.YOLO26(
    model=model,
    dual_buff=not LOW_LATENCY_MODE,
)

AXIS_START_PX = (5, detector.input_height() // 2)
AXIS_END_PX = (detector.input_width() - 5, detector.input_height() // 2)

cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
disp = display.Display()
serial = uart.UART("/dev/ttyS0", 115200)
position_filter = AdaptiveAlphaBetaFilter()
calib_table = CalibrationTable(CALIBRATION_POINTS) if USE_CALIBRATION_TABLE else None

if USE_RTSP:
    from maix import rtsp
    cam2 = cam.add_channel(320, 180, image.Format.FMT_YVU420SP)
    server = rtsp.Rtsp()
    server.bind_camera(cam2)
    server.start()
    print(server.get_url())

if USE_JPEG:
    from maix import http
    jpeg_server = http.JpegStreamer()
    jpeg_server.start()

if USE_WEBRTC:
    from maix import webrtc
    cam2 = cam.add_channel(320, 180, image.Format.FMT_YVU420SP)
    server = webrtc.WebRTC()
    server.bind_camera(cam2)
    server.start()
    print(server.get_url())

validate_calibration(
    AXIS_START_PX,
    AXIS_END_PX,
    AXIS_START_CM,
    AXIS_END_CM,
    detector.input_width(),
    detector.input_height(),
)

print(detector.input_width(), detector.input_height())

axis_color = image.Color.from_rgb(0, 220, 255)
zero_color = image.Color.from_rgb(255, 220, 0)
panel_color = image.Color.from_rgb(0, 0, 0)


def draw_calibration_axis(img):
    img.draw_line(
        AXIS_START_PX[0], AXIS_START_PX[1],
        AXIS_END_PX[0], AXIS_END_PX[1],
        axis_color, 2,
    )
    img.draw_cross(AXIS_START_PX[0], AXIS_START_PX[1], axis_color, 7, 2)
    img.draw_cross(AXIS_END_PX[0], AXIS_END_PX[1], axis_color, 7, 2)
    zero_ratio = (0.0 - AXIS_START_CM) / (AXIS_END_CM - AXIS_START_CM)
    zero_x, zero_y = axis_point(zero_ratio, AXIS_START_PX, AXIS_END_PX)
    img.draw_cross(int(zero_x), int(zero_y), zero_color, 9, 2)

last_ms = time.ticks_ms()
loop_ms = 1000
while not app.need_exit():
    loop_ms = time.ticks_ms() - last_ms
    print_debug("loop cost " + str(loop_ms) + "ms")
    last_ms = time.ticks_ms()

    img = cam.read()
    t = time.ticks_ms()

    if LENS_CORR_ENABLE:
        img = img.lens_corr(strength=LENS_CORR_STRENGTH)
    print_debug("lens_corr cost " + str(time.ticks_ms() - t) + "ms")

    t = time.ticks_ms()
    objs = detector.detect(img, conf_th=DETECTION_CONFIDENCE, iou_th=0.45)
    print_debug("detect cost " + str(time.ticks_ms() - t) + "ms")

    t = time.ticks_ms()
    now_ms = time.ticks_ms()
    draw_calibration_axis(img)
    img.draw_rect(0, 0, detector.input_width(), 35, panel_color, -1)

    # 模型只有 steel_ball 一个类别。如果一帧出现多个框，只选择置信度最高的框，
    # 避免多个误检框同时干扰位置滤波器。
    ball = max(objs, key=lambda obj: obj.score) if objs else None
    if ball is not None:
        raw_x = ball.x + ball.w * 0.5
        raw_y = ball.y + ball.h * 0.5
        filtered_x, filtered_y = position_filter.update(raw_x, raw_y, now_ms)

        if calib_table:
            # 多点标定表：直接用实测像素-CM查表插值
            position_cm, axis_ratio, _axis_distance = calib_table.to_cm(filtered_x)
        else:
            # 两点线性标定 + 三次校正
            position_cm, axis_ratio, _axis_distance = position_from_pixel(
                (filtered_x, filtered_y),
                AXIS_START_PX,
                AXIS_END_PX,
                AXIS_START_CM,
                AXIS_END_CM,
                correction_k=CORRECTION_K,
            )
        projected_x, projected_y = axis_point(
            axis_ratio, AXIS_START_PX, AXIS_END_PX
        )

        img.draw_rect(ball.x, ball.y, ball.w, ball.h, color=image.COLOR_GREEN, thickness=2)

        if DEBUG_LOG:
            img.draw_cross(
                int(filtered_x), int(filtered_y), image.COLOR_GREEN, 12, 2
            )
            img.draw_cross(
                int(projected_x), int(projected_y), zero_color, 7, 2
            )
            msg = (
                f"{detector.labels[ball.class_id]}: {ball.score:.2f} "
                f"px=({filtered_x:.1f},{filtered_y:.1f})"
            )
            img.draw_string(ball.x, ball.y, msg, color=image.COLOR_RED)
        img.draw_string(
            8, 5, f"BALL {position_cm:+.2f} cm",
            color=image.COLOR_WHITE, scale=1.4, thickness=2,
        )

        # ==================== 在这里向主控发送数据 ====================
        # 建议每次识别成功都发送以下信息：
        #   valid = 1                         当前钢珠坐标有效
        #   x_cm = position_cm                沿摆杆轴线的位置，单位：厘米
        #   vx_pixel_s = position_filter.vx   估算的横向像素速度，单位：像素/秒
        #   confidence = ball.score           YOLO 检测置信度
        #   frame_time_ms = now_ms            本次结果的时间戳，单位：毫秒
        #
        # 串口输出：位置(cm), 速度(px/s)
        tx_data = f"p{position_cm:.2f},s{position_filter.vx:.1f}\n"
        serial.write(tx_data.encode())
    else:
        position_filter.mark_missing(now_ms)

        img.draw_string(
            8, 5, "BALL LOST",
            color=image.COLOR_RED, scale=1.4, thickness=2,
        )

        # 丢检时不发送串口数据，避免占用

    fps_str = "FPS:" + str(0 if loop_ms == 0 else 1000//loop_ms)
    img.draw_string(
        img.width() - image.string_size(fps_str, scale=1.4, thickness=2).width(), 5, fps_str,
        color=image.COLOR_GREEN, scale=1.4, thickness=2,
    )
    print_debug("draw and get position cost " + str(time.ticks_ms() - t) + "ms")

    if USE_JPEG:
        jpeg_server.write(img)
    disp.show(img)
