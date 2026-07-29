import gc
import os
import time
import traceback

from maix import app, camera, display, image, nn, uart


APP_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(APP_DIR, "models", "yolo26_ball_2.mud")

UART_PORT = "/dev/ttyS0"
UART_BAUD = 115200
CONF_THRESHOLD = 0.5
IOU_THRESHOLD = 0.45
CAMERA_WARMUP_FRAMES = 30
HEALTH_INTERVAL_SECONDS = 2
RETRY_MAX_SECONDS = 10


def _sleep_until_retry(seconds):
    """分段等待，以便 USER 键或 MaixVision 停止请求能尽快生效。"""
    deadline = time.time() + seconds
    while not app.need_exit() and time.time() < deadline:
        time.sleep(min(0.2, max(0, deadline - time.time())))


def _safe_close(name, resource, method="close"):
    if resource is None:
        return
    close_fn = getattr(resource, method, None)
    if close_fn is None:
        return
    try:
        close_fn()
    except Exception as exc:
        print("[cleanup] {} failed: {}".format(name, exc))


def _check_model_files():
    if not os.path.isfile(MODEL_PATH):
        raise RuntimeError("model descriptor not found: {}".format(MODEL_PATH))

    model_dir = os.path.dirname(MODEL_PATH)
    cvimodel_path = os.path.join(model_dir, "yolo26_ball_2.cvimodel")
    if not os.path.isfile(cvimodel_path):
        raise RuntimeError("model data not found: {}".format(cvimodel_path))


def run_once(attempt):
    """初始化并运行一次；异常交给外层清理后重试。"""
    serial = None
    detector = None
    disp = None
    cam = None
    stage = "model file check"

    try:
        print("[app] run attempt {}".format(attempt))
        _check_model_files()

        stage = "model initialization"
        detector = nn.YOLO26(model=MODEL_PATH, dual_buff=True)
        width = detector.input_width()
        height = detector.input_height()
        detect_format = detector.input_format()
        if width <= 0 or height <= 0:
            raise RuntimeError(
                "invalid detector input size: {}x{}".format(width, height)
            )
        print("[model] YOLO26 ready: {}x{}, {}".format(width, height, MODEL_PATH))

        stage = "UART enumeration"
        available_ports = uart.list_devices()
        print("[uart] available ports: {}".format(available_ports))

        stage = "UART initialization"
        serial = uart.UART(UART_PORT, UART_BAUD)
        if serial is None:
            raise RuntimeError("UART initialization returned None")
        print("[uart] ready: {} @ {}".format(UART_PORT, UART_BAUD))

        stage = "display initialization"
        disp = display.Display()
        if disp is None:
            raise RuntimeError("display initialization returned None")
        print("[display] local camera preview ready")

        # 使用模型所需的单路摄像头，同时把同一帧显示到设备屏幕。
        stage = "camera initialization"
        cam = camera.Camera(width, height, detect_format)
        if cam is None:
            raise RuntimeError("camera initialization returned None")
        print("[camera] detection only: {}x{}".format(width, height))

        stage = "camera warmup"
        for _ in range(CAMERA_WARMUP_FRAMES):
            warmup_frame = cam.read()
            if warmup_frame is None:
                raise RuntimeError("camera returned an empty warmup frame")
            disp.show(warmup_frame)
        print("[camera] warmup complete: {} frames displayed".format(
            CAMERA_WARMUP_FRAMES
        ))
        print("[app] recognition, UART and local display active")

        started_at = time.time()
        last_health_at = started_at
        last_frame_at = started_at
        display_fps = 0.0
        frame_count = 0
        detection_count = 0
        uart_ok_count = 0
        uart_fail_count = 0

        while not app.need_exit():
            stage = "camera read"
            frame = cam.read()
            if frame is None:
                raise RuntimeError("camera returned an empty frame")
            frame_count += 1

            stage = "model detection"
            objects = detector.detect(
                frame,
                conf_th=CONF_THRESHOLD,
                iou_th=IOU_THRESHOLD,
            )
            if objects is None:
                objects = []
            detection_count += len(objects)

            stage = "UART write"
            for obj in objects:
                center_x = obj.x + obj.w // 2
                center_y = obj.y + obj.h // 2

                stage = "object overlay drawing"
                frame.draw_rect(
                    obj.x,
                    obj.y,
                    obj.w,
                    obj.h,
                    color=image.COLOR_RED,
                    thickness=2,
                )
                frame.draw_cross(center_x, center_y, image.COLOR_WHITE)

                label = "ball"
                labels = getattr(detector, "labels", None)
                if labels and 0 <= obj.class_id < len(labels):
                    label = labels[obj.class_id]
                label_y = obj.y - 18 if obj.y >= 18 else obj.y + 2
                frame.draw_string(
                    obj.x,
                    label_y,
                    "{} {:.2f} ({},{})".format(
                        label, obj.score, center_x, center_y
                    ),
                    color=image.COLOR_RED,
                )

                stage = "UART write"
                payload = "{},{}\n".format(center_x, center_y)
                written = serial.write_str(payload)
                if written == len(payload):
                    uart_ok_count += 1
                else:
                    uart_fail_count += 1
                    print(
                        "[uart] incomplete write for center ({}, {}): "
                        "expected {}, wrote {}".format(
                            center_x, center_y, len(payload), written
                        )
                    )

            stage = "FPS calculation"
            now = time.time()
            frame_seconds = now - last_frame_at
            if frame_seconds > 0:
                current_fps = 1.0 / frame_seconds
                if display_fps <= 0:
                    display_fps = current_fps
                else:
                    display_fps = display_fps * 0.8 + current_fps * 0.2
            last_frame_at = now

            stage = "FPS overlay drawing"
            frame.draw_string(
                8,
                8,
                "FPS: {:.1f}".format(display_fps),
                color=image.COLOR_GREEN,
            )

            stage = "display output"
            disp.show(frame)

            stage = "health reporting"
            if now - last_health_at >= HEALTH_INTERVAL_SECONDS:
                print(
                    "[health] uptime_s={:.1f} fps={:.1f} frames={} "
                    "detections={} uart_ok={} uart_fail={}".format(
                        max(0, now - started_at),
                        display_fps,
                        frame_count,
                        detection_count,
                        uart_ok_count,
                        uart_fail_count,
                    )
                )
                last_health_at = now

        print("[app] stop requested")
    except Exception as exc:
        raise RuntimeError("{} failed: {}".format(stage, exc)) from exc
    finally:
        _safe_close("camera", cam)
        _safe_close("display", disp)
        _safe_close("UART", serial)
        cam = None
        disp = None
        serial = None
        detector = None
        gc.collect()


def main():
    retry_seconds = 2
    attempt = 1
    while not app.need_exit():
        try:
            run_once(attempt)
            break
        except Exception as exc:
            print("[app] runtime failure on attempt {}: {}".format(attempt, exc))
            traceback.print_exc()
            if app.need_exit():
                break
            print(
                "[app] retry {} in {} seconds".format(attempt + 1, retry_seconds)
            )
            _sleep_until_retry(retry_seconds)
            attempt += 1
            retry_seconds = min(retry_seconds * 2, RETRY_MAX_SECONDS)

    print("[app] exited cleanly")


if __name__ == "__main__":
    main()
