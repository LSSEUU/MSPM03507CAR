from maix import app, camera, display, image, nn, uart
import struct

# ============================================================
# 1. 串口初始化（与第一个程序一致，可自行调整波特率）
# ============================================================
serial = uart.UART("/dev/ttyS0", 115200)   # 若无需发送可注释掉

# ============================================================
# 2. 模型与摄像头初始化
# ============================================================
detector = nn.YOLO26(model="/root/my_models/yolo26_ball_2.mud", dual_buff=True)
cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
disp = display.Display()

print(f"YOLO26n 已加载, 分辨率: {detector.input_width()}x{detector.input_height()}")
print("按设备按键退出")

# ============================================================
# 3. 辅助：绘制白色十字（中心点）
# ============================================================
def draw_white_cross(img, cx, cy, length=10, color=image.COLOR_WHITE):
    """在图像中心绘制白色十字（水平和垂直线段）"""
    # 水平线
    img.draw_line(cx - length, cy, cx + length, cy, color=color, thickness=1)
    # 垂直线
    img.draw_line(cx, cy - length, cx, cy + length, color=color, thickness=1)

# ============================================================
# 4. 主循环
# ============================================================
while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, conf_th=0.5, iou_th=0.45)

    for obj in objs:
        # --- 计算中心坐标 ---
        cx = obj.x + obj.w // 2
        cy = obj.y + obj.h // 2

        # --- 绘制检测框 (红色) ---
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)

        # --- 绘制标签 ---
        msg = f"{detector.labels[obj.class_id]}: {obj.score:.2f}"
        img.draw_string(obj.x, obj.y, msg, color=image.COLOR_RED)

        # --- 1. 绘制白色十字 (中心标记) ---
        draw_white_cross(img, cx, cy, length=12)

        # --- 2. 控制台打印输出 ---
        print(f"中心: ({cx}, {cy}) | 置信度: {obj.score:.2f}")

        # --- 3. 通过串口发送实时坐标 (文本格式) ---
        # 格式：cx,cy\n  例如 "120,85\n"
        coord_str = f"{cx},{cy}\n"
        serial.write(coord_str.encode())   # 注意编码为 bytes

    # 显示图像
    disp.show(img)