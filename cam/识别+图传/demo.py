from maix import time, webrtc, camera, image, display,network,err
import platform
import cv2
import numpy as np

_SSID     = platform.node()
_PASSWORD = "88888888"
_DEV_IP   = ''

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

def start_ap(ssid, password):
    '''
    设置wifi模块工作在AP模式，分享出WIFI热点
    ssid : 热点名称
    password : 连接密码
    '''
    w = network.wifi.Wifi()
    if not w.is_ap_mode():
        e = w.start_ap(ssid, password)
        err.check_raise(e, "start wifi ap failed")

    global _DEV_IP
    _DEV_IP = w.get_ip()
    print("Start AP success, ip:", _DEV_IP)

def find_circle(img):
    img_cv = image.image2cv(img, False, False) 

    # 转灰度
    gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY) 

    # 高斯模糊去噪声
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 边缘检测，阈值 0，150
    edged = cv2.Canny(blurred, 50, 150)

    # 膨胀处理
    #dilated = cv2.dilate(edged, kernel, iterations=1)

    # 腐蚀处理
    #ceroded = cv2.erode(dilated, kernel, iterations=1)

    # img_show = image.cv2image(ceroded, copy=False) 
    # disp.show(img_show)

    # 找轮廓
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 圆度阈值
    min_circularity = 0.85

    for cnt in contours:
        # 分别计算轮廓面积和周长
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)

        # 过滤掉小轮廓
        if perimeter < 100 or area < 100:
            continue
        
        # 计算圆度(该指标是度量区域形状接近圆形的指标，值越接近1，形状越接近圆形)
        circularity = 4 * np.pi * area / (perimeter**2)
        if circularity > min_circularity:
            # 最优椭圆拟合(内部采用最小二乘法进行拟合计算)
            center, axes, angle = cv2.fitEllipse(cnt)

            # 画椭圆并标识圆心
            cv2.ellipse(img_cv, (center, axes, angle), (0, 255, 0), 1)
            img.draw_cross(int(center[0]), int(center[1]), image.COLOR_BLUE)

    # 显示
    img_show = image.cv2image(img_cv, False, False) 
    # return img_show

def main():
    # 1.开启wifi热点
    start_ap(_SSID, _PASSWORD)

    # 2. 开启webrtc推流
    disp = display.Display()
    cam = camera.Camera(320, 240, image.Format.FMT_YVU420SP)
    cam2 = cam.add_channel(disp.width(), disp.height())

    server = webrtc.WebRTC()
    server.bind_camera(cam)
    server.start()

    while True:
        img = cam2.read()
        find_circle(img)

        img.draw_string(10, 20, 'IP = '+_DEV_IP, image.Color.from_rgb(0, 0, 255), scale = 2)
        disp.show(img)

if __name__ == "__main__":
    main()

