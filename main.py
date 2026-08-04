import time, os, sys, gc, math
from media.sensor import *
from media.display import *
from media.media import *
from ybUtils.YbUart import YbUart

uart = YbUart(baudrate=115200)

# ---------- 图像参数 ----------
PICTURE_WIDTH  = 200
PICTURE_HEIGHT = 120
DISPLAY_WIDTH  = 640
DISPLAY_HEIGHT = 480

# ---------- 循迹参数 ----------
LINE_GRAY_THRESHOLD = (0, 90)        # 黑线灰度阈值
ROI_TOP_RATIO    = 0.1
LOCAL_HALF_ROWS  = 25
DEAD_ANGLE       = 2.0
SMOOTH_FACTOR    = 0.3
PIXELS_THRESHOLD = 30
AREA_THRESHOLD   = 30
IGNORE_RECT_LEFT = (0, PICTURE_HEIGHT-40, 60, 40)   # 左下角忽略区域
IGNORE_RECT_RIGHT = (PICTURE_WIDTH-60, PICTURE_HEIGHT-40, 60, 40)  # 右下角

# ---------- 圆检测参数 ----------
CIRCLE_THRESHOLD = 2800
CIRCLE_R_MIN = 20
CIRCLE_R_MAX = 30
CIRCLE_MAG_MIN = 15
GAUSSIAN_KERNEL = 1
AREA_OF_CENTER = 3

# ---------- 全局状态 ----------
MODE_NONE   = 0
MODE_LINE   = 1
MODE_CIRCLE = 2
current_mode = MODE_NONE
last_angle = 0.0

# ---------- 循迹帧率统计 ----------
line_fps_counter = 0
line_fps_timer = time.ticks_ms()

# 主循环周期（ms），决定发送频率
LOOP_PERIOD_MS = 10   # 约100 Hz，实际受处理时间限制

# ---------- 串口接收 ----------
def receive_serial_data():
    data = uart.read(1)
    if data is not None:
        return chr(data[0])
    return None

# ---------- 传感器与显示初始化 ----------
def init_sensor():
    sensor = Sensor(id=2)
    sensor.reset()
    sensor.set_framesize(width=PICTURE_WIDTH, height=PICTURE_HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
    return sensor

def init_display():
    Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)

# ---------- 循迹相关函数 ----------
def linear_fit_xy(points):
    n = len(points)
    if n < 2: return None
    sum_y = sum(p[1] for p in points)
    sum_x = sum(p[0] for p in points)
    sum_xy = sum(p[0]*p[1] for p in points)
    sum_y2 = sum(p[1]*p[1] for p in points)
    denom = n * sum_y2 - sum_y * sum_y
    if abs(denom) < 1e-6: return None
    a = (n * sum_xy - sum_x * sum_y) / denom
    b = (sum_x - a * sum_y) / n
    return a, b

def fit_with_outlier_removal(points, max_residual=3):
    if len(points) < 3: return linear_fit_xy(points)
    reg = linear_fit_xy(points)
    if reg is None: return None
    a, b = reg
    inliers = [(x,y) for (x,y) in points if abs(x - (a*y+b)) <= max_residual]
    if len(inliers) < 2: return None
    return linear_fit_xy(inliers)

def fill_rect(img, x, y, w, h, color=255):
    for dy in range(h):
        for dx in range(w):
            px, py = x+dx, y+dy
            if 0 <= px < img.width() and 0 <= py < img.height():
                img.set_pixel(px, py, color)

def line_follow(gray_img, disp_img):
    global last_angle, line_fps_counter, line_fps_timer
    W, H = gray_img.width(), gray_img.height()

    # 帧率统计
    line_fps_counter += 1
    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, line_fps_timer)
    if elapsed >= 1000:
        fps = line_fps_counter * 1000 / elapsed
        print(f"Line FPS: {fps:.1f}")
        line_fps_counter = 0
        line_fps_timer = now
    y_ref = H // 2
    y_min = max(y_ref - LOCAL_HALF_ROWS, int(H * ROI_TOP_RATIO))
    y_max = min(y_ref + LOCAL_HALF_ROWS, H - 1)

    # 1. 去噪：只保留最大黑块
    blobs = gray_img.find_blobs([LINE_GRAY_THRESHOLD],
                                pixels_threshold=PIXELS_THRESHOLD,
                                area_threshold=AREA_THRESHOLD)
    clean_binary = gray_img.copy()
    for y in range(H):
        for x in range(W):
            clean_binary.set_pixel(x, y, 255)
    if blobs:
        main_blob = max(blobs, key=lambda b: b.area())
        for y in range(main_blob.y(), main_blob.y() + main_blob.h()):
            for x in range(main_blob.x(), main_blob.x() + main_blob.w()):
                if 0 <= x < W and 0 <= y < H:
                    if gray_img.get_pixel(x, y) <= LINE_GRAY_THRESHOLD[1]:
                        clean_binary.set_pixel(x, y, 0)

    # === 新增：两次腐蚀，去除细小噪点，增强黑线 ===
    clean_binary.erode(3)   # 第一次腐蚀
    clean_binary.erode(3)   # 第二次腐蚀
    clean_binary.erode(3)   # 腐蚀1
    clean_binary.erode(3)   # 腐蚀2
    clean_binary.erode(3)
    clean_binary.erode(3)
    clean_binary.dilate(3)  # 膨胀1
    clean_binary.dilate(3)  # 膨胀2
    clean_binary.dilate(3)
    clean_binary.dilate(3)
    clean_binary.dilate(3)
    clean_binary.dilate(3)
    clean_binary.dilate(3)

    # 2. 排除左下角干扰
    ix, iy, iw, ih = IGNORE_RECT_LEFT
    fill_rect(clean_binary, ix, iy, iw, ih, 255)
    ix, iy, iw, ih = IGNORE_RECT_RIGHT
    fill_rect(clean_binary, ix, iy, iw, ih, 255)

    # 3. 扫描左右边缘点
    left_pts, right_pts = [], []
    for y in range(y_min, y_max+1):
        lx = None
        for x in range(W):
            if clean_binary.get_pixel(x, y) == 0:
                lx = x; break
        rx = None
        for x in range(W-1, -1, -1):
            if clean_binary.get_pixel(x, y) == 0:
                rx = x; break
        if lx is not None and rx is not None and (rx - lx) >= 5:
            left_pts.append((lx, y))
            right_pts.append((rx, y))

    # 点数不足或拟合失败 → 不发送任何数据，直接返回
    if len(left_pts) < 5 or len(right_pts) < 5:
        print("No line")
        return

    left_reg = fit_with_outlier_removal(left_pts)
    right_reg = fit_with_outlier_removal(right_pts)
    if left_reg is None or right_reg is None:
        print("Fit fail")
        return

    a_l, b_l = left_reg
    a_r, b_r = right_reg
    cx_ref = ((a_l + a_r) * y_ref + b_l + b_r) / 2
    a_center = (a_l + a_r) / 2
    angle_deg = -math.degrees(math.atan(a_center))
    if abs(angle_deg) < DEAD_ANGLE: angle_deg = 0.0
    filtered_angle = last_angle * (1 - SMOOTH_FACTOR) + angle_deg * SMOOTH_FACTOR
    last_angle = filtered_angle

    # 可视化
    for x in range(0, W, 4):
        disp_img.draw_line(x, y_ref, x+2, y_ref, color=(0,0,255), thickness=1)
    disp_img.draw_line(int(a_l*y_min+b_l), y_min, int(a_l*y_max+b_l), y_max, color=(255,255,255), thickness=1)
    disp_img.draw_line(int(a_r*y_min+b_r), y_min, int(a_r*y_max+b_r), y_max, color=(255,255,255), thickness=1)
    disp_img.draw_line(int(cx_ref)-4, y_ref, int(cx_ref)+4, y_ref, color=(0,255,0), thickness=1)
    disp_img.draw_line(int(cx_ref), y_ref-4, int(cx_ref), y_ref+4, color=(0,255,0), thickness=1)

    # ---------- 串口发送 ----------
    # 1. 发送角度 (A3 B3 angle_H angle_L FF)
    offset_px = cx_ref - W / 2
    offset_int = int(offset_px * 100)          # 放大100倍，保留精度
    offset_int = max(-32768, min(32767, offset_int))
    angle_int = int(filtered_angle * 100)
    angle_int = max(-32768, min(32767, angle_int))
    uart.write(bytearray([0xA3, 0xB3, (angle_int >> 8) & 0xFF, angle_int & 0xFF,(offset_int >> 8) & 0xFF, offset_int & 0xFF, 0xFF]))
    print(f"angle，offset: {filtered_angle:.2f}{offset_int:.2f}")


# ---------- 圆检测函数 ----------
def circle_detect(img, disp_img):
    img_blur = img.gaussian(GAUSSIAN_KERNEL)
    circles = img_blur.find_circles(
        threshold=CIRCLE_THRESHOLD, x_margin=10, y_margin=10, r_margin=10,
        r_min=CIRCLE_R_MIN, r_max=CIRCLE_R_MAX, r_step=1
    )
    if not circles:
        return   # 无圆不发送
    circles = sorted(circles, key=lambda c: c.magnitude(), reverse=True)
    for c in circles:
        if c.magnitude() < CIRCLE_MAG_MIN:
            continue
        disp_img.draw_circle(c.x(), c.y(), c.r(), color=(255,0,0), thickness=2)
        dx = c.x() - PICTURE_WIDTH / 2
        dy = c.y() - PICTURE_HEIGHT / 2
        if dx*dx + dy*dy < AREA_OF_CENTER:
            dat = 'O'
        elif abs(dx) > abs(dy):
            dat = 'W' if dx > 0 else 'E'
        else:
            dat = 'N' if dy > 0 else 'S'
        # 发送方向 (A3 B4 dir_char FF)
        uart.write(bytearray([0xA3, 0xB4, ord(dat), 0xFF]))
        print(dat)
        return   # 只处理第一个有效圆

# ---------- 主循环 ----------
def main():
    global current_mode
    try:
        sensor = init_sensor()
        init_display()
        MediaManager.init()
        sensor.run()

        x_off = (DISPLAY_WIDTH - PICTURE_WIDTH) // 2
        y_off = (DISPLAY_HEIGHT - PICTURE_HEIGHT) // 2

        while True:
            loop_start = time.ticks_ms()

            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            disp_img = img.binary([LINE_GRAY_THRESHOLD], invert=True).copy(sensor.RGB565)

            # 检查串口命令
            cmd = receive_serial_data()
            if cmd is not None:
                cmd = cmd.lower()
                if cmd == 'f':
                    current_mode = MODE_LINE
                    print("Mode: Line Following")
                elif cmd == 'c':
                    current_mode = MODE_CIRCLE
                    print("Mode: Circle Detection")
                elif cmd == 'x':
                    current_mode = MODE_NONE
                    print("Mode: Stopped")

            # 根据模式执行
            if current_mode == MODE_LINE:
                line_follow(img, disp_img)
            elif current_mode == MODE_CIRCLE:
                circle_detect(img, disp_img)

            # 显示
            try:
                Display.clear(0xFFFF)
            except:
                pass
            Display.show_image(disp_img, x=x_off, y=y_off)

            # 固定频率控制
            elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
            sleep_time = max(0, LOOP_PERIOD_MS - elapsed)
            time.sleep_ms(sleep_time)

    except KeyboardInterrupt:
        print("用户中断")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        if 'sensor' in locals() and isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()

if __name__ == "__main__":
    main()
