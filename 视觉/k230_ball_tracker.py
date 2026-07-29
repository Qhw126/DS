"""
2026 电赛 H 题 - K230 CanMV 钢球实时识别与定位

运行环境：
    CanMV IDE + K230 CanMV MicroPython（摄像头直接连接 K230）

注意：
    这是 K230 板端脚本，不要在 Windows Python 中运行。
    打开 CanMV IDE，连接 K230 后打开本文件，点击“运行”。
    调试完成后可保存到开发板为 main.py，实现上电自动运行。

串口输出（ASCII，方便单片机调试）：
    $BALL,有效标志,当前位置cm,目标cm,误差cm,FPS*CRC\r\n

例如：
    $BALL,1,+2.35,+0.00,-2.35,38.6*5A
    $BALL,0,+0.00,+0.00,+0.00,39.1*59

下位机可向 K230 发送：
    T=3.5\n     把目标位置设为 +3.5 cm
    T=-6.0\n    把目标位置设为 -6.0 cm
    Z\n         目标位置归零
"""

import time
import os
import gc
import math

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

try:
    from machine import UART, FPIOA
except ImportError:
    UART = None
    FPIOA = None


# ============================================================
# 一、必须根据现场安装情况修改的参数
# ============================================================

# 识别分辨率。640x480 定位精度较好；若帧率不足可改成 320x240，
# 同时需要把 ROI、标定点、球半径范围按比例缩小。
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# ROI：只识别摆杆凹槽所在区域，格式 (左上角x, 左上角y, 宽, 高)。
# 默认取画面中部横向区域。安装好摄像头后必须根据 IDE 图像修改。
ROI = (50, 150, 540, 180)

# 两点标定：摆杆有效行程 -12 cm 和 +12 cm 对应的像素中心。
# 标定点可以不完全水平，程序会沿两点连线方向做投影换算。
CALIB_LEFT_PX = (55, 240)    # 对应 -12.0 cm
CALIB_RIGHT_PX = (585, 240)  # 对应 +12.0 cm
CALIB_LEFT_CM = -12.0
CALIB_RIGHT_CM = 12.0

# 初始目标位置。
TARGET_CM = 0.0

# 检测模式：
# "blob"   暗色区域检测，速度快，推荐用于闭环控制；
# "circle" 霍夫圆，参数直观但计算量较大，可用于确认球半径。
DETECT_MODE = "blob"

# 钢球/阴影在浅色背景上通常表现为低亮度区域。
# GRAYSCALE 的 find_blobs 阈值格式为：(灰度最小值, 灰度最大值)。
# 灰度范围是 0~255。钢球找不到就增大 75，背景也被选中就减小。
DARK_BALL_THRESHOLD = (0, 75)

# 钢球候选框尺寸范围，单位像素。
# 直径 1 cm，若 24 cm 摆杆在图像中占 530 px，球直径约 22 px。
BALL_DIAMETER_MIN = 10
BALL_DIAMETER_MAX = 42

# 候选框长宽比范围。圆形目标应接近 1。
MIN_ASPECT_RATIO = 0.58
MAX_ASPECT_RATIO = 1.72

# Blob 内满足阈值的像素占外接矩形面积的比例。
# 太小通常是零散噪点；钢球有反光时不要设得过高。
MIN_FILL_RATIO = 0.28

# 最小暗色像素数和最小候选区域面积。
MIN_PIXELS = 45
MIN_AREA = 80

# 相邻帧最大跳变像素。设为 0 表示不限制。
MAX_JUMP_PX = 90

# 指数低通滤波系数：越大响应越快，越小显示越稳。
# 控制用途一般建议 0.45~0.75。
FILTER_ALPHA = 0.60

# 连续丢球多少帧后清除位置历史，允许在任意位置重新捕获。
LOST_RESET_FRAMES = 8

# 是否使用高斯滤波。打开后抗噪更好，但帧率可能下降。
USE_GAUSSIAN = False


# ============================================================
# 二、霍夫圆模式参数
# ============================================================

HOUGH_THRESHOLD = 1800
HOUGH_R_MIN = 5
HOUGH_R_MAX = 24
HOUGH_R_STEP = 2
HOUGH_X_STRIDE = 2
HOUGH_Y_STRIDE = 2


# ============================================================
# 三、显示与通信参数
# ============================================================

# True：启用画面显示。
ENABLE_DISPLAY = True

# 显示方式：
# "lcd"：立创·庐山派常用 ST7701 LCD，同时回传 CanMV IDE（推荐）
# "ide"：只回传 CanMV IDE，不驱动实体 LCD
DISPLAY_MODE = "lcd"

# 实体 LCD 默认分辨率。庐山派配套 ST7701 通常为 800x480。
LCD_WIDTH = 800
LCD_HEIGHT = 480

# 使用 LCD 时是否同时把画面回传到 CanMV IDE。
DISPLAY_TO_IDE = True

# 串口默认关闭，先确认摄像头识别正常，再改成 True。
ENABLE_UART = False
UART_ID = 1
UART_BAUDRATE = 115200
UART_TX_PIN = 3
UART_RX_PIN = 4

# 串口发送频率。视觉可以更快，但控制数据按固定频率发送。
UART_SEND_HZ = 50

# 每隔多少帧在 IDE 终端打印一次，避免 print 降低帧率。
PRINT_INTERVAL_FRAMES = 15

# 灰度画面绘图颜色，范围 0（黑）~255（白）。
# 浅色管道背景下，中轴线使用深灰，目标和钢球框使用白色。
ROI_COLOR = 150
AXIS_COLOR = 70
TARGET_COLOR = 255
BALL_COLOR = 255
CENTER_COLOR = 0
TEXT_COLOR = 255
LOST_TEXT_COLOR = 0


# ============================================================
# 四、工具函数
# ============================================================

def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def pixel_to_cm(px, py):
    """
    把全图像素坐标投影到左右标定点构成的轴线上，再线性映射到 cm。
    这种方法允许摄像头有少量旋转。
    """
    lx = CALIB_LEFT_PX[0]
    ly = CALIB_LEFT_PX[1]
    vx = CALIB_RIGHT_PX[0] - lx
    vy = CALIB_RIGHT_PX[1] - ly
    length2 = vx * vx + vy * vy
    if length2 <= 0:
        return 0.0

    ratio = ((px - lx) * vx + (py - ly) * vy) / length2
    return CALIB_LEFT_CM + ratio * (CALIB_RIGHT_CM - CALIB_LEFT_CM)


def cm_to_pixel(x_cm):
    """把物理坐标转换成显示目标点所需的像素坐标。"""
    cm_span = CALIB_RIGHT_CM - CALIB_LEFT_CM
    if cm_span == 0:
        return CALIB_LEFT_PX
    ratio = (x_cm - CALIB_LEFT_CM) / cm_span
    px = CALIB_LEFT_PX[0] + ratio * (
        CALIB_RIGHT_PX[0] - CALIB_LEFT_PX[0])
    py = CALIB_LEFT_PX[1] + ratio * (
        CALIB_RIGHT_PX[1] - CALIB_LEFT_PX[1])
    return (int(px + 0.5), int(py + 0.5))


def distance2(x1, y1, x2, y2):
    dx = x1 - x2
    dy = y1 - y2
    return dx * dx + dy * dy


def xor_checksum(text):
    """计算 ASCII 正文的 XOR 校验。"""
    value = 0
    for ch in text:
        value ^= ord(ch)
    return value


def make_packet(valid, position_cm, target_cm, error_cm, fps):
    """
    生成一行带校验的数据。
    校验范围是 '$' 和 '*' 之间的正文，不含 '$'。
    """
    body = "BALL,%d,%+.2f,%+.2f,%+.2f,%.1f" % (
        1 if valid else 0,
        position_cm,
        target_cm,
        error_cm,
        fps
    )
    return "$" + body + "*%02X\r\n" % xor_checksum(body)


def read_target_command(uart, old_target):
    """
    读取下位机目标指令。
    为保持主循环非阻塞，只读取当前已经到达的数据。
    """
    if uart is None:
        return old_target

    try:
        data = uart.read()
        if not data:
            return old_target
        text = data.decode().strip()
        # 一次收到多行时取最后一条有效命令。
        lines = text.replace("\r", "\n").split("\n")
        new_target = old_target
        for line in lines:
            line = line.strip()
            if not line:
                continue
            if line == "Z" or line == "z":
                new_target = 0.0
            elif line.startswith("T=") or line.startswith("t="):
                try:
                    value = float(line[2:])
                    new_target = clamp(value, CALIB_LEFT_CM, CALIB_RIGHT_CM)
                except ValueError:
                    pass
        return new_target
    except BaseException:
        return old_target


def init_uart():
    """配置 UART1 GPIO3/4。若你的板卡引脚不同，只改文件顶部参数。"""
    if not ENABLE_UART:
        return None
    if UART is None or FPIOA is None:
        print("UART module unavailable")
        return None

    fpioa = FPIOA()
    if UART_ID == 1:
        fpioa.set_function(UART_TX_PIN, fpioa.UART1_TXD)
        fpioa.set_function(UART_RX_PIN, fpioa.UART1_RXD)
        uart_num = UART.UART1
    elif UART_ID == 2:
        fpioa.set_function(UART_TX_PIN, fpioa.UART2_TXD)
        fpioa.set_function(UART_RX_PIN, fpioa.UART2_RXD)
        uart_num = UART.UART2
    elif UART_ID == 4:
        fpioa.set_function(UART_TX_PIN, fpioa.UART4_TXD)
        fpioa.set_function(UART_RX_PIN, fpioa.UART4_RXD)
        uart_num = UART.UART4
    else:
        raise ValueError("UART_ID must be 1, 2 or 4")

    return UART(
        uart_num,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=0
    )


# ============================================================
# 五、钢球识别
# ============================================================

def detect_ball_blob(img, last_x, last_y):
    """
    暗色 Blob 检测。

    返回：
        (cx, cy, radius, score)，未找到时返回 None。
    """
    blobs = img.find_blobs(
        [DARK_BALL_THRESHOLD],
        roi=ROI,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=True,
        margin=2
    )

    best = None
    best_score = -1000000
    max_jump2 = MAX_JUMP_PX * MAX_JUMP_PX

    for blob in blobs:
        w = blob.w()
        h = blob.h()
        if w <= 0 or h <= 0:
            continue

        diameter = (w + h) * 0.5
        if diameter < BALL_DIAMETER_MIN or diameter > BALL_DIAMETER_MAX:
            continue

        aspect = w / h
        if aspect < MIN_ASPECT_RATIO or aspect > MAX_ASPECT_RATIO:
            continue

        rect_area = w * h
        fill_ratio = blob.pixels() / rect_area
        if fill_ratio < MIN_FILL_RATIO:
            continue

        cx = blob.cx()
        cy = blob.cy()

        # 已经锁定钢球时，排除不可能的瞬间大跳变。
        if last_x is not None and MAX_JUMP_PX > 0:
            jump2 = distance2(cx, cy, last_x, last_y)
            if jump2 > max_jump2:
                continue
        else:
            jump2 = 0

        # 越接近圆形、填充率越合理、越靠近上一位置，得分越高。
        aspect_error = abs(w - h) / diameter
        score = 200.0
        score -= aspect_error * 100.0
        score += fill_ratio * 40.0
        if last_x is not None:
            score -= jump2 / max(1, max_jump2) * 35.0

        if score > best_score:
            best_score = score
            radius = int(diameter * 0.5 + 0.5)
            best = (cx, cy, radius, score)

    return best


def detect_ball_circle(img, last_x, last_y):
    """
    霍夫圆检测。适合确认半径范围，帧率通常低于 Blob 模式。
    """
    circles = img.find_circles(
        roi=ROI,
        x_stride=HOUGH_X_STRIDE,
        y_stride=HOUGH_Y_STRIDE,
        threshold=HOUGH_THRESHOLD,
        x_margin=10,
        y_margin=10,
        r_margin=8,
        r_min=HOUGH_R_MIN,
        r_max=HOUGH_R_MAX,
        r_step=HOUGH_R_STEP
    )

    best = None
    best_score = -1000000
    max_jump2 = MAX_JUMP_PX * MAX_JUMP_PX

    for circle in circles:
        cx = circle.x()
        cy = circle.y()
        radius = circle.r()

        if last_x is not None and MAX_JUMP_PX > 0:
            jump2 = distance2(cx, cy, last_x, last_y)
            if jump2 > max_jump2:
                continue
        else:
            jump2 = 0

        score = circle.magnitude()
        if last_x is not None:
            score -= jump2 * 0.1

        if score > best_score:
            best_score = score
            best = (cx, cy, radius, score)

    return best


def draw_overlay(img, detection, valid, position_cm, target_cm,
                 error_cm, fps):
    """在 IDE 画面上绘制 ROI、标定轴、目标点、钢球和文字。"""
    rx, ry, rw, rh = ROI
    img.draw_rectangle(rx, ry, rw, rh, color=ROI_COLOR, thickness=1)

    img.draw_line(
        CALIB_LEFT_PX[0], CALIB_LEFT_PX[1],
        CALIB_RIGHT_PX[0], CALIB_RIGHT_PX[1],
        color=AXIS_COLOR, thickness=2
    )

    zero_px = cm_to_pixel(0.0)
    target_px = cm_to_pixel(target_cm)
    img.draw_cross(zero_px[0], zero_px[1],
                   color=AXIS_COLOR, size=10, thickness=2)
    img.draw_cross(target_px[0], target_px[1],
                   color=TARGET_COLOR, size=18, thickness=3)

    if valid and detection is not None:
        cx, cy, radius, score = detection
        img.draw_circle(cx, cy, radius,
                        color=BALL_COLOR, thickness=3)
        img.draw_cross(cx, cy, color=CENTER_COLOR,
                       size=8, thickness=2)
        status = "BALL X=%+.2fcm ERR=%+.2fcm" % (
            position_cm, error_cm)
        color = TEXT_COLOR
    else:
        status = "BALL LOST"
        color = LOST_TEXT_COLOR

    # 新版 CanMV 支持中文字体接口，但英文字符串显示速度和兼容性更好。
    img.draw_string_advanced(
        8, 8, 22, status, color=color
    )
    img.draw_string_advanced(
        8, 35, 20,
        "TARGET=%+.2fcm FPS=%.1f MODE=%s" % (
            target_cm, fps, DETECT_MODE),
        color=TEXT_COLOR
    )


# ============================================================
# 六、主程序
# ============================================================

def main():
    sensor = None
    uart = None
    media_ready = False
    display_ready = False

    target_cm = clamp(TARGET_CM, CALIB_LEFT_CM, CALIB_RIGHT_CM)
    last_x = None
    last_y = None
    filtered_cm = 0.0
    filter_initialized = False
    lost_frames = 0
    frame_count = 0
    last_send_ms = 0
    send_period_ms = int(1000 / max(1, UART_SEND_HZ))

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)

        # 创建并复位 K230 板载摄像头。
        sensor = Sensor(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
        sensor.reset()
        sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
        # 灰度输出：每像素 1 字节，减少内存带宽和阈值计算量。
        sensor.set_pixformat(Sensor.GRAYSCALE)

        # 摄像头方向不对时，取消下面对应行的注释。
        # sensor.set_hmirror(True)
        # sensor.set_vflip(True)

        if ENABLE_DISPLAY:
            if DISPLAY_MODE == "lcd":
                # ST7701 驱动实体 LCD；to_ide=True 时同时回传 IDE。
                Display.init(
                    Display.ST7701,
                    width=LCD_WIDTH,
                    height=LCD_HEIGHT,
                    to_ide=DISPLAY_TO_IDE
                )
            else:
                # VIRT 只用于 IDE。必须显式设置 to_ide=True，
                # 否则部分新固件中主循环正常运行但 IDE 没有实时画面。
                Display.init(
                    Display.VIRT,
                    width=IMAGE_WIDTH,
                    height=IMAGE_HEIGHT,
                    fps=60,
                    to_ide=True
                )
            display_ready = True

        MediaManager.init()
        media_ready = True
        sensor.run()

        uart = init_uart()
        clock = time.clock()

        print("K230 ball tracker started")
        print("mode:", DETECT_MODE)
        print("ROI:", ROI)
        print("cm/pixel:",
              (CALIB_RIGHT_CM - CALIB_LEFT_CM) /
              math.sqrt(
                  (CALIB_RIGHT_PX[0] - CALIB_LEFT_PX[0]) ** 2 +
                  (CALIB_RIGHT_PX[1] - CALIB_LEFT_PX[1]) ** 2
              ))

        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()

            if USE_GAUSSIAN:
                # size=1 对应较小卷积核；滤波会直接修改当前图像。
                img.gaussian(1)

            if DETECT_MODE == "circle":
                detection = detect_ball_circle(img, last_x, last_y)
            else:
                detection = detect_ball_blob(img, last_x, last_y)

            valid = detection is not None
            position_cm = filtered_cm
            error_cm = target_cm - position_cm

            if valid:
                cx, cy, radius, score = detection
                raw_cm = pixel_to_cm(cx, cy)

                if not filter_initialized:
                    filtered_cm = raw_cm
                    filter_initialized = True
                else:
                    filtered_cm += FILTER_ALPHA * (
                        raw_cm - filtered_cm)

                position_cm = filtered_cm
                error_cm = target_cm - position_cm
                last_x = cx
                last_y = cy
                lost_frames = 0
            else:
                lost_frames += 1
                if lost_frames >= LOST_RESET_FRAMES:
                    last_x = None
                    last_y = None
                    filter_initialized = False

            target_cm = read_target_command(uart, target_cm)
            if valid:
                error_cm = target_cm - position_cm

            fps_value = clock.fps()
            draw_overlay(
                img, detection, valid, position_cm,
                target_cm, error_cm, fps_value
            )

            if ENABLE_DISPLAY:
                if DISPLAY_MODE == "lcd":
                    # 640x480 图像在 800x480 LCD 上水平居中。
                    Display.show_image(
                        img,
                        x=(LCD_WIDTH - IMAGE_WIDTH) // 2,
                        y=(LCD_HEIGHT - IMAGE_HEIGHT) // 2
                    )
                else:
                    Display.show_image(img)

            # 使用 ticks_diff 正确处理毫秒计数器回绕。
            now_ms = time.ticks_ms()
            if uart is not None and time.ticks_diff(
                    now_ms, last_send_ms) >= send_period_ms:
                packet = make_packet(
                    valid,
                    position_cm if valid else 0.0,
                    target_cm,
                    error_cm if valid else 0.0,
                    fps_value
                )
                uart.write(packet)
                last_send_ms = now_ms

            frame_count += 1
            if frame_count % PRINT_INTERVAL_FRAMES == 0:
                if valid:
                    print("x=%+.2f target=%+.2f error=%+.2f fps=%.1f" % (
                        position_cm, target_cm, error_cm, fps_value))
                else:
                    print("BALL LOST fps=%.1f" % fps_value)

            # 定期回收 MicroPython 堆，避免长时间运行碎片积累。
            if frame_count % 30 == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("user stopped")
    except BaseException as error:
        print("Exception:", error)
        raise
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except BaseException:
                pass

        if uart is not None:
            try:
                uart.deinit()
            except BaseException:
                pass

        if display_ready:
            try:
                Display.deinit()
            except BaseException:
                pass

        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)

        if media_ready:
            try:
                MediaManager.deinit()
            except BaseException:
                pass


main()
