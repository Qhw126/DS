import gc
import os
import sys
import time

from machine import FPIOA, UART
from media.sensor import Sensor
from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
import gc
import os
import sys
import time

from machine import FPIOA, UART
from media.sensor import Sensor
from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8


# YOLO / display
KMODEL_PATH = "/sdcard/best.kmodel"                     #YOLO模型的位置
LABELS = ["ball"]                                       #类别标签
CAMERA_ID = 2                                           #相机ID
MODEL_INPUT_SIZE = [320, 320]                           #YOLO模型输入尺寸
IMAGE_SIZE = [640, 480]                                 #图像尺寸
DISPLAY_SIZE = [800, 480]                               #显示尺寸
CONF_THRESHOLD = 0.50                                   #置信度阈值
NMS_THRESHOLD = 0.45                                    #非极大值抑制阈值

# Ball is only accepted inside the horizontal pipe region.
# This is the 640x240 ROI (0, 96, 640, 48) scaled to 640x480.
DETECTION_ROI = (0, 192, 640, 96)                       #检测区域

# Traditional position calibration.
# Adjust these two pixel positions after placing the ball at both physical ends.
CALIB_LEFT_PX = 55                                      # 左侧校准像素位置
CALIB_RIGHT_PX = 585                                    # 右侧校准像素位置
CALIB_LEFT_CM = 12.0                                    # 左侧校准厘米位置
CALIB_RIGHT_CM = -12.0                                  # 右侧校准厘米位置
X_ZERO_PIXEL = 320                                      # X轴零点像素位置
X_ZERO_DEAD_ZONE = 3                                    # X轴零点死区
POSITION_FILTER_ALPHA = 0.65                            # 位置滤波器Alpha值

# UART2: IO11 TX -> STM32 RX, IO12 RX -> STM32 TX
UART_BAUD = 115200
UART_TX_PIN =11
UART_RX_PIN = 12
UART_SEND_INTERVAL_MS = 20                              #UART发送间隔毫秒
UART_PACKET = bytearray(8)                              #UART数据包格式

# Detection stability
MAX_CENTER_JUMP = 120                                   #最大中心跳跃
LOST_HOLD_FRAMES = 2                                    #丢失保持帧数
PRINT_INTERVAL = 20                                     #打印间隔


def clamp(value, minimum, maximum):                     #限制值在最小值和最大值之间
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def pixel_x_to_centered_x(pixel_x):                     #将像素X坐标转换为中心化X坐标
    """Image center=0, left positive, right negative."""
    value = X_ZERO_PIXEL - int(pixel_x)
    if -X_ZERO_DEAD_ZONE <= value <= X_ZERO_DEAD_ZONE:
        return 0
    return value


def pixel_x_to_cm(pixel_x):                             #将像素X坐标转换为物理位置
    """Linearly map YOLO center X to physical rail position."""
    pixel_span = CALIB_RIGHT_PX - CALIB_LEFT_PX
    if pixel_span == 0:
        return 0.0
    ratio = (pixel_x - CALIB_LEFT_PX) / pixel_span
    return CALIB_LEFT_CM + ratio * (CALIB_RIGHT_CM - CALIB_LEFT_CM)


def display_x(image_x):                                 #将图像X坐标转换为显示X坐标
    return int(image_x * DISPLAY_SIZE[0] / IMAGE_SIZE[0] + 0.5)


def display_y(image_y):
    return int(image_y * DISPLAY_SIZE[1] / IMAGE_SIZE[1] + 0.5)


def image_x(display_coordinate_x):                      #将显示X坐标转换为图像X坐标
    return int(
        display_coordinate_x * IMAGE_SIZE[0] / DISPLAY_SIZE[0] + 0.5
    )


def image_y(display_coordinate_y):
    return int(
        display_coordinate_y * IMAGE_SIZE[1] / DISPLAY_SIZE[1] + 0.5
    )


def send_ball_packet(uart, valid, x, y):                #发送球数据包
    """AA 55 valid signedX_L signedX_H yL yH checksum."""
    if not valid:
        x = 0
        y = 0

    x_word = clamp(int(x), -32768, 32767) & 0xFFFF
    y_word = clamp(int(y), 0, 65535)
    packet = UART_PACKET
    packet[0] = 0xAA
    packet[1] = 0x55
    packet[2] = 1 if valid else 0
    packet[3] = x_word & 0xFF
    packet[4] = (x_word >> 8) & 0xFF
    packet[5] = y_word & 0xFF
    packet[6] = (y_word >> 8) & 0xFF
    packet[7] = sum(packet[0:7]) & 0xFF
    uart.write(packet)


def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    return UART(UART.UART2, UART_BAUD)


def select_ball(detections, last_center):           #选择最可能的球体
    """Select the highest-confidence ball consistent with recent motion."""
    if not detections or len(detections[0]) == 0:
        return None

    boxes = detections[0]
    confidences = detections[2]
    roi_x, roi_y, roi_w, roi_h = DETECTION_ROI
    roi_right = roi_x + roi_w
    roi_bottom = roi_y + roi_h
    best = None
    best_score = -1.0

    for index in range(len(boxes)):
        box = boxes[index]
        confidence = float(confidences[index])
        x = int(box[0])
        y = int(box[1])
        w = int(box[2])
        h = int(box[3])
        cx = x + w // 2
        cy = y + h // 2

        candidate_image_x = image_x(cx)
        candidate_image_y = image_y(cy)

        # YOLO returns display coordinates; filter in 640x480 image coordinates.
        if candidate_image_x < roi_x or candidate_image_x >= roi_right:
            continue
        if candidate_image_y < roi_y or candidate_image_y >= roi_bottom:
            continue

        if last_center is not None:
            dx = cx - last_center[0]
            dy = cy - last_center[1]
            distance2 = dx * dx + dy * dy
            if distance2 > MAX_CENTER_JUMP * MAX_CENTER_JUMP:
                continue
            score = confidence / (1.0 + distance2 * 0.0005)
        else:
            score = confidence

        if score > best_score:
            best_score = score
            best = (x, y, w, h, cx, cy, confidence)

    return best


def main():
    pipeline = None
    detector = None
    uart = None

    last_center = None
    held_detection = None
    lost_frames = 0
    filtered_x = 0.0
    filter_ready = False
    last_send_ms = 0
    frame_count = 0

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)

        # LCKFB K230 camera used by the training capture script is sensor id 2.
        sensor = Sensor(
            id=CAMERA_ID,
            width=1280,
            height=960,
            fps=90
        )
        pipeline = PipeLine(
            rgb888p_size=IMAGE_SIZE,
            display_size=DISPLAY_SIZE,
            display_mode="lcd"
        )
        pipeline.create(sensor=sensor)

        detector = YOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=IMAGE_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=DISPLAY_SIZE,
            conf_thresh=CONF_THRESHOLD,
            nms_thresh=NMS_THRESHOLD
        )
        detector.config_preprocess()
        uart = init_uart()
        clock = time.clock()

        print("YOLO ball hybrid tracker started")
        print("model:", KMODEL_PATH)

        while True:
            os.exitpoint()
            clock.tick()
            frame = pipeline.get_frame()
            detections = detector.run(frame)
            detection = select_ball(detections, last_center)

            if detection is not None:
                held_detection = detection
                lost_frames = 0
            else:
                lost_frames += 1
                if lost_frames > LOST_HOLD_FRAMES:
                    held_detection = None
                    last_center = None
                    filter_ready = False

            valid = held_detection is not None
            centered_x = 0
            center_y = 0
            position_cm = 0.0

            pipeline.osd_img.clear()

            # Draw image center even when the ball is lost.
            image_center_x = DISPLAY_SIZE[0] // 2
            image_center_y = DISPLAY_SIZE[1] // 2
            roi_x, roi_y, roi_w, roi_h = DETECTION_ROI
            draw_roi_x = display_x(roi_x)
            draw_roi_y = display_y(roi_y)
            draw_roi_w = display_x(roi_x + roi_w) - draw_roi_x
            draw_roi_h = display_y(roi_y + roi_h) - draw_roi_y
            pipeline.osd_img.draw_rectangle(
                draw_roi_x, draw_roi_y, draw_roi_w, draw_roi_h,
                color=(150, 150, 150), thickness=1
            )
            pipeline.osd_img.draw_cross(
                image_center_x, image_center_y,
                size=5, color=(0, 0, 0), thickness=1
            )

            if valid:
                x, y, w, h, cx, cy, confidence = held_detection
                if not filter_ready:
                    filtered_x = float(cx)
                    filter_ready = True
                else:
                    filtered_x += POSITION_FILTER_ALPHA * (
                        cx - filtered_x
                    )

                filtered_cx = int(filtered_x + 0.5)
                last_center = (filtered_cx, cy)

                # Detection coordinates are already in the 800x480 OSD space.
                draw_x = x
                draw_y = y
                draw_w = w
                draw_h = h
                draw_cx = filtered_cx
                draw_cy = cy

                ball_image_x = image_x(filtered_cx)
                ball_image_y = image_y(cy)
                centered_x = pixel_x_to_centered_x(ball_image_x)
                center_y = ball_image_y
                position_cm = pixel_x_to_cm(ball_image_x)

                pipeline.osd_img.draw_rectangle(
                    draw_x, draw_y, draw_w, draw_h,
                    color=(255, 255, 255), thickness=2
                )
                pipeline.osd_img.draw_cross(
                    draw_cx, draw_cy,
                    size=8, color=(255, 255, 255), thickness=2
                )
                pipeline.osd_img.draw_line(
                    image_center_x, image_center_y,
                    draw_cx, image_center_y,
                    color=(0, 0, 0), thickness=1
                )
                pipeline.osd_img.draw_line(
                    draw_cx, image_center_y,
                    draw_cx, draw_cy,
                    color=(0, 0, 0), thickness=1
                )
                status = "BALL X=%+d  POS=%+.2fcm  C=%.2f" % (
                    centered_x, position_cm, confidence
                )
            else:
                status = "BALL LOST"

            pipeline.osd_img.draw_string_advanced(
                8, DISPLAY_SIZE[1] - 55, 22,
                status, color=(255, 255, 255)
            )
            pipeline.osd_img.draw_string_advanced(
                8, DISPLAY_SIZE[1] - 28, 18,
                "FPS=%.1f" % clock.fps(),
                color=(255, 255, 255)
            )
            pipeline.show_image()

            now_ms = time.ticks_ms()
            if time.ticks_diff(
                    now_ms, last_send_ms) >= UART_SEND_INTERVAL_MS:
                send_ball_packet(
                    uart, valid, centered_x, center_y
                )
                last_send_ms = now_ms

            frame_count += 1
            if frame_count % PRINT_INTERVAL == 0:
                if valid:
                    print("x=%+d pos=%+.2fcm y=%d fps=%.1f" % (
                        centered_x, position_cm, center_y, clock.fps()
                    ))
                else:
                    print("BALL LOST fps=%.1f" % clock.fps())

            if frame_count % 30 == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("user stopped")
    except BaseException as error:
        sys.print_exception(error)
        raise
    finally:
        if detector is not None:
            detector.deinit()
        if pipeline is not None:
            pipeline.destroy()
        if uart is not None:
            uart.deinit()
        gc.collect()


main()


