import time
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 240

# 横向轨道区域：需要上下移动时只调整 ROI_Y
ROI_X = 0
ROI_Y = 96
ROI_W = IMAGE_WIDTH
ROI_H = 48

DISPLAY_INTERVAL = 2
PRINT_INTERVAL = 5

# 金属球高光候选
BRIGHT_THRESHOLD = (195, 255)
MIN_BRIGHT_PIXELS = 1
MAX_BRIGHT_PIXELS = 500
MIN_BRIGHT_AREA = 1
MAX_BRIGHT_AREA = 900
MIN_BRIGHT_WIDTH = 1
MAX_BRIGHT_WIDTH = 45
MIN_BRIGHT_HEIGHT = 1
MAX_BRIGHT_HEIGHT = 30

# 在高光周围寻找钢球的暗反射
DARK_THRESHOLD = 135
SEARCH_RADIUS_X = 13
SEARCH_RADIUS_Y = 13
SAMPLE_STEP = 3
MIN_DARK_SAMPLES = 2
MIN_BRIGHT_SAMPLES = 1
MIN_LOCAL_RANGE = 45

# 球体候选形状范围，同时兼容高速拖影和远处小球
MIN_ASPECT_RATIO = 0.18
MAX_ASPECT_RATIO = 5.50
MIN_DENSITY = 0.04

# 高速跟踪参数
CONFIRM_FRAMES = 1
LOST_FRAMES = 7
MAX_PREDICT_FRAMES = 6
SMOOTH_ALPHA = 0.92
VELOCITY_ALPHA = 0.72
VELOCITY_DECAY = 0.88
TRACK_BASE_DISTANCE = 35
TRACK_SPEED_GAIN = 2.8
MAX_TRACK_DISTANCE = 220


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


sensor = Sensor()
sensor.reset()
sensor.set_pixformat(Sensor.GRAYSCALE)
sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
sensor.set_hmirror(False)
sensor.set_vflip(False)

Display.init(
    Display.VIRT,
    width=IMAGE_WIDTH,
    height=IMAGE_HEIGHT,
    to_ide=True
)

MediaManager.init()
sensor.run()

clock = time.clock()
frame_count = 0
found_frames = 0
lost_frames = 0
predict_frames = 0
ball_found = False

smooth_x = 0
smooth_y = 0
smooth_w = 0
smooth_h = 0
velocity_x = 0
velocity_y = 0

try:
    while True:
        clock.tick()
        frame_count += 1
        img = sensor.snapshot()

        bright_blobs = img.find_blobs(
            [BRIGHT_THRESHOLD],
            roi=(ROI_X, ROI_Y, ROI_W, ROI_H),
            pixels_threshold=MIN_BRIGHT_PIXELS,
            area_threshold=MIN_BRIGHT_AREA,
            merge=True,
            margin=2
        )

        best_candidate = None
        best_score = -1

        for blob in bright_blobs:
            w = blob.w()
            h = blob.h()
            pixels = blob.pixels()
            area = w * h

            if area == 0 or h == 0:
                continue
            if pixels > MAX_BRIGHT_PIXELS or area > MAX_BRIGHT_AREA:
                continue
            if w < MIN_BRIGHT_WIDTH or w > MAX_BRIGHT_WIDTH:
                continue
            if h < MIN_BRIGHT_HEIGHT or h > MAX_BRIGHT_HEIGHT:
                continue

            aspect_ratio = w / h
            density = pixels / area

            if not MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO:
                continue
            if density < MIN_DENSITY:
                continue

            center_x = blob.cx()
            center_y = blob.cy()
            left = clamp(center_x - SEARCH_RADIUS_X, ROI_X,
                         ROI_X + ROI_W - 1)
            right = clamp(center_x + SEARCH_RADIUS_X, ROI_X,
                          ROI_X + ROI_W - 1)
            top = clamp(center_y - SEARCH_RADIUS_Y, ROI_Y,
                        ROI_Y + ROI_H - 1)
            bottom = clamp(center_y + SEARCH_RADIUS_Y, ROI_Y,
                           ROI_Y + ROI_H - 1)

            local_min = 255
            local_max = 0
            dark_count = 0
            bright_count = 0
            dark_sum_x = 0
            dark_sum_y = 0

            sample_y = top
            while sample_y <= bottom:
                sample_x = left
                while sample_x <= right:
                    value = img.get_pixel(sample_x, sample_y)

                    if value < local_min:
                        local_min = value
                    if value > local_max:
                        local_max = value

                    if value <= DARK_THRESHOLD:
                        dark_count += 1
                        dark_sum_x += sample_x
                        dark_sum_y += sample_y
                    if value >= BRIGHT_THRESHOLD[0]:
                        bright_count += 1

                    sample_x += SAMPLE_STEP
                sample_y += SAMPLE_STEP

            if dark_count < MIN_DARK_SAMPLES:
                continue
            if bright_count < MIN_BRIGHT_SAMPLES:
                continue
            if local_max - local_min < MIN_LOCAL_RANGE:
                continue

            # 高光点与暗反射中心的中点更接近金属球中心
            dark_center_x = dark_sum_x // dark_count
            dark_center_y = dark_sum_y // dark_count
            candidate_x = (center_x + dark_center_x) // 2
            candidate_y = (center_y + dark_center_y) // 2
            candidate_w = max(w, min(SEARCH_RADIUS_X * 2,
                                     abs(center_x - dark_center_x) * 2 + w))
            candidate_h = max(h, min(SEARCH_RADIUS_Y * 2,
                                     abs(center_y - dark_center_y) * 2 + h))

            shape_score = min(w, h) / max(w, h)
            contrast_score = local_max - local_min
            score = contrast_score + pixels * 2 + dark_count * 3
            score *= 0.5 + shape_score

            if ball_found:
                predict_x = smooth_x + velocity_x
                predict_y = smooth_y + velocity_y
                dx = candidate_x - predict_x
                dy = candidate_y - predict_y
                distance_squared = dx * dx + dy * dy
                speed = abs(velocity_x) + abs(velocity_y)
                track_distance = int(
                    TRACK_BASE_DISTANCE + speed * TRACK_SPEED_GAIN
                )
                track_distance = clamp(
                    track_distance,
                    TRACK_BASE_DISTANCE,
                    MAX_TRACK_DISTANCE
                )

                if distance_squared > track_distance * track_distance:
                    continue

                score = score * 4.0 / (
                    1.0 + distance_squared * 0.001
                )

            if score > best_score:
                best_score = score
                best_candidate = (
                    candidate_x,
                    candidate_y,
                    candidate_w,
                    candidate_h
                )

        if best_candidate is not None:
            current_x = best_candidate[0]
            current_y = best_candidate[1]
            current_w = best_candidate[2]
            current_h = best_candidate[3]

            found_frames += 1
            lost_frames = 0
            predict_frames = 0

            if found_frames >= CONFIRM_FRAMES:
                if not ball_found:
                    smooth_x = current_x
                    smooth_y = current_y
                    smooth_w = current_w
                    smooth_h = current_h
                    velocity_x = 0
                    velocity_y = 0
                else:
                    measured_vx = current_x - smooth_x
                    measured_vy = current_y - smooth_y
                    velocity_x = int(
                        velocity_x * (1.0 - VELOCITY_ALPHA)
                        + measured_vx * VELOCITY_ALPHA
                    )
                    velocity_y = int(
                        velocity_y * (1.0 - VELOCITY_ALPHA)
                        + measured_vy * VELOCITY_ALPHA
                    )
                    smooth_x = int(
                        smooth_x * (1.0 - SMOOTH_ALPHA)
                        + current_x * SMOOTH_ALPHA
                    )
                    smooth_y = int(
                        smooth_y * (1.0 - SMOOTH_ALPHA)
                        + current_y * SMOOTH_ALPHA
                    )
                    smooth_w = int(
                        smooth_w * (1.0 - SMOOTH_ALPHA)
                        + current_w * SMOOTH_ALPHA
                    )
                    smooth_h = int(
                        smooth_h * (1.0 - SMOOTH_ALPHA)
                        + current_h * SMOOTH_ALPHA
                    )

                ball_found = True
        else:
            found_frames = 0
            lost_frames += 1

            if ball_found and predict_frames < MAX_PREDICT_FRAMES:
                smooth_x = clamp(
                    smooth_x + velocity_x,
                    ROI_X,
                    ROI_X + ROI_W - 1
                )
                smooth_y = clamp(
                    smooth_y + velocity_y,
                    ROI_Y,
                    ROI_Y + ROI_H - 1
                )
                velocity_x = int(velocity_x * VELOCITY_DECAY)
                velocity_y = int(velocity_y * VELOCITY_DECAY)
                predict_frames += 1

            if lost_frames >= LOST_FRAMES:
                ball_found = False
                smooth_x = 0
                smooth_y = 0
                smooth_w = 0
                smooth_h = 0
                velocity_x = 0
                velocity_y = 0
                predict_frames = 0

        img.draw_rectangle(
            ROI_X,
            ROI_Y,
            ROI_W,
            ROI_H,
            color=150,
            thickness=1
        )

        if ball_found:
            box_x = smooth_x - smooth_w // 2
            box_y = smooth_y - smooth_h // 2
            img.draw_rectangle(
                box_x,
                box_y,
                smooth_w,
                smooth_h,
                color=255,
                thickness=2
            )
            img.draw_cross(
                smooth_x,
                smooth_y,
                color=255,
                size=8,
                thickness=2
            )
            img.draw_string(5, 5, "BALL: YES", color=255, scale=2)
            img.draw_string(
                5,
                30,
                "X:%d Y:%d" % (smooth_x, smooth_y),
                color=255,
                scale=2
            )

            if frame_count % PRINT_INTERVAL == 0:
                print(
                    "BALL_FOUND, x=%d, y=%d, vx=%d, vy=%d" %
                    (smooth_x, smooth_y, velocity_x, velocity_y)
                )
        else:
            img.draw_string(5, 5, "BALL: NO", color=255, scale=2)
            if frame_count % PRINT_INTERVAL == 0:
                print("NO_BALL")

        img.draw_string(
            5,
            IMAGE_HEIGHT - 25,
            "FPS:%.1f" % clock.fps(),
            color=255,
            scale=2
        )

        if frame_count % DISPLAY_INTERVAL == 0:
            Display.show_image(img)

finally:
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
