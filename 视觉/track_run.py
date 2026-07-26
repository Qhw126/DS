import time, gc
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

sensor = Sensor(id=2)
sensor.reset()
sensor.set_framesize(width=320, height=240)
sensor.set_pixformat(Sensor.RGB565)
Display.init(Display.ST7701, width=800, height=480, to_ide=True, quality=30)
MediaManager.init()
sensor.run()
time.sleep_ms(300)

TH = (21, 49, 9, 68, -2, 51)

clock = time.clock()
while True:
    clock.tick()
    try:
        img = sensor.snapshot()
    except Exception:
        gc.collect()
        continue

    # 二值化 → 转灰度（画面还是黑白，格式变灰度）
    img = img.binary([TH]).to_grayscale()

    # 找白线（[(255,255)] 对灰度图有效）
    line = img.get_regression([(255, 255)])
    off = ((line.x1() + line.x2()) / 2 - 160) / 160 if line else 0.0

    img.draw_string_advanced(10, 10, 28, "OFF:%+.3f" % off, color=255)
    Display.show_image(img, x=240, y=120)
    gc.collect()
