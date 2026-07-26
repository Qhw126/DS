import time, gc
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

sensor = Sensor(id=2)
sensor.reset()
sensor.set_framesize(width=320, height=240)
sensor.set_pixformat(Sensor.RGB565)
Display.init(Display.ST7701, width=800, height=480, to_ide=True, quality=50)
MediaManager.init()
sensor.run()
time.sleep_ms(300)

# LAB阈值 (Lmin,Lmax, Amin,Amax, Bmin,Bmax) - 红色
TH = (23, 43, -19, 49, -7, 36)

def restart_sensor():
    print("重启传感器...")
    try:
        sensor.stop()
    except Exception:
        pass
    time.sleep_ms(100)
    sensor.run()
    time.sleep_ms(300)
    print("传感器重启完成")

clock = time.clock()
while True:
    clock.tick()
    try:
        img = sensor.snapshot()
    except Exception as e:
        print("采图失败:", e)
        restart_sensor()
        gc.collect()
        continue

    # 在彩色图上找红色色块
    blobs = img.find_blobs([TH], pixels_threshold=30, merge=True)

    off = 0.0
    if blobs:
        cx, tw = 0, 0
        for b in blobs:
            if b.cy() > 80:
                cx += b.cx() * b.pixels()
                tw += b.pixels()
        if tw > 50:
            off = (cx / tw - 160) / 160

    # 二值化显示（重要：赋值回来！）
    img = img.binary([TH])

    # OSD
    img.draw_string_advanced(10, 10, 28, "OFF:%+.3f" % off, color=(255, 255, 255))

    Display.show_image(img, x=240, y=120)
    gc.collect()
