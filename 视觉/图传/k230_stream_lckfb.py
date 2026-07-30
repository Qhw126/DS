# K230 MJPEG WiFi streamer for LCKFB/CanMV K230
import gc
import time
import network
import socket

from media.sensor import Sensor, CAM_CHN_ID_0
from media.display import Display
from media.media import MediaManager


WIFI_SSID = "elysia vivo X200s"
WIFI_PASS = "20060719"
STREAM_PORT = 8080

SENSOR_ID = 2
STREAM_WIDTH = 320
STREAM_HEIGHT = 240
JPEG_QUALITY = 60
FRAME_DELAY_MS = 80

CRLF = b"\r\n"
BOUNDARY = b"--frame"


def is_eagain(exc):
    return (exc.args and exc.args[0] == 11) or getattr(exc, "errno", None) == 11


def send_all(conn, data):
    data = memoryview(data)
    sent_total = 0
    while sent_total < len(data):
        try:
            sent = conn.send(data[sent_total:])
        except OSError as exc:
            if is_eagain(exc):
                time.sleep_ms(10)
                continue
            raise
        if sent == 0:
            raise OSError("socket closed")
        sent_total += sent


def jpeg_bytes(img):
    jpg = img.compress(quality=JPEG_QUALITY)
    for method_name in ("bytearray", "to_bytes"):
        try:
            return getattr(jpg, method_name)()
        except Exception:
            pass
    try:
        return bytes(jpg)
    except Exception:
        return jpg


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    print("Connecting WiFi:", WIFI_SSID)
    start = time.time()
    last_retry = 0

    while True:
        ip = wlan.ifconfig()[0]
        if wlan.isconnected() and ip != "0.0.0.0":
            break

        now = time.time()
        if now - start > 60:
            print("Last ifconfig:", wlan.ifconfig())
            raise RuntimeError("WiFi timeout")

        if now - last_retry >= 3:
            last_retry = now
            try:
                wlan.connect(WIFI_SSID, WIFI_PASS)
            except Exception as exc:
                print("connect retry failed:", exc)
            print("Waiting WiFi... status:", wlan.status(), "ifconfig:", wlan.ifconfig())

        time.sleep(1)

    while wlan.ifconfig()[0] == "0.0.0.0":
        time.sleep_ms(100)

    ip = wlan.ifconfig()[0]
    print("WiFi connected")
    print("IP:", ip)
    print("PC/OpenCV URL: http://{}:{}".format(ip, STREAM_PORT))
    print("Browser URL: http://{}:{}/".format(ip, STREAM_PORT))
    return wlan


def init_camera():
    sensor = Sensor(id=SENSOR_ID, width=STREAM_WIDTH, height=STREAM_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=STREAM_WIDTH, height=STREAM_HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    Display.init(Display.VIRT, width=STREAM_WIDTH, height=STREAM_HEIGHT, fps=15)
    MediaManager.init()
    sensor.run()
    print("Camera ready:", STREAM_WIDTH, "x", STREAM_HEIGHT)
    return sensor


def handle_client(conn, sensor):
    print("Client connected")
    send_all(conn, b"HTTP/1.1 200 OK" + CRLF)
    send_all(conn, b"Content-Type: multipart/x-mixed-replace; boundary=frame" + CRLF)
    send_all(conn, b"Cache-Control: no-cache" + CRLF)
    send_all(conn, b"Connection: close" + CRLF + CRLF)

    frame_count = 0
    while True:
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        jpg = jpeg_bytes(img)
        send_all(conn, BOUNDARY + CRLF)
        send_all(conn, b"Content-Type: image/jpeg" + CRLF)
        send_all(conn, b"Content-Length: %d" % len(jpg) + CRLF + CRLF)
        send_all(conn, jpg)
        send_all(conn, CRLF)

        frame_count += 1
        if frame_count % 60 == 0:
            print("stream frames:", frame_count, "last bytes:", len(jpg))
            gc.collect()
        time.sleep_ms(FRAME_DELAY_MS)


def start_server(sensor):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", STREAM_PORT))
    server.listen(1)
    print("HTTP server listening on port", STREAM_PORT)

    while True:
        try:
            conn, addr = server.accept()
        except OSError as exc:
            if is_eagain(exc):
                time.sleep_ms(20)
                continue
            raise

        print("Client:", addr)
        try:
            handle_client(conn, sensor)
        except Exception as exc:
            print("Client disconnected:", exc)
        finally:
            try:
                conn.close()
            except Exception:
                pass


sensor = None
camera_ready = False

try:
    connect_wifi()
    sensor = init_camera()
    camera_ready = True
    start_server(sensor)
finally:
    if sensor and camera_ready:
        sensor.stop()
        Display.deinit()
        MediaManager.deinit()
