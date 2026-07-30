# PC receiver for K230 MJPEG stream.
# Install dependency once: python -m pip install opencv-python
import cv2
import datetime
import os
import sys
import time


K230_IP = sys.argv[1] if len(sys.argv) > 1 else "10.172.230.91"
STREAM_URL = "http://{}:8080".format(K230_IP)
RETRY_SECONDS = 60
RECORD_FPS = 12.0


def open_stream():
    print("Connecting", STREAM_URL, "...")
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        return None
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    return cap


deadline = time.time() + RETRY_SECONDS
cap = None
while time.time() < deadline:
    cap = open_stream()
    if cap is not None:
        break
    print("Failed. Retrying...")
    time.sleep(2)

if cap is None:
    print("Failed. Check K230 IP, WiFi, and whether the K230 script is running.")
    raise SystemExit(1)

print("Connected. Press q to quit, r to replay current recording.")

ts = datetime.datetime.now().strftime("%m%d_%H%M%S")
filename = "record_{}.avi".format(ts)
writer = None
frame_count = 0

while True:
    ok, frame = cap.read()
    if not ok:
        print("Reconnecting...")
        cap.release()
        time.sleep(1)
        cap = open_stream()
        continue

    if writer is None:
        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        writer = cv2.VideoWriter(filename, fourcc, RECORD_FPS, (w, h))
        print("Recording:", filename, "size:", w, "x", h, "fps:", RECORD_FPS)

    cv2.imshow("K230 Ball Monitor - q quit, r replay", frame)
    writer.write(frame)
    frame_count += 1

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    if key == ord("r") and os.path.exists(filename):
        os.startfile(filename)

cap.release()
if writer is not None:
    writer.release()
cv2.destroyAllWindows()
print("Saved:", filename, "frames:", frame_count)
