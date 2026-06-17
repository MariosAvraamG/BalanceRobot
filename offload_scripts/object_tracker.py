import time
import argparse
import threading
from dataclasses import dataclass
import cv2
import numpy as np
import requests
from ultralytics import YOLO
from pynput import keyboard as pynput_kb
from flask import Flask, Response, request, jsonify, redirect, url_for
from PIL import Image
import io


app = Flask(__name__)
frame_lock = threading.Lock()
latest_frame = None
JPEG_QUALITY    = 80


PORT = 5000

DEFAULT_IP = "youssef.local"

MAX_ANGULAR_VEL = 0.5
MAX_LINEAR_VEL  = 4
CENTER_THRESHOLD = 0.40 
SEARCH_VEL       = 0.15 
SEARCH_TURN_SECS = 0.35
PERSIST_FRAMES   = 4
CMD_RATE_HZ      = 10

TRACKABLE_CLASSES = {
    "person":    0,
    "bottle":    39,
    "car":       2,
    "plant":     58
}

INVERTED_CLASS_MAP = {
    0: "person",
    39: "bottle",
    2: "car",
    58: "plant"
}

KEY_CLASS_MAP = {
    "1": 39, #bottle
    "2": 2, #car
    "3": 58, #plant
    "4": 0 #person
}

model = YOLO("yolov8n.pt")

_pending = {"change" : None}
_pending_lock = threading.Lock()

def _on_press(key):
    try:
        char = key.char
        if char in KEY_CLASS_MAP:
            cid = KEY_CLASS_MAP[char]
            with _pending_lock:
                _pending["change"] = cid
                print(f"Key {char} → switching to {cid}")
    except AttributeError:
        pass

@dataclass
class MotorCommand:
    linear_vel: float
    angular_vel: float


def compute_motor_command(box_xyxy, frame_width: int, frame_height: int) -> MotorCommand:
    x1, y1, x2, y2 = box_xyxy
    offset = ((x1 + x2) / 2 - frame_width / 2) / (frame_width / 2)

    if abs(offset) > CENTER_THRESHOLD:
        return MotorCommand(linear_vel=0.0,
                            angular_vel=offset * MAX_ANGULAR_VEL)
    else:
        box_height_ratio = (y2 - y1) / frame_height
        linear_vel = MAX_LINEAR_VEL * (1.0 - min(1.0, box_height_ratio))
        return MotorCommand(linear_vel=linear_vel, angular_vel=offset * MAX_ANGULAR_VEL)


class ObjectTracker:
    def __init__(self, class_id: int, min_conf: float = 0.5):
        self.class_id      = class_id
        self.min_conf      = min_conf
        self.acquiring_box = None          
        self.last_xyxy     = None          
        self._locked_centroid  = None
        self._acquire_centroid = None
        self._acquire_count    = 0
        self._lost             = 0

    MAX_LOST_FRAMES = 15
    GHOST_FRAMES = 8
    def update(self, boxes) -> object | None:
        candidates = [
            b for b in boxes
            if int(b.cls[0]) == self.class_id and float(b.conf[0]) >= self.min_conf
        ]

        if not candidates:
            self._lost += 1
            if self._lost >= self.MAX_LOST_FRAMES:
                self._reset()
            self.acquiring_box = None
            return None

        self._lost = 0

        ref = self._locked_centroid or self._acquire_centroid
        if ref is not None:
            match = min(candidates, key=lambda b: self._dist(b, ref))
        else:
            match = max(candidates, key=lambda b: float(b.conf[0]))

        cx, cy = self._cx_cy(match)

        if self._locked_centroid is not None:
            self._locked_centroid = (cx, cy)
            self.acquiring_box = None
            self.last_xyxy = [float(v) for v in match.xyxy[0]]
            return match

        self._acquire_centroid = (cx, cy)
        self._acquire_count   += 1
        self.acquiring_box     = match

        if self._acquire_count >= PERSIST_FRAMES:
            self._locked_centroid  = (cx, cy)
            self._acquire_centroid = None
            self._acquire_count    = 0
            self.acquiring_box     = None
            return match

        return None

    @property
    def ghost_xyxy(self):
        if self.last_xyxy is not None and self._lost <= self.GHOST_FRAMES:
            return self.last_xyxy
        return None

    def _reset(self):
        self._locked_centroid  = None
        self._acquire_centroid = None
        self._acquire_count    = 0
        self.last_xyxy         = None

    @staticmethod
    def _cx_cy(box):
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
        return (x1 + x2) / 2, (y1 + y2) / 2

    @staticmethod
    def _dist(box, centroid):
        cx, cy = ObjectTracker._cx_cy(box)
        return (cx - centroid[0]) ** 2 + (cy - centroid[1]) ** 2


class SearchSweep:
    def command(self, last_seen_side: int = 1) -> MotorCommand:
        return MotorCommand(linear_vel=0.0, angular_vel=last_seen_side * SEARCH_VEL)


def display_stream(pi_ip: str, track_class_id: int):
    url = f"http://{pi_ip}:{PORT}/stream"
    try:
        resp = requests.get(url, stream=True, timeout=10)
    except requests.exceptions.ConnectionError:
        print(f"ERROR: Could not connect to {url}")
        return

    if resp.status_code != 200:
        print(f"ERROR: Server returned {resp.status_code}")
        return

    tracker        = ObjectTracker(track_class_id)
    search         = SearchSweep()
    last_seen_side = 1 
    last_cmd_time  = 0.0
    cmd_interval  = 1.0 / CMD_RATE_HZ
    buf           = b""

    try:
      for chunk in resp.iter_content(chunk_size=4096):
        buf += chunk

        with _pending_lock:
            change_cid = _pending["change"]
            if change_cid is not None:
                _pending["change"] = None
        if change_cid is not None:
            track_class_id = change_cid
            tracker = ObjectTracker(change_cid)

        while True:
            boundary = buf.find(b"--frame\r\n")
            next_boundary = buf.find(b"--frame\r\n", boundary + 9)
            if boundary == -1 or next_boundary == -1:
                break
            segment = buf[boundary:next_boundary]
            buf = buf[next_boundary:]

            capture_time = None
            ts_marker = b"X-Capture-Time: "
            ts_pos = segment.find(ts_marker)
            if ts_pos != -1:
                ts_end = segment.find(b"\r\n", ts_pos)
                try:
                    capture_time = float(segment[ts_pos + len(ts_marker):ts_end])
                except ValueError:
                    pass

            jpg_start = segment.find(b"\xff\xd8")
            jpg_end   = segment.rfind(b"\xff\xd9")
            if jpg_start == -1 or jpg_end == -1:
                continue
            jpg = segment[jpg_start:jpg_end + 2]

            decode_time = time.time()

            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            frame_h, frame_w = frame.shape[:2]

            results = model(frame, verbose=False)

            for box in results[0].boxes:
                if int(box.cls[0]) != track_class_id or float(box.conf[0]) < 0.5:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = f"{model.names[int(box.cls[0])]} {box.conf[0]:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            target = tracker.update(results[0].boxes)

            if target is not None:
                xyxy = [float(v) for v in target.xyxy[0]]
                cmd  = compute_motor_command(xyxy, frame_w, frame_h)
                x1, y1, x2, y2 = map(int, target.xyxy[0])
                cx_obj, cy_obj = (x1 + x2) // 2, (y1 + y2) // 2
                last_seen_side = -1 if cx_obj < frame_w / 2 else 1
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 3)
                cv2.circle(frame, (cx_obj, cy_obj), 6,  (255, 100, 0), -1)
                cv2.circle(frame, (cx_obj, cy_obj), 10, (255, 255, 255), 2)
            elif tracker.acquiring_box is not None:
                x1, y1, x2, y2 = map(int, tracker.acquiring_box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)
                cv2.putText(frame, "acquiring...", (x1, y2 + 16),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
                cmd = MotorCommand(linear_vel=0.0, angular_vel=0.0)
            else:
                if tracker.ghost_xyxy is not None:
                    cmd = compute_motor_command(tracker.ghost_xyxy, frame_w, frame_h)
                    gx1, gy1, gx2, gy2 = map(int, tracker.ghost_xyxy)
                    cv2.rectangle(frame, (gx1, gy1), (gx2, gy2), (120, 120, 120), 2)
                    cv2.putText(frame, "lost", (gx1, gy1 - 8),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1)
                else:
                    cmd = search.command(last_seen_side)

            now_cmd = time.time()
            if now_cmd - last_cmd_time >= cmd_interval:
                last_cmd_time = now_cmd
                print(cmd)
                def _send(lv=cmd.linear_vel, av=cmd.angular_vel):
                    t0 = time.time()
                    try:
                        r = requests.post(
                            f"http://{pi_ip}:{PORT}/command",
                            json={"linear_vel": lv, "angular_vel": av, "tracked_object": INVERTED_CLASS_MAP.get(tracker.class_id, "N/A")},
                            timeout=0.5,
                        )
                    except requests.exceptions.RequestException as e:
                        print(f"CMD send failed: {e}")
                threading.Thread(target=_send, daemon=True).start()

            text = f"lin: {cmd.linear_vel:+.2f}  ang: {cmd.angular_vel:+.2f}"
            cv2.putText(frame, text, (8, 24),cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)

            jpeg_buf = io.BytesIO()
            Image.fromarray(frame[:, :, ::-1]).save(jpeg_buf, format="JPEG", quality=JPEG_QUALITY)
            global latest_frame
            with frame_lock:
                latest_frame = jpeg_buf.getvalue()

            cv2.imshow("Pi Camera Stream - YOLOv8n", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except requests.exceptions.ConnectionError:
        print("Stream connection lost.")
    finally:
        cv2.destroyAllWindows()

def generate_mjpeg():
    global latest_frame
    while True:
        with frame_lock:
            jpeg = latest_frame
        if jpeg:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + jpeg + b"\r\n"
            )
        time.sleep(0.05)

@app.route("/stream")
def stream():
    return Response(
        generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pi camera MJPEG viewer with object tracking")
    parser.add_argument("ip", nargs="?", default=DEFAULT_IP, help="Pi IP address")
    parser.add_argument(
        "--track",
        choices=TRACKABLE_CLASSES.keys(),
        default="person",
        help="Object class to track (default: person)",
    )
    args = parser.parse_args()
    class_id = TRACKABLE_CLASSES[args.track]
    kb_listener = pynput_kb.Listener(on_press=_on_press)
    kb_listener.daemon = True
    kb_listener.start()

    server_thread = threading.Thread(target=lambda: app.run(host="0.0.0.0", port=8000), daemon=True)
    print(f"Server running on http://0.0.0.0:{8000}")
    server_thread.start()

    display_stream(args.ip, class_id)
