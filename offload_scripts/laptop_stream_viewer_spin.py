#!/usr/bin/env python3
"""
Laptop MJPEG stream viewer for the Raspberry Pi camera server.
Connects to the Pi's HTTP stream, runs YOLOv8n inference on every frame, and displays detections.
Tracks a single object and issues either angular or linear velocity commands (never both).
When no target is found the robot performs a step-and-wait search sweep.

Install dependencies:
    pip3 install opencv-python requests numpy ultralytics

Usage:
    python3 laptop_stream_viewer.py <pi_ip_address> [--track {person,keyboard,cellphone}]

Example:
    python3 laptop_stream_viewer.py 192.168.1.42 --track person
"""

import time
import argparse
import threading
from dataclasses import dataclass
import cv2
import numpy as np
import requests
from ultralytics import YOLO

PORT = 5000
DEFAULT_IP = "youssef.local"

MAX_ANGULAR_VEL = 0.4
MAX_LINEAR_VEL  = 3
CENTER_THRESHOLD = 0.30   # fraction of half-width: inside this → linear mode

SEARCH_VEL       = 0.2    # angular speed while searching
SEARCH_TURN_SECS = 0.35   # how long to turn each step
SEARCH_WAIT_SECS = 0.40   # how long to pause between steps

PERSIST_FRAMES   = 4      # consecutive detections required before locking

CMD_RATE_HZ      = 1     # max motor commands issued per second

TRACKABLE_CLASSES = {
    "person":    0,
    "bottle":    39,
    "apple":     47,
    "keyboard":  66,
    "cellphone": 67,
}

model = YOLO("yolov8n.pt")


@dataclass
class MotorCommand:
    linear_vel: float
    angular_vel: float


def compute_motor_command(box_xyxy, frame_width: int, frame_height: int) -> MotorCommand:
    x1, y1, x2, y2 = box_xyxy
    offset = ((x1 + x2) / 2 - frame_width / 2) / (frame_width / 2)  # -1..+1

    if abs(offset) > CENTER_THRESHOLD:
        return MotorCommand(linear_vel=0.0,
                            angular_vel=offset * MAX_ANGULAR_VEL)
    else:
        box_height_ratio = (y2 - y1) / frame_height   # 0=far, 1=close
        linear_vel = MAX_LINEAR_VEL * (1.0 - min(1.0, box_height_ratio))
        return MotorCommand(linear_vel=linear_vel, angular_vel=offset * MAX_ANGULAR_VEL)


class ObjectTracker:
    """
    Two-phase tracker:
      Acquisition – object must appear in PERSIST_FRAMES consecutive frames
                    before the lock is confirmed. During this phase the tracker
                    returns None (search/wait behaviour in the main loop).
      Locked       – centroid-nearest matching; lock drops after MAX_LOST_FRAMES
                    misses and acquisition restarts from scratch.

    Attributes
    ----------
    acquiring_box : the candidate box during acquisition (for display only), or None.
    """

    MAX_LOST_FRAMES = 10

    GHOST_FRAMES = 5  # frames to show last-known box after lock is lost

    def __init__(self, class_id: int, min_conf: float = 0.5):
        self.class_id      = class_id
        self.min_conf      = min_conf
        self.acquiring_box = None          # exposed for display
        self.last_xyxy     = None          # last confirmed locked box coords

        self._locked_centroid  = None
        self._acquire_centroid = None
        self._acquire_count    = 0
        self._lost             = 0

    # ------------------------------------------------------------------
    def update(self, boxes) -> object | None:
        """
        Returns the locked box if we have a confirmed lock, otherwise None.
        Sets self.acquiring_box to the tentative candidate during acquisition.
        """
        candidates = [
            b for b in boxes
            if int(b.cls[0]) == self.class_id and float(b.conf[0]) >= self.min_conf
        ]

        # ── No detections ──────────────────────────────────────────────
        if not candidates:
            self._lost += 1
            if self._lost >= self.MAX_LOST_FRAMES:
                self._reset()
            self.acquiring_box = None
            return None

        self._lost = 0

        # ── Pick best candidate ────────────────────────────────────────
        ref = self._locked_centroid or self._acquire_centroid
        if ref is not None:
            match = min(candidates, key=lambda b: self._dist(b, ref))
        else:
            match = max(candidates, key=lambda b: float(b.conf[0]))

        cx, cy = self._cx_cy(match)

        # ── Already locked ─────────────────────────────────────────────
        if self._locked_centroid is not None:
            self._locked_centroid = (cx, cy)
            self.acquiring_box = None
            self.last_xyxy = [float(v) for v in match.xyxy[0]]
            return match

        # ── Acquisition phase ──────────────────────────────────────────
        self._acquire_centroid = (cx, cy)
        self._acquire_count   += 1
        self.acquiring_box     = match

        if self._acquire_count >= PERSIST_FRAMES:
            self._locked_centroid  = (cx, cy)
            self._acquire_centroid = None
            self._acquire_count    = 0
            self.acquiring_box     = None
            return match

        return None   # not yet confirmed

    # ------------------------------------------------------------------
    @property
    def ghost_xyxy(self):
        """Last known box coords while within the ghost window, else None."""
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
    """Spin continuously in one direction until the target is found again."""

    def command(self) -> MotorCommand:
        return MotorCommand(linear_vel=0.0, angular_vel=SEARCH_VEL)


def display_stream(pi_ip: str, track_class_id: int):
    url = f"http://{pi_ip}:{PORT}/stream"
    print(f"Connecting to {url} ...")

    try:
        resp = requests.get(url, stream=True, timeout=10)
    except requests.exceptions.ConnectionError:
        print(f"ERROR: Could not connect to {url}")
        print("Make sure pi_camera_server.py is running on the Pi.")
        return

    if resp.status_code != 200:
        print(f"ERROR: Server returned {resp.status_code}")
        return

    print("Connected. Press 'q' to quit.")

    tracker       = ObjectTracker(track_class_id)
    search        = SearchSweep()
    last_cmd_time = 0.0
    cmd_interval  = 1.0 / CMD_RATE_HZ
    buf           = b""

    # Shared latency stats updated by the POST thread
    latency = {"frame_ms": 0.0, "inference_ms": 0.0, "post_rtt_ms": 0.0, "serial_ms": 0.0, "fps": 0.0}
    hud_lines   = []
    hud_updated = 0.0
    last_frame_time = None

    try:
      for chunk in resp.iter_content(chunk_size=4096):
        buf += chunk

        while True:
            # Parse MJPEG boundary to extract the capture timestamp header
            boundary = buf.find(b"--frame\r\n")
            next_boundary = buf.find(b"--frame\r\n", boundary + 9)
            if boundary == -1 or next_boundary == -1:
                break

            segment = buf[boundary:next_boundary]
            buf = buf[next_boundary:]

            # Extract X-Capture-Time header
            capture_time = None
            ts_marker = b"X-Capture-Time: "
            ts_pos = segment.find(ts_marker)
            if ts_pos != -1:
                ts_end = segment.find(b"\r\n", ts_pos)
                try:
                    capture_time = float(segment[ts_pos + len(ts_marker):ts_end])
                except ValueError:
                    pass

            # Extract JPEG
            jpg_start = segment.find(b"\xff\xd8")
            jpg_end   = segment.rfind(b"\xff\xd9")
            if jpg_start == -1 or jpg_end == -1:
                continue
            jpg = segment[jpg_start:jpg_end + 2]

            decode_time = time.time()
            if capture_time:
                latency["frame_ms"] = (decode_time - capture_time) * 1000
            if last_frame_time:
                latency["fps"] = 1.0 / (decode_time - last_frame_time)
            last_frame_time = decode_time

            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            frame_h, frame_w = frame.shape[:2]

            t_inf = time.time()
            results = model(frame, verbose=False)
            latency["inference_ms"] = (time.time() - t_inf) * 1000

            # Draw detections of the tracked class only
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
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 3)
                cx_obj, cy_obj = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(frame, (cx_obj, cy_obj), 6,  (255, 100, 0), -1)
                cv2.circle(frame, (cx_obj, cy_obj), 10, (255, 255, 255), 2)
                state_label = "TRACKING"
            elif tracker.acquiring_box is not None:
                x1, y1, x2, y2 = map(int, tracker.acquiring_box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)
                cv2.putText(frame, "acquiring...", (x1, y2 + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
                cmd = MotorCommand(linear_vel=0.0, angular_vel=0.0)
                state_label = "ACQUIRING"
            else:
                cmd = search.command()
                state_label = "SEARCHING"
                if tracker.ghost_xyxy is not None:
                    gx1, gy1, gx2, gy2 = map(int, tracker.ghost_xyxy)
                    cv2.rectangle(frame, (gx1, gy1), (gx2, gy2), (120, 120, 120), 2)
                    cv2.putText(frame, "lost", (gx1, gy1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1)

            # Rate-limited command dispatch
            now_cmd = time.time()
            if now_cmd - last_cmd_time >= cmd_interval:
                last_cmd_time = now_cmd
                print(cmd)
                def _send(lv=cmd.linear_vel, av=cmd.angular_vel):
                    t0 = time.time()
                    try:
                        r = requests.post(
                            f"http://{pi_ip}:{PORT}/command",
                            json={"linear_vel": lv, "angular_vel": av},
                            timeout=0.5,
                        )
                        rtt = (time.time() - t0) * 1000
                        latency["post_rtt_ms"] = rtt
                        data = r.json()
                        latency["serial_ms"] = data.get("serial_write_ms", 0.0)
                    except requests.exceptions.RequestException as e:
                        print(f"CMD send failed: {e}")
                threading.Thread(target=_send, daemon=True).start()

            # # HUD
            # cx_frame = frame_w // 2
            # cv2.line(frame, (cx_frame, 0), (cx_frame, frame_h), (100, 100, 100), 1)

            now_hud = time.time()
            if now_hud - hud_updated >= 1.0:
                hud_updated = now_hud
                total_ms = latency["frame_ms"] + latency["inference_ms"] + latency["post_rtt_ms"] / 2 + latency["serial_ms"]
                hud_lines = [
                    # f"FPS: {latency['fps']:.1f}",
                    # f"frame tx:   {latency['frame_ms']:.1f} ms",
                    # f"inference:  {latency['inference_ms']:.1f} ms",
                    # f"post RTT:   {latency['post_rtt_ms']:.1f} ms",
                    # f"serial:     {latency['serial_ms']:.2f} ms",
                    # f"total est:  {total_ms:.1f} ms",
                    # state_label,
                    f"lin: {cmd.linear_vel:+.2f}  ang: {cmd.angular_vel:+.2f}",
                ]
            for i, text in enumerate(hud_lines):
                cv2.putText(frame, text, (8, 24 + i * 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

            cv2.imshow("Pi Camera Stream - YOLOv8n", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                resp.close()
                cv2.destroyAllWindows()
                print("Stream closed.")
                return
    except requests.exceptions.ConnectionError:
        print("Stream connection lost.")
        cv2.destroyAllWindows()


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
    print(f"Tracking: {args.track} (COCO class {class_id})")
    display_stream(args.ip, class_id)
