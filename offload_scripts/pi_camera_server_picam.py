#!/usr/bin/env python3
"""
MJPEG camera server using the Raspberry Pi camera module (picamera2).
Streams over HTTP on port 5000.
Forwards motor commands to an ESP via serial as a binary packet.

Packet format (10 bytes):
    [0xAA][0xFF]  – 2-byte header
    [float32 LE]  – linear_vel
    [float32 LE]  – angular_vel

Install dependencies:
    pip3 install flask picamera2 pyserial
"""

import struct
import time
import threading
import io
import serial
from flask import Flask, Response, request, jsonify
import libcamera
from picamera2 import Picamera2
from PIL import Image

app = Flask(__name__)

latest_frame = None
frame_lock = threading.Lock()

RESOLUTION      = (640, 480)
JPEG_QUALITY    = 80
EVERY_NTH_FRAME = 3        # encode and publish only every Nth captured frame
PORT            = 5000

SERIAL_PORT     = "/dev/serial0"   # adjust if ESP shows up on a different port
SERIAL_BAUD     = 115200
PACKET_HEADER   = b'\xAA\x55'

ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
print(f"Serial open: {SERIAL_PORT} @ {SERIAL_BAUD}")


def capture_loop():
    global latest_frame

    picam2 = Picamera2()
    full_res = picam2.camera_properties["PixelArraySize"]
    config = picam2.create_video_configuration(
        main={"size": RESOLUTION, "format": "RGB888"},
        raw={"size": (1640, 1232)},   # full-FOV binned sensor mode on IMX219
        controls={"ScalerCrop": (0, 0, full_res[0], full_res[1])},
        transform=libcamera.Transform(vflip=True, hflip=True)
    )
    picam2.configure(config)
    picam2.start()
    print("Full sensor:", full_res)
    print("ScalerCrop:", picam2.capture_metadata()["ScalerCrop"])

    count = 0
    while True:
        frame = picam2.capture_array()

        count += 1
        if count % EVERY_NTH_FRAME != 0:
            continue

        buf = io.BytesIO()
        Image.fromarray(frame).save(buf, format="JPEG", quality=JPEG_QUALITY)
        with frame_lock:
            latest_frame = (buf.getvalue(), time.time())


def generate_mjpeg():
    while True:
        with frame_lock:
            item = latest_frame
        if item:
            jpeg, capture_time = item
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"X-Capture-Time: " + f"{capture_time:.6f}".encode() + b"\r\n\r\n"
                + jpeg + b"\r\n"
            )
        time.sleep(0.05)


@app.route("/stream")
def stream():
    return Response(
        generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/command", methods=["POST"])
def command():
    data = request.get_json(force=True)
    linear_vel  = float(data.get("linear_vel",  0.0))
    angular_vel = float(data.get("angular_vel", 0.0))
    received_at = time.time()
    packet = PACKET_HEADER + struct.pack('<ff', linear_vel, angular_vel)
    print("writing serial packet: linear_vel: " + str(linear_vel) + ", angular_vel: " + str(angular_vel))
    t_serial = time.time()
    ser.write(packet)
    serial_write_ms = (time.time() - t_serial) * 1000
    return jsonify({"status": "ok", "received_at": received_at, "serial_write_ms": serial_write_ms})


@app.route("/")
def index():
    return "<h2>Pi Camera Live</h2><img src='/stream' />"


if __name__ == "__main__":
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    print(f"Server running on http://0.0.0.0:{PORT}")
    print(f"Streaming {RESOLUTION} from Pi camera module")
    app.run(host="0.0.0.0", port=PORT, threaded=True)
