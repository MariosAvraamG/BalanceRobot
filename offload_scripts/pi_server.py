import struct
import time
import threading
import io
import serial
from flask import Flask, Response, request, jsonify, redirect, url_for
import libcamera
from picamera2 import Picamera2
from PIL import Image
import RPi.GPIO as GPIO
import queue


app = Flask(__name__)

latest_frame = None
frame_lock = threading.Lock()

command_queue = queue.Queue()
ultrasound_override = threading.Event()

curr_object = None
object_lock = threading.Lock()

RESOLUTION      = (640, 480)
JPEG_QUALITY    = 80
EVERY_NTH_FRAME = 3
PORT            = 5000

SERIAL_PORT     = "/dev/serial0"
SERIAL_BAUD     = 115200
PACKET_HEADER   = b'\xAA\x55'

ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

GPIO_PIN_TRIG = 23
GPIO_PIN_ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN_TRIG, GPIO.OUT)
GPIO.setup(GPIO_PIN_ECHO, GPIO.IN)


class ultrasoundSensor:
    def get_distance(self, num_of_samples=4):
        samples = []
        for _ in range(num_of_samples):
            GPIO.output(GPIO_PIN_TRIG, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(GPIO_PIN_TRIG, GPIO.LOW)

            timeout = time.monotonic() + 0.03
            start = time.monotonic()
            i = 0
            while GPIO.input(GPIO_PIN_ECHO) == GPIO.LOW:
                start = time.monotonic()
                # if start > timeout:
                #     return None

            timeout = time.monotonic() + 0.03
            end = time.monotonic()
            while GPIO.input(GPIO_PIN_ECHO) == GPIO.HIGH:
                end = time.monotonic()
                # if end > timeout:
                #     return None

            diff = end - start
            samples.append(diff / 2)
            time.sleep(0.05)
        samples.sort()
        med = samples[len(samples) // 2]
        print(f"med: {med}")
        return med * 340 * 100
    
def ultrasound_loop():
    time.sleep(2)
    print("start ultrasound sensor")
    ultrasound = ultrasoundSensor()
    while True:
        dist = ultrasound.get_distance(5)
        if(dist < 15):
            time.sleep(0.5)
        else:
            time.sleep(2)
        if dist is None:
            print("ultrasound timeout — no echo received")
            ultrasound_override.clear()
            continue
        print(f"distance detected: {dist}cm")
        if dist < 15:
            print("queuing ultrasound")
            ultrasound_override.set()
            command_queue.put((0.0, 0.0))
        else:
            ultrasound_override.clear()

def serial_writer_loop():
    while True:
        try:
            linear_vel, angular_vel = command_queue.get(timeout=0.1)
            packet = PACKET_HEADER + struct.pack('<ff', linear_vel, angular_vel)
            ser.write(packet)
            print(f"wrote message via serial: linear: {linear_vel}, angular: {angular_vel}")
        except queue.Empty:
            continue

def capture_loop():
    global latest_frame

    picam2 = Picamera2()
    full_res = picam2.camera_properties["PixelArraySize"]
    config = picam2.create_video_configuration(
        main={"size": RESOLUTION, "format": "RGB888"},
        raw={"size": (1640, 1232)}, 
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
        Image.fromarray(frame[:, :, ::-1]).save(buf, format="JPEG", quality=JPEG_QUALITY)
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
    if ultrasound_override.is_set():
        return jsonify({"status": "blocked", "reason": "obstacle detected"})
        
    data = request.get_json(force=True)
    linear_vel  = float(data.get("linear_vel", 0.0))
    angular_vel = float(data.get("angular_vel", 0.0))
    tracked_object = data.get("tracked_object", None)
    print(f"got tracked object: {tracked_object}")

    global curr_object
    with object_lock:
        curr_object = tracked_object
    
    command_queue.put((linear_vel, angular_vel))
    
    return jsonify({"status": "ok", "received_at": time.time()})

@app.route("/object", methods=["GET"])
def object():
    global curr_object
    with object_lock:
        return jsonify({"current_object" : curr_object})

@app.route("/")
def index():
    return redirect(url_for("/stream"))

@app.after_request
def add_cors(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response


if __name__ == "__main__":
    s = threading.Thread(target=serial_writer_loop, daemon=True)
    s.start()
    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()
    u = threading.Thread(target=ultrasound_loop, daemon=True)
    u.start()
    print(f"Server running on http://0.0.0.0:{PORT}")
    app.run(host="0.0.0.0", port=PORT, threaded=True, debug=False)
