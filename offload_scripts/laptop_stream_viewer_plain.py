#!/usr/bin/env python3
"""
Laptop MJPEG stream viewer for the Raspberry Pi camera server.
Connects to the Pi's HTTP stream and displays it in a window.

Install dependencies:
    pip3 install opencv-python requests numpy

Usage:
    python3 laptop_stream_viewer_plain.py <pi_ip_address>

Example:
    python3 laptop_stream_viewer_plain.py 192.168.1.42
"""

import sys
import cv2
import numpy as np
import requests

PORT = 5000
DEFAULT_IP = "youssef.local"  # change to your Pi's IP if mDNS doesn't work


def display_stream(pi_ip: str):
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

    buf = b""
    for chunk in resp.iter_content(chunk_size=4096):
        buf += chunk

        while True:
            start = buf.find(b"\xff\xd8")
            end = buf.find(b"\xff\xd9", start)
            if start == -1 or end == -1:
                break

            jpg = buf[start:end + 2]
            buf = buf[end + 2:]

            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow("Pi Camera Stream", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                resp.close()
                cv2.destroyAllWindows()
                print("Stream closed.")
                return


if __name__ == "__main__":
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP
    display_stream(ip)
