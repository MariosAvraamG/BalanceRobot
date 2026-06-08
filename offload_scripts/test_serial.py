#!/usr/bin/env python3
import serial
import time

SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200

ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
print(f"Opened {SERIAL_PORT} @ {SERIAL_BAUD}, sending 'hello' every second. Ctrl+C to stop.")

while True:
    ser.write(b"hello\n")
    print("Sent: hello")
    time.sleep(1)
