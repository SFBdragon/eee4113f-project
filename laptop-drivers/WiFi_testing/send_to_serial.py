# Only useable script I've made so far lol
# But it works!
# It sends slip-encoded messages to the serial port, which the buoy should be able to read and transmit via ESP-NOW.

import serial
import time

PORT = "/dev/cu.usbserial-6"   # the USB port identifier on the laptop 
BAUD = 115200

SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

def slip_encode(payload: bytes) -> bytes:
    out = bytearray([SLIP_END])
    for b in payload:
        if b == SLIP_END:
            out += bytes([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC:
            out += bytes([SLIP_ESC, SLIP_ESC_ESC])
        else:
            out.append(b)
    out.append(SLIP_END)
    return bytes(out)

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(0.5)

for i in range(10):
    msg = f"Buoy test frame {i}".encode()
    ser.write(slip_encode(msg))
    print("sent:", msg)
    time.sleep(1)

ser.close()