#!/usr/bin/env python3
"""
One-shot diagnostic for the macOS termios (22, 'Invalid argument') error.

Opens a SINGLE port two ways and reports which succeed:
  (A) the old way:  serial.Serial(port, baud, timeout=0)
  (B) the new way:  construct unopened, timeout=0.05, then .open()

Run with ONLY the port you want to test (you can have other devices
plugged in; this opens just one). This isolates the cause:

  - If (A) fails and (B) works  -> it was the timeout=0 / non-blocking
                                    tcsetattr path. The fixed scripts work.
  - If BOTH fail                -> the baud rate or the port/driver itself
                                    is the problem (try --baud 115200, or a
                                    different cable/port).
  - If BOTH work                -> the port is fine; any earlier failure was
                                    something else (e.g. opening two at once).

Usage:
  python3 port_probe.py --list
  python3 port_probe.py --port /dev/cu.usbserial-0001 --baud 115200
"""
import argparse, sys
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial not found. Install with:  pip install pyserial")


def try_old(port, baud):
    try:
        s = serial.Serial(port, baud, timeout=0)
        s.close()
        return True, ""
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def try_new(port, baud):
    try:
        s = serial.Serial()
        s.port = port
        s.baudrate = baud
        s.timeout = 0.05
        s.open()
        s.close()
        return True, ""
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        for p in sorted(list_ports.comports(), key=lambda p: p.device):
            print(f"  {p.device:<28} SER={p.serial_number or '-':<18} {p.description}")
        return
    if not args.port:
        sys.exit("Need --port (or --list).")

    print(f"Testing {args.port} @ {args.baud} baud\n")
    okA, errA = try_old(args.port, args.baud)
    print(f"(A) serial.Serial(port, baud, timeout=0)     -> "
          f"{'OK' if okA else 'FAIL: ' + errA}")
    okB, errB = try_new(args.port, args.baud)
    print(f"(B) unopened + timeout=0.05 + .open()        -> "
          f"{'OK' if okB else 'FAIL: ' + errB}")

    print()
    if not okA and okB:
        print(">> Confirmed: timeout=0 was the cause. The fixed scripts use (B).")
    elif okA and okB:
        print(">> Port opens fine both ways. Earlier failure was elsewhere.")
    elif not okA and not okB:
        print(">> Both failed. Likely the baud rate or the port/driver/cable.")
        print("   Try --baud 115200, a different USB port, or a different cable.")
    else:
        print(">> (A) worked but (B) didn't — unexpected; paste this output.")


if __name__ == "__main__":
    main()
