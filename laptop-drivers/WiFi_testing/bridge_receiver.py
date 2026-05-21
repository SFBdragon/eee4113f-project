#!/usr/bin/env python3
"""
RECEIVER half of the split throughput/latency test.

Run this in ONE terminal and bridge_sender.py in ANOTHER. This process
opens ONLY the buoy port, so it never collides with the sender's port
even when the two USB adapters share a serial number.

It reads SLIP frames off the BUOY port, pulls the embedded sequence
number and send-timestamp out of each, and reports:
  - frames received / lost / out-of-order
  - one-way latency (min / median / mean / p95 / max / jitter)
  - goodput (decoded payload bytes per second over the receive window)

Latency works because the send timestamp is wall-clock (time.time())
from the SAME laptop, so subtracting the arrival wall-clock is valid.

Usage:
  # terminal 1:
  python3 bridge_receiver.py --port /dev/cu.usbserial-0001 --baud 115200 \
                             --expect 1000 --payload 200
  # terminal 2 (start within a few seconds):
  python3 bridge_sender.py --port /dev/cu.usbserial-5 --baud 115200 \
                           --frames 1000 --payload 200 --rate 0
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial not found. Install with:  pip install pyserial")

SLIP_END     = 0xC0
SLIP_ESC     = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD


def slip_decode_stream(buf: bytearray):
    """Pull complete SLIP frames out of a running byte buffer.
    Returns (list_of_payloads, leftover)."""
    frames = []
    cur = bytearray()
    i = 0
    in_frame = False
    last_consumed = 0
    while i < len(buf):
        b = buf[i]
        if b == SLIP_END:
            if in_frame and len(cur) > 0:
                frames.append(bytes(cur))
                cur = bytearray()
                in_frame = False
                last_consumed = i + 1
            else:
                in_frame = True
                cur = bytearray()
            i += 1
            continue
        in_frame = True
        if b == SLIP_ESC and i + 1 < len(buf):
            nxt = buf[i + 1]
            if nxt == SLIP_ESC_END:
                cur.append(SLIP_END)
            elif nxt == SLIP_ESC_ESC:
                cur.append(SLIP_ESC)
            else:
                cur.append(nxt)
            i += 2
            continue
        elif b == SLIP_ESC:
            break  # escape split across reads; keep remainder
        else:
            cur.append(b)
            i += 1
    leftover = buf[last_consumed:]
    return frames, bytearray(leftover)


def open_serial(port, baud):
    """Open a serial port robustly on macOS.

    Avoids serial.Serial(..., timeout=0), whose internal switch to
    non-blocking mode triggers a second tcsetattr() that some macOS
    driver/pyserial combos reject with termios (22, 'Invalid argument').
    We construct unopened, set a small non-zero timeout, then open.
    """
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05          # small non-zero: non-blocking-ish reads, no tcsetattr(0) path
    ser.write_timeout = 10
    ser.open()
    return ser


def main():
    ap = argparse.ArgumentParser(description="Split test RECEIVER (buoy side)")
    ap.add_argument("--port", help="buoy serial port")
    ap.add_argument("--list", action="store_true", help="list ports and exit")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--expect", type=int, default=1000, help="frames expected (stop when reached)")
    ap.add_argument("--payload", type=int, default=200, help="payload bytes per frame")
    ap.add_argument("--idle-timeout", type=float, default=3.0,
                    help="stop after this many seconds of no new frames")
    args = ap.parse_args()

    if args.list:
        for p in sorted(list_ports.comports(), key=lambda p: p.device):
            print(f"  {p.device:<28} SER={p.serial_number or '—':<18} {p.description}")
        return
    if not args.port:
        sys.exit("Need --port (or --list).")

    ser = open_serial(args.port, args.baud)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print(f"Receiver listening on {args.port} @ {args.baud}, "
          f"expecting {args.expect} frames. Start the sender now.")

    latencies = []          # seconds
    seqs_seen = set()
    out_of_order = 0
    last_seq = -1
    first_arrival = None
    last_arrival = None
    leftover = bytearray()
    last_rx = time.perf_counter()

    while True:
        chunk = ser.read(4096)
        now_wall = time.time()
        if chunk:
            arrival_perf = time.perf_counter()
            leftover += chunk
            frames, leftover = slip_decode_stream(leftover)
            for f in frames:
                if len(f) < 12:
                    continue
                seq = int.from_bytes(f[:4], "big")
                ts_us = int.from_bytes(f[4:12], "big")
                lat = now_wall - (ts_us / 1_000_000.0)
                # guard against absurd values from a split/garbled frame
                if -0.5 < lat < 30.0:
                    latencies.append(lat)
                if seq in seqs_seen:
                    continue
                seqs_seen.add(seq)
                if seq < last_seq:
                    out_of_order += 1
                last_seq = seq
                if first_arrival is None:
                    first_arrival = arrival_perf
                last_arrival = arrival_perf
            last_rx = time.perf_counter()
            if len(seqs_seen) >= args.expect:
                break
        else:
            if time.perf_counter() - last_rx > args.idle_timeout:
                break
            time.sleep(0.0005)

    ser.close()

    got = len(seqs_seen)
    lost = args.expect - got
    print("\n=== RESULTS ===")
    print(f"frames received : {got}/{args.expect}  "
          f"(lost {lost}, {100.0*lost/args.expect:.1f}%)")
    print(f"out-of-order    : {out_of_order}")

    if first_arrival and last_arrival and last_arrival > first_arrival:
        window = last_arrival - first_arrival
        good_bytes = got * args.payload
        print(f"receive window  : {window:.3f}s")
        print(f"goodput         : {good_bytes/window/1000:.2f} kB/s "
              f"({good_bytes*8/window/1000:.1f} kbit/s)")

    if latencies:
        s = sorted(latencies)
        def pct(p): return s[min(len(s)-1, int(p/100.0*len(s)))]
        mean = sum(s)/len(s)
        jitter = (sum(abs(s[i]-s[i-1]) for i in range(1, len(s)))
                  / (len(s)-1)) if len(s) > 1 else 0.0
        print("one-way latency (ms): "
              f"min={s[0]*1000:.2f}  med={pct(50)*1000:.2f}  "
              f"mean={mean*1000:.2f}  p95={pct(95)*1000:.2f}  "
              f"max={s[-1]*1000:.2f}  jitter={jitter*1000:.2f}")
        print("  (valid only if sender & receiver ran on the same machine;")
        print("   for a clean latency number use the sender's --rate option.)")
    else:
        print("no usable latency samples (frames too short or all garbled).")


if __name__ == "__main__":
    main()
