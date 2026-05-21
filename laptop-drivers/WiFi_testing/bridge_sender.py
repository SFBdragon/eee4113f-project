#!/usr/bin/env python3
"""
SENDER half of the split throughput/latency test.

Use this when two USB-UART adapters share a serial number and macOS
refuses to open both in one process (termios (22) error). Run this in
ONE terminal and bridge_receiver.py in ANOTHER — each process opens only
its own single port, so the duplicate-serial collision never triggers.

This sender writes timestamped SLIP frames to the SHIP port. The
receiver reads them off the BUOY port and computes latency + throughput
using the embedded timestamps. Because both run on the same laptop, the
clocks match, so one-way latency is meaningful.

Each frame payload layout (big-endian):
   bytes 0..3  : sequence number (uint32)
   bytes 4..11 : send timestamp, microseconds since an epoch we print
                 at startup (uint64) — receiver subtracts to get latency
   bytes 12..  : padding pattern up to --payload

Usage:
   python3 bridge_sender.py --port /dev/cu.usbserial-5 --baud 115200 \
                            --frames 1000 --payload 200 --rate 0
   # --rate 0  = blast as fast as possible (throughput test)
   # --rate N  = N frames/sec, spaced out (latency test)
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

# Shared wall-clock epoch base. Both scripts use time.time() (Unix epoch),
# so the receiver can interpret the embedded send timestamp directly.


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


def make_payload(seq: int, size: int) -> bytes:
    body = bytearray()
    body += seq.to_bytes(4, "big")
    # microseconds since Unix epoch, captured as late as possible
    ts_us = int(time.time() * 1_000_000)
    body += ts_us.to_bytes(8, "big")
    while len(body) < size:
        body.append((len(body) + seq) & 0xFF)
    return bytes(body[:size])


def open_serial(port, baud):
    """Open robustly on macOS — see note in bridge_receiver.py.
    Avoids the timeout=0 non-blocking tcsetattr path that throws
    termios (22) on some macOS setups."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05
    ser.write_timeout = 10
    ser.open()
    return ser


def main():
    ap = argparse.ArgumentParser(description="Split test SENDER (ship side)")
    ap.add_argument("--port", help="ship serial port")
    ap.add_argument("--list", action="store_true", help="list ports and exit")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--frames", type=int, default=1000)
    ap.add_argument("--payload", type=int, default=200, help="payload bytes (>=12)")
    ap.add_argument("--rate", type=float, default=0.0,
                    help="frames/sec; 0 = max rate (throughput). Use a low "
                         "rate (e.g. 50) for a clean latency measurement.")
    args = ap.parse_args()

    if args.list:
        for p in sorted(list_ports.comports(), key=lambda p: p.device):
            print(f"  {p.device:<28} SER={p.serial_number or '—':<18} {p.description}")
        return
    if not args.port:
        sys.exit("Need --port (or --list).")
    if args.payload < 12:
        sys.exit("--payload must be >= 12 (4B seq + 8B timestamp).")

    enc_len = len(slip_encode(make_payload(0, args.payload)))
    if enc_len > 250:
        sys.exit(f"Encoded frame {enc_len}B > 250. Reduce --payload.")

    ser = open_serial(args.port, args.baud)
    time.sleep(0.2)
    ser.reset_output_buffer()

    interval = (1.0 / args.rate) if args.rate > 0 else 0.0
    print(f"Sending {args.frames} frames, payload={args.payload}B "
          f"(encoded {enc_len}B), rate="
          f"{'MAX' if args.rate == 0 else str(args.rate)+'/s'}")
    print("Start the receiver FIRST if you haven't. Sending in 1.5s...")
    time.sleep(1.5)

    t0 = time.perf_counter()
    next_send = time.perf_counter()
    for seq in range(args.frames):
        # build payload with timestamp as late as possible before write
        ser.write(slip_encode(make_payload(seq, args.payload)))
        if interval:
            next_send += interval
            sleep = next_send - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
    ser.flush()
    dt = time.perf_counter() - t0
    ser.close()
    print(f"Done. Sent {args.frames} frames in {dt:.3f}s "
          f"({args.frames/dt:.0f} frames/s offered).")


if __name__ == "__main__":
    main()
