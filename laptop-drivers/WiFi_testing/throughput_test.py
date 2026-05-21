#!/usr/bin/env python3
"""
Throughput / loss test for the ESP-NOW UART bridge.

Topology this script assumes (symmetric, no firmware change):

  laptop ─┬─ CP2102 #1 ──> SHIP ESP32 (GPIO16/17) ──ESP-NOW──┐
          │   (sender)                                        │
          └─ CP2102 #2 <── BUOY ESP32 (GPIO16/17) <───────────┘
              (receiver)

Both ESP32 USB ports stay free so you can watch the [TX] ok=/fail=
logs in a serial monitor while this runs.

What it does:
  - builds a known test payload
  - SLIP-encodes it into frames (the bridge forwards whole SLIP frames,
    so encoding must be real, and each frame stays under the 250B
    ESP-NOW limit once encoded)
  - sends N frames into the SHIP port as fast as the link accepts
  - reads them back off the BUOY port
  - reports goodput (payload bytes/sec), frames sent vs received, loss
  - optionally sweeps several baud rates to find where UART is the ceiling

Usage examples:
  python3 throughput_test.py --ship /dev/cu.usbserial-AAAA \
                             --buoy /dev/cu.usbserial-BBBB
  python3 throughput_test.py --ship COM5 --buoy COM6 --sweep
  python3 throughput_test.py --ship ... --buoy ... --baud 921600 \
                             --frames 2000 --payload 200

List ports with:  python3 -m serial.tools.list_ports -v
Requires:         pip install pyserial
"""

import argparse
import sys
import time

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial not found. Install with:  pip install pyserial")

# ---- SLIP constants (RFC 1055) --------------------------------------
SLIP_END     = 0xC0
SLIP_ESC     = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD


def open_serial(port, baud, write_timeout=None):
    """Open a serial port robustly on macOS.

    Avoids serial.Serial(..., timeout=0): its internal switch to
    non-blocking mode triggers a second tcsetattr() that some macOS
    driver/pyserial combos reject with termios (22, 'Invalid argument').
    Construct unopened, set a small non-zero timeout, then open.
    """
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05
    if write_timeout is not None:
        ser.write_timeout = write_timeout
    ser.open()
    return ser



def slip_encode(payload: bytes) -> bytes:
    """Encode one payload into a SLIP frame (END-delimited, escaped)."""
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


def slip_decode_stream(buf: bytearray):
    """Pull complete SLIP frames out of a running byte buffer.

    Returns (list_of_decoded_payloads, leftover_buffer). Handles the
    leading/trailing END bytes and un-escapes the payload.
    """
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
                # leading END (start of a frame) or empty run
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
            # escape byte split across reads — stop, keep remainder
            break
        else:
            cur.append(b)
            i += 1
    # keep any unparsed tail (incl. a partial frame in progress)
    leftover = buf[last_consumed:]
    return frames, bytearray(leftover)


def build_frames(n_frames: int, payload_size: int):
    """Make n test frames. Each payload starts with a 4-byte big-endian
    sequence number so the receiver can detect loss / reordering, padded
    out to payload_size with a recognizable pattern."""
    frames = []
    for seq in range(n_frames):
        body = bytearray(seq.to_bytes(4, "big"))
        # fill remainder with a counting pattern (avoids long runs)
        while len(body) < payload_size:
            body.append((len(body) + seq) & 0xFF)
        body = body[:payload_size]
        frames.append((seq, slip_encode(bytes(body))))
    return frames


def run_once(ship_port, buoy_port, baud, n_frames, payload_size,
             read_timeout):
    enc_frames = build_frames(n_frames, payload_size)
    enc_len = len(enc_frames[0][1])
    if enc_len > 250:
        print(f"  ! encoded frame is {enc_len}B (>250). Reduce --payload.")
        return None

    payload_total = n_frames * payload_size

    ship = open_serial(ship_port, baud, write_timeout=5)
    buoy = open_serial(buoy_port, baud)
    # let the lines settle
    time.sleep(0.2)
    ship.reset_input_buffer(); ship.reset_output_buffer()
    buoy.reset_input_buffer(); buoy.reset_output_buffer()

    received = {}
    rx_raw = bytearray()
    leftover = bytearray()

    t0 = time.perf_counter()

    # Writer: push all frames out the ship side.
    for _seq, fr in enc_frames:
        ship.write(fr)
    ship.flush()
    t_sent = time.perf_counter()

    # Reader: drain buoy side until we have all frames or hit idle timeout.
    last_rx = time.perf_counter()
    while True:
        chunk = buoy.read(4096)
        if chunk:
            rx_raw += chunk
            leftover += chunk
            frames, leftover = slip_decode_stream(leftover)
            for f in frames:
                if len(f) >= 4:
                    seq = int.from_bytes(f[:4], "big")
                    received[seq] = f
            last_rx = time.perf_counter()
            if len(received) >= n_frames:
                break
        else:
            if time.perf_counter() - last_rx > read_timeout:
                break
            time.sleep(0.001)

    t_end = time.perf_counter()
    ship.close(); buoy.close()

    got = len(received)
    lost = n_frames - got
    send_secs = t_sent - t0
    total_secs = t_end - t0
    got_bytes = got * payload_size

    return {
        "baud": baud,
        "payload_size": payload_size,
        "encoded_frame_len": enc_len,
        "frames_sent": n_frames,
        "frames_recv": got,
        "frames_lost": lost,
        "loss_pct": 100.0 * lost / n_frames if n_frames else 0.0,
        "payload_total_bytes": payload_total,
        "recv_payload_bytes": got_bytes,
        # goodput measured over the whole send+receive window
        "goodput_Bps": got_bytes / total_secs if total_secs > 0 else 0.0,
        "send_window_s": send_secs,
        "total_window_s": total_secs,
    }


def run_latency(ship_port, buoy_port, baud, n_frames, payload_size,
                gap_s, per_frame_timeout):
    """One-way latency test (single-laptop, shared clock).

    Sends frames ONE AT A TIME and measures the wall-clock delta between
    injecting frame N on the ship port and reading it back on the buoy
    port. Because one machine owns both ports, both timestamps come from
    the same clock, so no clock-sync error. This is true one-way latency:
      ship-host -> ship-ESP -> air -> buoy-ESP -> buoy-host.

    gap_s spaces frames out so each round is measured in isolation (a
    back-to-back blast would measure queueing, not latency). Frames that
    don't arrive within per_frame_timeout are counted as lost.
    """
    enc_frames = build_frames(n_frames, payload_size)
    enc_len = len(enc_frames[0][1])
    if enc_len > 250:
        print(f"  ! encoded frame is {enc_len}B (>250). Reduce --payload.")
        return None

    ship = open_serial(ship_port, baud, write_timeout=5)
    buoy = open_serial(buoy_port, baud)
    time.sleep(0.2)
    ship.reset_input_buffer(); ship.reset_output_buffer()
    buoy.reset_input_buffer(); buoy.reset_output_buffer()

    latencies = []          # seconds, one per successfully round-tripped frame
    lost = 0
    leftover = bytearray()

    for seq, fr in enc_frames:
        # drop any stale bytes so a previous straggler can't be mis-timed
        leftover = bytearray()
        t_send = time.perf_counter()
        ship.write(fr)
        ship.flush()

        got = False
        deadline = t_send + per_frame_timeout
        while time.perf_counter() < deadline:
            chunk = buoy.read(4096)
            if chunk:
                leftover += chunk
                frames, leftover = slip_decode_stream(leftover)
                for f in frames:
                    if len(f) >= 4 and int.from_bytes(f[:4], "big") == seq:
                        latencies.append(time.perf_counter() - t_send)
                        got = True
                        break
                if got:
                    break
            else:
                time.sleep(0.0002)
        if not got:
            lost += 1
        time.sleep(gap_s)

    ship.close(); buoy.close()

    if not latencies:
        return {"baud": baud, "payload_size": payload_size,
                "encoded_frame_len": enc_len, "frames_sent": n_frames,
                "frames_recv": 0, "frames_lost": lost, "mode": "latency",
                "all_lost": True}

    s = sorted(latencies)
    def pct(p):
        return s[min(len(s) - 1, int(p / 100.0 * len(s)))]
    mean = sum(s) / len(s)
    # jitter = mean absolute deviation between consecutive samples
    jitter = (sum(abs(s[i] - s[i - 1]) for i in range(1, len(s)))
              / (len(s) - 1)) if len(s) > 1 else 0.0

    return {
        "mode": "latency",
        "baud": baud,
        "payload_size": payload_size,
        "encoded_frame_len": enc_len,
        "frames_sent": n_frames,
        "frames_recv": len(latencies),
        "frames_lost": lost,
        "loss_pct": 100.0 * lost / n_frames if n_frames else 0.0,
        "min_ms": s[0] * 1000,
        "median_ms": pct(50) * 1000,
        "mean_ms": mean * 1000,
        "p95_ms": pct(95) * 1000,
        "max_ms": s[-1] * 1000,
        "jitter_ms": jitter * 1000,
    }


def fmt_latency(res):
    if not res:
        return ""
    if res.get("all_lost"):
        return (f"  baud={res['baud']:>7}  payload={res['payload_size']:>3}B  "
                f"ALL {res['frames_sent']} FRAMES LOST — check wiring / MACs / baud")
    return (
        f"  baud={res['baud']:>7}  payload={res['payload_size']:>3}B  "
        f"recv={res['frames_recv']:>4}/{res['frames_sent']:<4} "
        f"loss={res['loss_pct']:4.1f}%  "
        f"min={res['min_ms']:6.2f}  med={res['median_ms']:6.2f}  "
        f"mean={res['mean_ms']:6.2f}  p95={res['p95_ms']:6.2f}  "
        f"max={res['max_ms']:7.2f}  jitter={res['jitter_ms']:5.2f}  (ms)"
    )


def fmt(res):
    if not res:
        return ""
    kbps = res["goodput_Bps"] * 8 / 1000.0
    return (
        f"  baud={res['baud']:>7}  "
        f"payload={res['payload_size']:>3}B  "
        f"encFrame={res['encoded_frame_len']:>3}B  "
        f"sent={res['frames_sent']:>5}  "
        f"recv={res['frames_recv']:>5}  "
        f"loss={res['loss_pct']:5.1f}%  "
        f"goodput={res['goodput_Bps']/1000:7.2f} kB/s "
        f"({kbps:7.1f} kbit/s)"
    )


def list_serial_ports():
    """Print every serial port with description + USB serial number, so
    you can tell two near-identical CP2102s apart and confirm a port is
    actually present before using it."""
    ports = sorted(list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print("No serial ports found. Is anything plugged in?")
        return
    print("Available serial ports:")
    for p in ports:
        ser = p.serial_number or "—"
        desc = p.description or "—"
        print(f"  {p.device:<28} SER={ser:<18} {desc}")
    print("\nTip: note each CP2102's SER= value once; the cu.usbserial-N")
    print("number can reshuffle between replugs, but SER is stable.")


def probe_link(ship_port, buoy_port, baud, payload_size, timeout_s):
    """Send a single frame ship->buoy and report whether it came back.
    Use this to confirm wiring/direction before a full run."""
    try:
        ship = open_serial(ship_port, baud, write_timeout=5)
    except serial.SerialException as e:
        print(f"  ship port {ship_port} failed to open: {e}")
        return False
    try:
        buoy = open_serial(buoy_port, baud)
    except serial.SerialException as e:
        print(f"  buoy port {buoy_port} failed to open: {e}")
        ship.close()
        return False

    time.sleep(0.2)
    ship.reset_input_buffer(); ship.reset_output_buffer()
    buoy.reset_input_buffer(); buoy.reset_output_buffer()

    seq, fr = build_frames(1, payload_size)[0]
    t0 = time.perf_counter()
    ship.write(fr); ship.flush()

    leftover = bytearray()
    ok = False
    while time.perf_counter() - t0 < timeout_s:
        chunk = buoy.read(4096)
        if chunk:
            leftover += chunk
            frames, leftover = slip_decode_stream(leftover)
            for f in frames:
                if len(f) >= 4 and int.from_bytes(f[:4], "big") == seq:
                    ok = True
                    break
        if ok:
            break
        time.sleep(0.001)
    rtt = (time.perf_counter() - t0) * 1000
    ship.close(); buoy.close()

    if ok:
        print(f"  PROBE OK — frame returned in {rtt:.2f} ms. Direction & link good.")
    else:
        print(f"  PROBE FAILED — no frame back within {timeout_s}s.")
        print("    Check: (1) --ship/--buoy not swapped, (2) both ESP32 powered,")
        print("    (3) each board has the OTHER's PEER_MAC, (4) --baud matches DATA_BAUD.")
    return ok


def main():
    ap = argparse.ArgumentParser(description="ESP-NOW UART bridge throughput + latency test")
    ap.add_argument("--ship", help="serial port feeding the SHIP ESP32 (sender)")
    ap.add_argument("--buoy", help="serial port reading the BUOY ESP32 (receiver)")
    ap.add_argument("--list", action="store_true", help="list serial ports (with SER numbers) and exit")
    ap.add_argument("--probe", action="store_true", help="send one frame to confirm direction/link, then exit")
    ap.add_argument("--baud", type=int, default=115200, help="UART baud (must match firmware DATA_BAUD)")
    ap.add_argument("--mode", choices=["throughput", "latency", "both"],
                    default="both", help="which test to run (default: both)")
    ap.add_argument("--frames", type=int, default=1000, help="frames for the throughput test")
    ap.add_argument("--payload", type=int, default=200, help="payload bytes per frame (pre-SLIP)")
    ap.add_argument("--read-timeout", type=float, default=2.0, help="throughput: idle seconds before giving up on stragglers")
    # latency-specific
    ap.add_argument("--lat-frames", type=int, default=200, help="frames for the latency test (sent one at a time)")
    ap.add_argument("--gap", type=float, default=0.02, help="latency: seconds between frames (isolate each round)")
    ap.add_argument("--lat-timeout", type=float, default=1.0, help="latency: per-frame timeout in seconds")
    ap.add_argument("--sweep", action="store_true", help="sweep a range of baud rates")
    args = ap.parse_args()

    if args.list:
        list_serial_ports()
        return

    if not args.ship or not args.buoy:
        sys.exit("Need --ship and --buoy ports. Run with --list to see "
                 "available ports.")

    if args.probe:
        print(f"Probing {args.ship} -> {args.buoy} @ {args.baud} baud ...")
        probe_link(args.ship, args.buoy, args.baud, args.payload, 1.0)
        return

    print("ESP-NOW UART bridge test")
    print(f"  ship(sender)={args.ship}  buoy(receiver)={args.buoy}")
    print(f"  mode={args.mode}  payload={args.payload}B\n")
    print("  NOTE: --baud must match the firmware's DATA_BAUD. To sweep")
    print("        meaningfully, re-flash the firmware at each baud; a")
    print("        software-only sweep will mismatch a fixed-baud build.\n")

    bauds = [9600, 57600, 115200, 230400, 460800, 921600] if args.sweep else [args.baud]

    tput_results = []
    lat_results = []
    for b in bauds:
        if args.mode in ("throughput", "both"):
            print(f"[throughput] @ {b} baud ...")
            try:
                res = run_once(args.ship, args.buoy, b, args.frames,
                               args.payload, args.read_timeout)
                if res:
                    print(fmt(res))
                    tput_results.append(res)
            except serial.SerialException as e:
                print(f"  serial error: {e}")
            time.sleep(0.3)

        if args.mode in ("latency", "both"):
            print(f"[latency]    @ {b} baud ...")
            try:
                res = run_latency(args.ship, args.buoy, b, args.lat_frames,
                                  args.payload, args.gap, args.lat_timeout)
                if res:
                    print(fmt_latency(res))
                    lat_results.append(res)
            except serial.SerialException as e:
                print(f"  serial error: {e}")
            time.sleep(0.3)

    if tput_results:
        best = max(tput_results, key=lambda r: r["goodput_Bps"])
        print("\nBest goodput:")
        print(fmt(best))
        print("  (decoded payload bytes over the full send+drain window;")
        print("   sharp loss at higher baud = UART/bridge is the ceiling.)")

    if lat_results:
        best = min((r for r in lat_results if not r.get("all_lost")),
                   key=lambda r: r["median_ms"], default=None)
        if best:
            print("\nLowest median latency:")
            print(fmt_latency(best))
            print("  (one-way host->host, shared clock; 'max' and 'jitter'")
            print("   matter most if your protocol has timing requirements.)")


if __name__ == "__main__":
    main()
