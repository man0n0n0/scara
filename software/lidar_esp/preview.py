#!/usr/bin/env python3
"""See what the ESP32-CAM pole tracker sees.

    python3 preview.py status        # live heartbeat: score vs threshold, fps
    python3 preview.py frame         # grab a frame, show it in the terminal, save PNG
    python3 preview.py frame --quarter
    python3 preview.py profiles      # the two column profiles the detector works on

"status" is the first thing to run when nothing is detected: it says whether the
board is alive, what the best match score is, and what it has to beat.
"""

import argparse
import sys
import time

import serial
from serial.tools import list_ports

CSI = "\x1b["


def pick_port(explicit):
    if explicit:
        return explicit
    ports = [p for p in list_ports.comports() if p.vid is not None]
    if not ports:
        sys.exit("no serial device found; pass -p /dev/ttyUSB0")
    return ports[0].device


def read_line(ser, timeout=5.0):
    """Read one text line, tolerating binary garbage left in the buffer."""
    end = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < end:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            return buf.decode("utf-8", "replace").strip()
        if b != b"\r":
            buf += b
    return None


def grab_frame(ser, quarter):
    ser.reset_input_buffer()
    ser.write(b"g" if quarter else b"f")
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        line = read_line(ser, timeout=5)
        if line is None:
            continue
        if line.startswith("F "):
            _, w, h, n = line.split()
            w, h, n = int(w), int(h), int(n)
            data = bytearray()
            while len(data) < n and time.monotonic() < deadline + 20:
                chunk = ser.read(n - len(data))
                if chunk:
                    data += chunk
            if len(data) != n:
                sys.exit(f"short frame: {len(data)}/{n} bytes "
                         f"(raise --baud reliability or use --quarter)")
            return w, h, bytes(data)
    sys.exit("board never answered with a frame header; is the sketch running?")


def show_terminal(w, h, data, cols=100):
    """Half-block render: two image rows per text row, xterm 24-step grayscale."""
    step = max(1, w // cols)
    out = []
    for y in range(0, h - step, step * 2):
        row = []
        for x in range(0, w - step, step):
            top = data[y * w + x]
            bot = data[min(y + step, h - 1) * w + x]
            row.append(f"{CSI}38;5;{232 + top * 23 // 255}m"
                       f"{CSI}48;5;{232 + bot * 23 // 255}m▀")
        out.append("".join(row) + f"{CSI}0m")
    print("\n".join(out))


def save_png(w, h, data, path, det=None):
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print("(PIL not installed, skipping PNG)")
        return
    img = Image.frombytes("L", (w, h), data).convert("RGB")
    if det:
        x, ybase, width = det
        d = ImageDraw.Draw(img)
        x0, x1 = x - width / 2, x + width / 2
        d.line([(x0, 0), (x0, h)], fill=(255, 60, 60))
        d.line([(x1, 0), (x1, h)], fill=(255, 60, 60))
        if ybase >= 0:
            d.line([(x0 - 15, ybase), (x1 + 15, ybase)], fill=(60, 180, 255))
    img.save(path)
    print(f"saved {path} ({w}x{h})" + ("  overlay: detection drawn" if det else ""))


def last_detection(ser, wait=2.0):
    end = time.monotonic() + wait
    det = None
    while time.monotonic() < end:
        line = read_line(ser, timeout=0.5)
        if line and line.startswith("P "):
            p = line.split()
            det = (float(p[1]), int(p[2]), int(p[3]))
    return det


def sparkline(vals, width=100, height=10):
    step = max(1, len(vals) // width)
    agg = [max(vals[i:i + step]) for i in range(0, len(vals), step)][:width]
    top = max(agg) or 1
    rows = []
    for level in range(height, 0, -1):
        rows.append("".join("#" if v * height / top >= level else " " for v in agg))
    return "\n".join(rows), top, agg


def cmd_status(ser, args):
    print("waiting for status lines ('S best floor thresh fps') -- Ctrl-C to stop\n")
    seen = {"P": 0, "N": 0, "S": 0, "other": 0}
    try:
        while True:
            line = read_line(ser, timeout=5)
            if line is None:
                print("  ... nothing on the port for 5 s. Wrong port or baud, "
                      "or the sketch is not running.")
                continue
            tag = line.split()[0] if line.split() else ""
            if tag == "S":
                _, best, floor, thresh, fps = line.split()
                ratio = float(best) / float(thresh) if float(thresh) else 0
                verdict = "DETECTING" if ratio > 1 else f"below threshold ({ratio:.2f}x)"
                print(f"  best {best:>8}  floor {floor:>8}  needs >{thresh:>8}  "
                      f"{fps:>5} fps   {verdict}")
                seen["S"] += 1
            elif tag in ("P", "N"):
                seen[tag] += 1
            else:
                print(f"  {line}")
                seen["other"] += 1
    except KeyboardInterrupt:
        print(f"\n  frames with detection: {seen['P']}, without: {seen['N']}, "
              f"status: {seen['S']}, other lines: {seen['other']}")


def cmd_frame(ser, args):
    det = last_detection(ser) if not args.no_overlay else None
    w, h, data = grab_frame(ser, args.quarter)
    lo, hi = min(data), max(data)
    mean = sum(data) / len(data)
    print(f"frame {w}x{h}  min {lo}  max {hi}  mean {mean:.1f}")
    if hi - lo < 20:
        print("  !! almost no contrast: exposure/gain are wrong, or the lens cap is on.")
        print("     Tune set_aec_value() / set_agc_gain() in the sketch.")
    elif mean < 25:
        print("  !! very dark frame: raise set_aec_value() or set_agc_gain().")
    elif mean > 230:
        print("  !! saturated frame: lower set_aec_value() or set_agc_gain().")
    show_terminal(w, h, data, args.cols)
    if det:
        print(f"detection overlay: x={det[0]:.1f} base={det[1]} w={det[2]}")
    save_png(w, h, data, args.out, det)


def cmd_profiles(ser, args):
    ser.reset_input_buffer()
    ser.write(b"p")
    prof = {}
    deadline = time.monotonic() + 10
    while len(prof) < 2 and time.monotonic() < deadline:
        line = read_line(ser, timeout=5)
        if line and line[:2] in ("D ", "E "):
            prof[line[0]] = [int(v) for v in line[1:].split()]
    if len(prof) < 2:
        sys.exit("board did not send profiles")

    for key, name in (("D", "rising edges (dpos)"), ("E", "falling edges (dneg)")):
        plot, top, _ = sparkline(prof[key], args.cols)
        print(f"\n{name}  peak {top}")
        print(plot)
        print("-" * min(args.cols, len(prof[key])))
    flat = max(max(prof["D"]), max(prof["E"]))
    if flat < 100:
        print("\n  !! both profiles are nearly flat. The detector has nothing to work "
              "with:\n     check the frame first with 'preview.py frame'.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["status", "frame", "profiles"])
    ap.add_argument("-p", "--port")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("--cols", type=int, default=100, help="terminal render width")
    ap.add_argument("--quarter", action="store_true", help="decimated frame, 4x faster")
    ap.add_argument("--no-overlay", action="store_true")
    ap.add_argument("-o", "--out", default="preview.png")
    args = ap.parse_args()

    port = pick_port(args.port)
    print(f"{port} @ {args.baud}")
    with serial.Serial(port, args.baud, timeout=1) as ser:
        time.sleep(0.3)
        {"status": cmd_status, "frame": cmd_frame, "profiles": cmd_profiles}[args.cmd](ser, args)


if __name__ == "__main__":
    main()
