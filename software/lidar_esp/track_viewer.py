#!/usr/bin/env python3
"""Live floor-plane view of the pole tracked by the ESP32-CAM.

    python3 track_viewer.py                      # pixels only, no calibration
    python3 track_viewer.py -H homography.json   # floor XY in mm

Reads the firmware's "P x ybase w score z" / "N" lines.
"""

import argparse
import json
import sys
import time
from collections import deque

import serial
from serial.tools import list_ports

CSI = "\x1b["
PLOT_W, PLOT_H = 60, 22
TRAIL = 40


def pick_port(explicit):
    if explicit:
        return explicit
    ports = [p for p in list_ports.comports() if p.vid is not None]
    if not ports:
        sys.exit("no serial device found; pass -p /dev/ttyUSB0")
    return ports[0].device


def apply_h(h, x, y):
    u = h[0][0] * x + h[0][1] * y + h[0][2]
    v = h[1][0] * x + h[1][1] * y + h[1][2]
    w = h[2][0] * x + h[2][1] * y + h[2][2]
    return (u / w, v / w) if w else (float("nan"), float("nan"))


def render(trail, extent, info, stats):
    x0, x1, y0, y1 = extent
    grid = [[" "] * PLOT_W for _ in range(PLOT_H)]
    for age, (px, py) in enumerate(trail):
        cx = int((px - x0) / max(x1 - x0, 1e-6) * (PLOT_W - 1))
        cy = int((1 - (py - y0) / max(y1 - y0, 1e-6)) * (PLOT_H - 1))
        if 0 <= cx < PLOT_W and 0 <= cy < PLOT_H:
            fresh = age == len(trail) - 1
            grid[cy][cx] = "O" if fresh else ("o" if age > len(trail) - 8 else ".")

    out = [f"{CSI}H{CSI}J", f"{CSI}1m{info}{CSI}0m\n", f"{stats}\n\n"]
    out.append("   " + "-" * PLOT_W + "\n")
    for row in grid:
        out.append("  |" + "".join(row) + "|\n")
    out.append("   " + "-" * PLOT_W + "\n")
    out.append(f"   {x0:.0f}{' ' * (PLOT_W - 12)}{x1:.0f}   (vertical {y0:.0f}..{y1:.0f})\n")
    out.append("   Ctrl-C to quit, 'b' on the board's serial to relearn background\n")
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("-H", "--homography", help="homography.json from calibrate_homography.py")
    ap.add_argument("--extent", type=float, nargs=4, metavar=("X0", "X1", "Y0", "Y1"),
                    help="plot bounds; default is pixel space, or auto-fit in mm")
    args = ap.parse_args()

    h = json.load(open(args.homography))["H"] if args.homography else None
    unit = "mm" if h else "px"
    extent = tuple(args.extent) if args.extent else ((0, 640, 480, 0) if not h else None)

    port = pick_port(args.port)
    trail = deque(maxlen=TRAIL)
    misses = hits = 0
    last = time.monotonic()
    fps = 0.0
    boot = ""

    with serial.Serial(port, args.baud, timeout=2) as ser:
        sys.stdout.write(f"{CSI}?25l")
        print(f"listening on {port} ...")
        try:
            while True:
                line = ser.readline().decode("utf-8", "replace").strip()
                if not line:
                    continue
                tag, *rest = line.split()

                if tag in ("I", "B"):
                    boot = line
                    print(line)
                    continue
                if tag == "N":
                    misses += 1
                    continue
                if tag != "P" or len(rest) < 5:
                    print(line)
                    continue

                xpx, ybase, w, score, z = (float(rest[0]), int(rest[1]),
                                           int(rest[2]), int(rest[3]), float(rest[4]))
                if ybase < 0:
                    misses += 1
                    continue
                hits += 1
                now = time.monotonic()
                fps = 0.8 * fps + 0.2 / max(now - last, 1e-6)
                last = now

                pt = apply_h(h, xpx, ybase) if h else (xpx, ybase)
                trail.append(pt)

                ext = extent
                if ext is None:
                    xs = [p[0] for p in trail] or [0]
                    ys = [p[1] for p in trail] or [0]
                    pad = 200
                    ext = (min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad)

                info = boot or f"ESP32-CAM pole tracker  {port}"
                stats = (f"pos ({pt[0]:7.1f}, {pt[1]:7.1f}) {unit}   "
                         f"width {w} px -> z~{z:.0f} mm   score {score}   "
                         f"{fps:4.1f} fps   hits {hits} misses {misses}")
                render(trail, ext, info, stats)
        except KeyboardInterrupt:
            pass
        finally:
            sys.stdout.write(f"{CSI}?25h\n")


if __name__ == "__main__":
    main()
