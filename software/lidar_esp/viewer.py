#!/usr/bin/env python3
"""Live terminal heatmap for the ESP32 + VL53L5CX depth matrix.

    python3 viewer.py                 # auto-detect the board
    python3 viewer.py -p /dev/ttyACM0 --max 2000

Reads the "INFO {...}" and "F <n> <d0> ..." lines emitted by main.py.
"""

import argparse
import json
import sys
import time

import serial
from serial.tools import list_ports

# blue (near) -> cyan -> green -> yellow -> red (far), xterm-256 cube
RAMP = [21, 27, 33, 39, 45, 51, 50, 48, 46, 82, 118, 154, 190, 226, 220, 214, 208, 202, 196]
CSI = "\x1b["


def pick_port(explicit):
    if explicit:
        return explicit
    ports = [p for p in list_ports.comports() if p.vid is not None]
    if not ports:
        sys.exit("no serial device found; pass -p /dev/ttyACM0")
    return ports[0].device


def color(mm, lo, hi):
    if mm < 0:
        return 236  # dark grey for invalid zones
    f = (min(max(mm, lo), hi) - lo) / max(hi - lo, 1)
    return RAMP[int(f * (len(RAMP) - 1))]


def orient(grid, n, rot, raw, shift=0, flip_x=False, flip_y=False):
    """Map sensor zone order to what you see standing behind the sensor.

    Zone 0 sits at the BOTTOM LEFT of the SPAD array and the lens flips the
    scene both horizontally and vertically, so zone 0 is lit by a target at the
    TOP RIGHT (datasheet 7.1.3).  Printing zones in index order puts the array's
    bottom row first, which already undoes the vertical flip -- only the
    horizontal mirror is left.

    Everything after that is empirical: --shift for a zone-index offset,
    --rot/--flip-x/--flip-y for how the module is physically mounted.
    """
    g = grid
    if shift:
        shift %= len(g)
        g = g[shift:] + g[:shift]       # circular shift in zone-index order
    if raw:
        return g
    g = [g[r * n + (n - 1 - c)] for r in range(n) for c in range(n)]
    for _ in range((rot // 90) % 4):    # rotate 90 deg clockwise, rot times
        g = [g[(n - 1 - c) * n + r] for r in range(n) for c in range(n)]
    if flip_x:
        g = [g[r * n + (n - 1 - c)] for r in range(n) for c in range(n)]
    if flip_y:
        g = [g[(n - 1 - r) * n + c] for r in range(n) for c in range(n)]
    return g


def render(grid, n, lo, hi, info, frame, fps, orientation="", probe=None):
    out = [f"{CSI}H{CSI}J"]  # home + clear
    if info:
        out.append(
            f"{CSI}1m{info.get('board', '?')}{CSI}0m  "
            f"MicroPython {info.get('micropython', '?')}  "
            f"uid {info.get('uid', '?')}  "
            f"{info.get('cpu_hz', 0) // 1_000_000} MHz  "
            f"{info.get('mem_free', 0) // 1024} KiB free\n"
            f"{info.get('sensor', '')} {n}x{n} @ {info.get('hz', '?')} Hz  "
            f"{info.get('i2c', '')}\n"
        )
    out.append(f"frame {frame}  {fps:4.1f} fps  scale {lo}-{hi} mm  {orientation}\n\n")

    for row in range(n):
        cells = []
        for col in range(n):
            mm = grid[row * n + col]
            label = "  ·  " if mm < 0 else f"{mm:5d}"
            cells.append(f"{CSI}48;5;{color(mm, lo, hi)}m{CSI}38;5;255m{label}{CSI}0m")
        out.append("  " + "".join(cells) + "\n")

    valid = [v for v in grid if v >= 0]
    out.append(
        f"\n  valid {len(valid)}/{len(grid)} zones"
        + (f"   min {min(valid)} mm   max {max(valid)} mm" if valid else "")
        + "\n"
        + (f"  nearest: raw zone {probe[0]} (row {probe[1]}, col {probe[2]})"
           f" at {probe[3]} mm\n" if probe else "")
        + "  Ctrl-C to quit\n"
    )
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--min", type=int, default=0, help="near end of colour scale, mm")
    ap.add_argument("--max", type=int, default=4000, help="far end of colour scale, mm")
    ap.add_argument("--rot", type=int, default=0, choices=[0, 90, 180, 270],
                    help="extra clockwise rotation for how the module is mounted")
    ap.add_argument("--raw", action="store_true",
                    help="show raw sensor zone order (zone 0 first, lens flip not undone)")
    # 4 measured on this board with --probe: the sensor's zone data comes back
    # offset by 4 zones (half a row).  Pass --shift 0 to see it uncorrected.
    ap.add_argument("--shift", type=int, default=4,
                    help="circular shift in zone index order, to cancel a data offset "
                         "(default 4, measured on this hardware)")
    ap.add_argument("--flip-x", action="store_true", help="mirror left/right")
    ap.add_argument("--flip-y", action="store_true", help="mirror top/bottom")
    ap.add_argument("--probe", action="store_true",
                    help="show the raw zone index of the nearest zone (mapping aid)")
    args = ap.parse_args()

    orientation = ("raw zone order" if args.raw else f"scene view, rot {args.rot}") + \
        (f", shift {args.shift}" if args.shift else "") + \
        (", flip-x" if args.flip_x else "") + (", flip-y" if args.flip_y else "")
    port = pick_port(args.port)
    info, frame, last, fps = None, 0, time.monotonic(), 0.0

    with serial.Serial(port, args.baud, timeout=2) as ser:
        sys.stdout.write(f"{CSI}?25l")  # hide cursor
        print(f"listening on {port} ...")
        try:
            while True:
                line = ser.readline().decode("utf-8", "replace").strip()
                if not line:
                    continue
                if line.startswith("INFO "):
                    info = json.loads(line[5:])
                elif line.startswith("F "):
                    parts = line.split()
                    n = int(parts[1])
                    grid = [int(v) for v in parts[2:]]
                    if len(grid) != n * n:
                        continue
                    frame += 1
                    now = time.monotonic()
                    fps = 0.8 * fps + 0.2 / max(now - last, 1e-6)
                    last = now
                    view = orient(grid, n, args.rot, args.raw, args.shift,
                                   args.flip_x, args.flip_y)
                    probe = None
                    if args.probe:
                        valid = [(v, i) for i, v in enumerate(grid) if v >= 0]
                        if valid:
                            mm, zone = min(valid)
                            probe = (zone, zone // n, zone % n, mm)
                    render(view, n, args.min, args.max, info, frame, fps,
                           orientation, probe)
                else:
                    print(line)  # tracebacks and other board output
        except KeyboardInterrupt:
            pass
        finally:
            sys.stdout.write(f"{CSI}?25h\n")


if __name__ == "__main__":
    main()
