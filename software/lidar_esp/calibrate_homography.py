#!/usr/bin/env python3
"""Solve the image -> floor-plane homography for the ESP32-CAM pole tracker.

The firmware reports the pole's base pixel (x, y_base).  Because the base is
always on the floor, one 3x3 homography maps that pixel to floor coordinates in
mm -- no depth sensing needed.  Needs 4+ non-collinear samples; use 6-8 spread
over the working area for a decent fit.

    python3 calibrate_homography.py              # live capture from the board
    python3 calibrate_homography.py --points pts.csv   # offline: x_px,y_px,X_mm,Y_mm

Writes homography.json, used by track_viewer.py.
"""

import argparse
import json
import sys
import time

import numpy as np


def solve(src, dst):
    """Direct linear transform. src/dst are Nx2 arrays. Returns 3x3 H."""
    a = []
    for (x, y), (u, v) in zip(src, dst):
        a.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        a.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, vt = np.linalg.svd(np.asarray(a, float))
    h = vt[-1].reshape(3, 3)
    return h / h[2, 2]


def apply(h, pts):
    p = np.hstack([np.asarray(pts, float), np.ones((len(pts), 1))])
    q = p @ h.T
    return q[:, :2] / q[:, 2:3]


def residuals(h, src, dst):
    err = apply(h, src) - np.asarray(dst, float)
    d = np.hypot(err[:, 0], err[:, 1])
    return d


def capture_live(port, baud, samples=10, timeout=8.0):
    import serial
    src, dst = [], []
    with serial.Serial(port, baud, timeout=1) as ser:
        print("Place the pole at a known floor position, type 'X Y' in mm and press\n"
              "Enter to capture. Blank line when done.\n")
        while True:
            try:
                raw = input(f"[{len(src)} captured] floor X Y (mm) > ").strip()
            except EOFError:
                break
            if not raw:
                break
            try:
                X, Y = (float(v) for v in raw.replace(",", " ").split())
            except ValueError:
                print("  need two numbers, e.g. '300 1200'")
                continue

            # Average a few detections to beat per-frame noise -- but never block
            # silently: report what the board is actually sending and give up.
            got, tally = [], {}
            ser.reset_input_buffer()
            deadline = time.monotonic() + timeout
            while len(got) < samples and time.monotonic() < deadline:
                line = ser.readline().decode("utf-8", "replace").split()
                if not line:
                    continue
                tag = line[0]
                tally[tag] = tally.get(tag, 0) + 1
                if tag == "P" and len(line) >= 4 and int(line[2]) >= 0:
                    got.append((float(line[1]), float(line[2])))

            if len(got) < samples:
                summary = ", ".join(f"{k}:{v}" for k, v in sorted(tally.items())) or "nothing"
                print(f"  only {len(got)}/{samples} detections in {timeout:.0f}s "
                      f"(lines seen: {summary}) -- not captured.")
                if not tally:
                    print("  The board is silent. Check the port and that --baud matches")
                    print("  SERIAL_BAUD in the sketch (921600).")
                elif tally.get("N"):
                    print("  The board is running but detects nothing. Diagnose with:")
                    print("    python3 preview.py status     # score vs threshold")
                    print("    python3 preview.py frame      # is the image sane?")
                    print("    python3 preview.py profiles   # does the pole show up?")
                continue

            px = np.mean(got, axis=0)
            src.append(px.tolist())
            dst.append([X, Y])
            print(f"  pixel ({px[0]:.1f}, {px[1]:.1f})  spread "
                  f"{np.std(got, axis=0).round(2).tolist()} px")
    return src, dst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("--points", help="CSV x_px,y_px,X_mm,Y_mm instead of live capture")
    ap.add_argument("-o", "--out", default="homography.json")
    args = ap.parse_args()

    if args.points:
        data = np.loadtxt(args.points, delimiter=",", ndmin=2)
        src, dst = data[:, :2].tolist(), data[:, 2:4].tolist()
    else:
        src, dst = capture_live(args.port, args.baud)

    if len(src) < 4:
        sys.exit(f"need at least 4 points, got {len(src)}")

    h = solve(src, dst)
    d = residuals(h, src, dst)
    print(f"\nfit over {len(src)} points: mean {d.mean():.1f} mm, max {d.max():.1f} mm")
    if d.max() > 50:
        print("  warning: >50 mm residual -- check for a mistyped coordinate,")
        print("  collinear points, or a camera that moved between samples")

    json.dump({"H": h.tolist(), "points": {"src": src, "dst": dst},
               "residual_mm": {"mean": d.mean(), "max": d.max()}},
              open(args.out, "w"), indent=2)
    print(f"wrote {args.out}")
    print("\nC array, if you prefer to do the mapping on the board:")
    print("static const float H[9] = {" +
          ", ".join(f"{v:.8g}f" for v in h.ravel()) + "};")


if __name__ == "__main__":
    main()
