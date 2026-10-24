"""Run mpy/detector.py under CPython against synthetic QVGA scenes."""
import math
import sys

import numpy as np

sys.path.insert(0, ".")
from detector import Detector

W, H = 320, 240
FOCAL = W / (2 * math.tan(math.radians(30)))   # ~60 deg HFOV
DIA = 33.0
rng = np.random.default_rng(0)


def scene(Z, xc, contrast, noise=6.0, shadow=True, pole=True):
    y, x = np.mgrid[0:H, 0:W]
    img = 110 + 30 * np.sin(x / 48.0) * np.cos(y / 65.0) + 20 * np.sin(y / 26.0)
    for xe in (30, 150, 290):                    # static clutter edges
        img[:, xe:xe + 20] += 35
    if pole:
        w = FOCAL * DIA / Z
        h = min(FOCAL * 1000 / Z, H - 1)
        if shadow:
            img[int(H * 0.7):, int(xc + w):int(xc + w + 35)] -= 18
        cols = np.arange(W)
        cov = np.clip(np.minimum(cols + 1, xc + w / 2) - np.maximum(cols, xc - w / 2), 0, None)
        y0 = int(H - 1 - h)
        img[y0:, :] = img[y0:, :] * (1 - cov) + (img[y0:, :] + contrast) * cov
    return np.clip(img + rng.normal(0, noise, (H, W)), 0, 255).astype(np.uint8).tobytes()


def run(row_step=1):
    d = Detector(W, H, FOCAL, DIA, 500, 3000, row_step=row_step)
    print(f"QVGA {W}x{H}  focal {FOCAL:.0f} px  width window {d.wmin}..{d.wmax} px"
          f"  row_step {d.row_step}")

    empties = [scene(0, 0, 0, pole=False) for _ in range(8)]
    assert d.learn(empties), "learn failed"
    for b in empties[:4]:
        d.set_floor_from(b)
    print(f"background learned, detection floor {d.floor}")

    print("\n   Z   contrast   detected_x   true_x    err_px    err_mm   width  est_Z  base")
    bad = 0
    for Z in (600, 1200, 2000):
        for contrast in (60, 25, 12, -25):
            for xc in (80, 160, 240):
                r = d.detect(scene(Z, xc, contrast))
                mmpp = Z / FOCAL
                if not r["hit"]:
                    print(f"{Z:5d} {contrast:+6d}      MISS (score {r['score']} <= {r['thresh']})")
                    bad += 1
                    continue
                err = r["x"] - xc
                if abs(err) > 3:
                    bad += 1
                print(f"{Z:5d} {contrast:+6d}   {r['x']:9.2f} {xc:8d} {err:+9.2f} {err*mmpp:+9.1f}"
                      f"   {r['w']:4d} {r['z']:6.0f} {r['base']:5d}")

    r = d.detect(scene(0, 0, 0, pole=False))
    print(f"\nempty scene -> hit={r['hit']} (score {r['score']} vs thresh {r['thresh']})")
    if r["hit"]:
        bad += 1

    print("\nFAILURES:", bad)
    return 1 if bad else 0

if __name__ == "__main__":
    # row_step 1 is the viper path; 4 is the fallback used when the firmware
    # has no native emitter -- run both so the sensitivity cost is visible.
    rc = 0
    for step in (1, 4):
        print("=" * 72)
        rc |= run(step)
    sys.exit(rc)
