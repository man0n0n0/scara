#!/usr/bin/env python3
"""Pull frames from the capture firmware, or from the running tracker, into a dataset folder.

Works with either firmware, detected automatically:
  capture firmware: /capture (one fresh JPEG) and /status (camera settings)
  tracker firmware: /snapshot.jpg (last frame the model ran on) and /cam
                    (camera settings). The tracker serves only ~2 frames/s and
                    the same frame twice is skipped, so --every below ~0.5 s
                    brings nothing.

Each run is one "session": one lighting condition / background / camera height.
Keep sessions separate. To test detectability honestly, hold one whole lighting
session out as the Edge Impulse test set instead of a random split.

  # one frame every 0.5 s until Ctrl-C
  ./grab_frames.py 192.168.0.42 --lighting spot_warm --every 0.5

  # press Enter for each frame, q + Enter to stop
  ./grab_frames.py 192.168.0.42 --lighting daylight --manual

  # negatives (no wormbot in view) help FOMO against glare and reflections
  ./grab_frames.py 192.168.0.42 --lighting spot_warm --negatives --count 100

Then upload: edge-impulse-uploader --category split dataset/<session>/*.jpg
(or the Studio's Data acquisition > Upload), and label boxes in Studio.
Standard library only.
"""

import argparse
import json
import sys
import time
import urllib.request
from datetime import datetime
from pathlib import Path


def fetch(url, timeout=5.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read(), dict(r.headers)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", help="IP or hostname of the XIAO (capture or tracker firmware)")
    ap.add_argument("--out", default="dataset", help="dataset root (default: dataset/)")
    ap.add_argument("--lighting", required=True, help="tag for the lighting condition, e.g. spot_warm, led_cold, dim")
    ap.add_argument("--session", help="session folder name (default: <lighting>_<timestamp>)")
    ap.add_argument("--note", default="", help="free text stored in meta.json (height, background...)")
    ap.add_argument("--negatives", action="store_true", help="frames without the wormbot (prefix 'neg')")
    ap.add_argument("--every", type=float, default=1.0, help="seconds between frames (default 1.0)")
    ap.add_argument("--count", type=int, default=0, help="stop after N frames (default: until Ctrl-C)")
    ap.add_argument("--manual", action="store_true", help="grab on Enter instead of a timer")
    a = ap.parse_args()

    base = f"http://{a.host}"
    try:  # the tracker has /cam; the capture firmware has camera settings on /status
        settings = json.loads(fetch(f"{base}/cam")[0])
        firmware, frame_url = "tracker", f"{base}/snapshot.jpg"
    except Exception:
        try:
            settings = json.loads(fetch(f"{base}/status")[0])
            firmware, frame_url = "capture", f"{base}/capture"
        except Exception as e:
            sys.exit(f"cannot reach the XIAO at {base}: {e}")

    session = a.session or f"{a.lighting}_{datetime.now():%Y%m%d_%H%M%S}"
    folder = Path(a.out) / session
    folder.mkdir(parents=True, exist_ok=True)
    meta = {
        "session": session,
        "lighting": a.lighting,
        "negatives": a.negatives,
        "note": a.note,
        "started": datetime.now().isoformat(timespec="seconds"),
        "firmware": firmware,
        "camera": settings,
    }
    (folder / "meta.json").write_text(json.dumps(meta, indent=2))
    print(f"{firmware} firmware - saving to {folder}/  camera settings: {settings}")
    if firmware == "capture" and settings.get("framesize") != 5:
        print("note: framesize is not QVGA (5). Fine if the tracker uses the same setting (it loads it from NVS).")

    prefix = "neg" if a.negatives else "wormbot"
    n = 0
    sizes = set()
    last = None
    try:
        while not a.count or n < a.count:
            if a.manual:
                if input(f"[{n}] Enter = grab, q = quit: ").strip().lower() == "q":
                    break
            t0 = time.monotonic()
            try:
                jpg, hdr = fetch(frame_url)
            except Exception as e:
                print(f"capture failed: {e}")
                time.sleep(1)
                continue
            if jpg == last:  # tracker: no new frame since the last grab
                time.sleep(0.1)
                continue
            last = jpg
            sizes.add(hdr.get("X-Frame-Size", "?"))
            name = folder / f"{prefix}.{session}.{n:04d}.jpg"
            name.write_bytes(jpg)
            n += 1
            print(f"\r{n} frames ({len(jpg) // 1024} kB)", end="", flush=True)
            if not a.manual:
                time.sleep(max(0.0, a.every - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    print()
    meta["frames"] = n
    meta["frame_sizes"] = sorted(sizes)
    meta["ended"] = datetime.now().isoformat(timespec="seconds")
    (folder / "meta.json").write_text(json.dumps(meta, indent=2))
    print(f"{n} frames in {folder}")


if __name__ == "__main__":
    main()
