"""Find out why Camera() fails, by trying every plausible configuration.

Run it on the board INSTEAD of main.py:

    mpremote connect /dev/ttyUSB0 run camprobe.py

ESP_ERR_NOT_SUPPORTED / EOPNOTSUPP from Camera() has exactly two sources in
the esp32-camera driver: a JPEG request on a sensor without JPEG support, or
"Detected camera not supported" -- meaning no sensor answered on the SCCB bus
at any known address.  We never ask for JPEG, so it is always the second: the
sensor was not found.  That is wiring, clock or power, never pixel format.
"""

from camera import Camera, PixelFormat, FrameSize, GrabMode

from boards import BOARDS

XCLKS = (20_000_000, 10_000_000, 8_000_000)


def attempt(name, pins, xclk):
    kw = dict(pins)
    kw["xclk_freq"] = xclk
    kw.update(pixel_format=PixelFormat.GRAYSCALE,
              frame_size=FrameSize.QVGA,
              grab_mode=GrabMode.WHEN_EMPTY,
              fb_count=1)
    cam = None
    try:
        cam = Camera(**kw)
        img = cam.capture()
        n = len(img) if img else 0
        return True, "OK, captured %d bytes" % n
    except Exception as e:                      # noqa: BLE001 -- report anything
        return False, "%s: %s" % (type(e).__name__, e)
    finally:
        if cam is not None:
            try:
                cam.deinit()
            except Exception:
                pass


def main():
    print("module exports:", [n for n in dir(Camera) if not n.startswith("_")])
    print()
    hits = []
    for name, pins in BOARDS.items():
        for xclk in XCLKS:
            ok, msg = attempt(name, pins, xclk)
            print("%-18s %2d MHz  %s" % (name, xclk // 1_000_000, msg))
            if ok:
                hits.append((name, xclk))
                break               # this map works, no need for slower clocks
    print()
    if hits:
        for name, xclk in hits:
            print("WORKS: %s at %d MHz" % (name, xclk // 1_000_000))
        print("Put that name in BOARD and that frequency in XCLK_HZ in main.py.")
    else:
        print("Nothing answered on the SCCB bus. In order of likelihood:")
        print("  1. power -- the OV2640 needs 5 V with real current headroom;")
        print("     a USB-serial adapter's 3.3 V pin will not probe reliably")
        print("  2. the ribbon connector is not fully seated / latch not closed")
        print("  3. the camera module itself is dead (try another OV2640)")
        print("  4. a board whose pin map is not in boards.py")


main()
