"""Pole tracker on ESP32-CAM, MicroPython.

Needs a MicroPython build with the camera module:
  https://github.com/cnadler86/micropython-camera-API  (AI_THINKER variant)

Serves a web UI on http://192.168.4.1/ (its own AP by default).
"""

import asyncio
import gc
import math
import time

import network
from camera import Camera, PixelFormat, FrameSize, GrabMode

import homography as hg
from boards import BOARDS
from detector import ACCEL, Detector
from webapp import Server

# ---- configuration
AP_SSID, AP_PASS = "poletracker", "pole1234"   # >= 8 chars
STA_SSID, STA_PASS = None, None                # set both to join your own network

FRAME_W, FRAME_H = 320, 240                    # QVGA: VGA grayscale caps near 3 fps
HFOV_DEG = 60.0
TARGET_DIA_MM = 33.0
Z_MIN_MM, Z_MAX_MM = 500.0, 3000.0
LEARN_FRAMES = 12

# Board pin map and master clock.  camprobe.py sweeps both and tells you what
# this board actually answers to -- run it first if Camera() fails.
# ESP32-WROVER-DEV with camera (and the Freenove ESP32-Wrover CAM) use the
# ESP-WROVER-KIT map: XCLK on 21, no power-down line, and D0 on GPIO 4 --
# which on this board is a camera data pin, not the flash LED.
BOARD = "WROVER_KIT"
XCLK_HZ = 20_000_000

# fb_count must be 1 for a non-JPEG format: the driver says so itself, and
# GRAB_LATEST is meaningless with a single buffer.
FB_COUNT = 1


class State:
    def __init__(self):
        self.w, self.h = FRAME_W, FRAME_H
        self.focal_px = FRAME_W / (2 * math.tan(math.radians(HFOV_DEG / 2)))
        self.cam = None
        self.det = Detector(self.w, self.h, self.focal_px, TARGET_DIA_MM,
                            Z_MIN_MM, Z_MAX_MM)
        self.last = {"hit": False, "x": -1.0, "base": -1, "w": 0,
                     "z": 0.0, "score": 0, "thresh": 0, "floor": 0}
        self.xy = None
        self.fps = 0.0
        self.frames = 0
        self._frame = None            # bytes of the most recent capture
        self.busy = False             # pause the loop during learn/calibration
        self.calib = hg.load()
        self.locks = {}
        self.settings = {"exposure": 300, "gain": 4, "margin": 1.8,
                         "row_step": self.det.row_step}

    # ---- camera
    def camera_init(self):
        common = {"pixel_format": PixelFormat.GRAYSCALE,
                  "frame_size": FrameSize.QVGA,
                  "grab_mode": GrabMode.WHEN_EMPTY,
                  "fb_count": FB_COUNT}
        kw = dict(BOARDS[BOARD])
        kw["xclk_freq"] = XCLK_HZ
        kw.update(common)
        try:
            self.cam = Camera(**kw)
        except TypeError as e:
            # board-specific build: it rejects the pin keywords it compiled in
            print("camera: pins refused (%s), retrying without them" % e)
            self.cam = Camera(**common)
        except OSError as e:
            # ESP_ERR_NOT_SUPPORTED here means the driver found no sensor on
            # the SCCB bus -- the pixel format has nothing to do with it.
            raise OSError("%s -- no camera answered on the SCCB bus with the "
                          "%s pin map at %d MHz. Run camprobe.py to sweep the "
                          "pin maps and clock rates; if nothing answers it is "
                          "power or the ribbon connector, not the code."
                          % (e, BOARD, XCLK_HZ // 1_000_000))
        self.locks = self.lock_sensor()
        print("camera locks:", self.locks)
        self.apply_settings(self.settings)

    def lock_sensor(self):
        """Background subtraction cannot survive auto-exposure hunting.

        Property names differ between builds of the camera module, so each one
        is tried and the outcome reported -- a name that does not exist here
        must not kill the boot, but it must not pass unnoticed either.
        """
        done = {}
        for attr, val in (("whitebal", False), ("awb_gain", False),
                          ("exposure_ctrl", False), ("aec2", False),
                          ("gain_ctrl", False)):
            try:
                setattr(self.cam, attr, val)
                done[attr] = "off"
            except (AttributeError, OSError, TypeError) as e:
                done[attr] = "unsupported (%s)" % e
        return done

    def apply_settings(self, cfg):
        out = {}
        for key, attr in (("exposure", "aec_value"), ("gain", "agc_gain")):
            if key in cfg:
                try:
                    setattr(self.cam, attr, int(cfg[key]))
                    self.settings[key] = int(cfg[key])
                    out[key] = int(cfg[key])
                except (AttributeError, OSError, ValueError) as e:
                    out[key] = "error: %s" % e
        if "row_step" in cfg:
            # The background profiles are sums over the sampled rows, so they
            # are only valid for the step that produced them: force a relearn.
            self.det.row_step = max(1, int(cfg["row_step"]))
            self.det.have_bg = False
            self.det.floor = 0
            self.settings["row_step"] = self.det.row_step
            out["row_step"] = self.det.row_step
            out["note"] = "row_step changed -- background dropped, relearn now"
        if "margin" in cfg:
            self.det.margin = float(cfg["margin"])
            self.settings["margin"] = self.det.margin
            out["margin"] = self.det.margin
        return out

    def capture(self):
        img = self.cam.capture()
        if img is None:
            return None
        buf = bytes(img)          # copy out of the driver's framebuffer
        try:
            self.cam.free_buffer()
        except AttributeError:
            pass
        return buf

    def snapshot(self):
        return self._frame

    # ---- detection loop
    async def run(self):
        t0 = time.ticks_ms()
        n = 0
        while True:
            if self.busy:
                await asyncio.sleep_ms(50)
                continue
            buf = self.capture()
            if buf is None:
                await asyncio.sleep_ms(50)
                continue
            self._frame = buf
            res = self.det.detect(buf)
            self.last = res
            self.xy = None
            if res["hit"] and res["base"] >= 0 and self.calib.get("H"):
                self.xy = hg.apply(self.calib["H"], res["x"], res["base"])
            self.frames += 1
            n += 1
            if n >= 10:
                dt = time.ticks_diff(time.ticks_ms(), t0)
                self.fps = 10000.0 / dt if dt else 0.0
                t0 = time.ticks_ms()
                n = 0
                gc.collect()
            await asyncio.sleep_ms(0)      # let the server breathe

    # ---- actions
    async def relearn(self):
        self.busy = True
        await asyncio.sleep_ms(80)
        try:
            frames = []
            for _ in range(LEARN_FRAMES):
                b = self.capture()
                if b:
                    frames.append(b)
                await asyncio.sleep_ms(10)
            if not frames:
                return False
            ok = self.det.learn(frames)
            self.det.floor = 0
            for b in frames[:4]:
                self.det.set_floor_from(b)
            del frames
            gc.collect()
            return ok
        finally:
            self.busy = False

    def calib_add(self, X, Y, samples=8):
        pts = []
        for _ in range(samples * 3):
            buf = self.capture()
            if not buf:
                continue
            r = self.det.detect(buf)
            if r["hit"] and r["base"] >= 0:
                pts.append((r["x"], r["base"]))
            if len(pts) >= samples:
                break
        if len(pts) < max(3, samples // 2):
            return {"ok": False, "got": len(pts),
                    "msg": "not enough detections -- check the live view first"}
        x = sum(p[0] for p in pts) / len(pts)
        y = sum(p[1] for p in pts) / len(pts)
        self.calib.setdefault("src", []).append([x, y])
        self.calib.setdefault("dst", []).append([X, Y])
        hg.save(self.calib)
        return {"ok": True, "px": [x, y], "n": len(self.calib["src"]),
                "samples": len(pts)}

    def calib_solve(self):
        src, dst = self.calib.get("src", []), self.calib.get("dst", [])
        if len(src) < 4:
            return {"ok": False, "msg": "need at least 4 points, have %d" % len(src)}
        try:
            h = hg.solve(src, dst)
        except ValueError as e:
            return {"ok": False, "msg": str(e)}
        res = hg.residuals(h, src, dst)
        self.calib["H"] = h
        hg.save(self.calib)
        return {"ok": True, "H": h, "n": len(src),
                "residual_mean": sum(res) / len(res), "residual_max": max(res)}

    def calib_clear(self):
        self.calib = {"H": None, "src": [], "dst": []}
        hg.save(self.calib)

    def calib_undo(self):
        for k in ("src", "dst"):
            if self.calib.get(k):
                self.calib[k].pop()
        self.calib["H"] = None
        hg.save(self.calib)

    def as_dict(self):
        d = dict(self.last)
        d["xy"] = list(self.xy) if self.xy else None
        d["fps"] = round(self.fps, 2)
        d["frames"] = self.frames
        d["w_img"], d["h_img"] = self.w, self.h
        d["width_window"] = [self.det.wmin, self.det.wmax]
        d["have_bg"] = self.det.have_bg
        d["settings"] = self.settings
        d["calib_points"] = len(self.calib.get("src", []))
        d["calibrated"] = bool(self.calib.get("H"))
        d["mem_free"] = gc.mem_free()
        d["accel"] = ACCEL
        d["locks"] = self.locks
        d["board"] = BOARD
        return d


def wifi_up():
    if STA_SSID:
        sta = network.WLAN(network.STA_IF)
        sta.active(True)
        sta.connect(STA_SSID, STA_PASS)
        for _ in range(40):
            if sta.isconnected():
                print("wifi:", sta.ifconfig()[0])
                return sta.ifconfig()[0]
            time.sleep(0.5)
        print("wifi: STA failed, falling back to AP")
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=AP_SSID, password=AP_PASS)
    while not ap.active():
        time.sleep(0.2)
    print("AP:", AP_SSID, ap.ifconfig()[0])
    return ap.ifconfig()[0]


async def amain():
    st = State()
    print("hot loops:", ACCEL, "row_step", st.det.row_step)
    if ACCEL not in ("viper", "native"):
        print("WARNING: this firmware has no native emitter -- expect ~1 fps")
    ip = wifi_up()
    st.camera_init()
    print("camera ready, learning background (keep the scene empty)")
    await st.relearn()
    print("floor =", st.det.floor)
    srv = Server(st)
    await srv.start()
    print("open http://%s/" % ip)
    await st.run()


if __name__ == "__main__":        # the board runs this file as __main__
    try:
        asyncio.run(amain())
    finally:
        asyncio.new_event_loop()
