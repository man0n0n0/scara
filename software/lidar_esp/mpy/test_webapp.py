"""Exercise webapp.py + detector.py + homography.py under CPython with a fake camera."""
import asyncio, json, math, sys, urllib.request, urllib.error
import numpy as np
sys.path.insert(0, ".")
import homography as hg
from detector import Detector
from webapp import Server
from test_detector import scene, W, H, FOCAL, DIA

PORT = 8123

class FakeState:
    def __init__(self):
        self.w, self.h = W, H
        self.det = Detector(W, H, FOCAL, DIA, 500, 3000)
        self.det.learn([scene(0, 0, 0, pole=False) for _ in range(6)])
        for _ in range(3):
            self.det.set_floor_from(scene(0, 0, 0, pole=False))
        self.pole_x, self.Z = 160, 1200
        self._frame = scene(self.Z, self.pole_x, 50)
        self.last = self.det.detect(self._frame)
        self.calib = {"H": None, "src": [], "dst": []}
        self.xy = None
        self.fps = 7.5
    def snapshot(self): return self._frame
    def as_dict(self):
        d = dict(self.last); d["xy"] = self.xy; d["fps"] = self.fps
        d["w_img"], d["h_img"] = self.w, self.h
        d["width_window"] = [self.det.wmin, self.det.wmax]
        d["have_bg"] = self.det.have_bg; d["settings"] = {}
        d["calib_points"] = len(self.calib["src"])
        d["calibrated"] = bool(self.calib["H"]); d["mem_free"] = 123456
        d["frames"] = 1
        return d
    async def relearn(self): return True
    def apply_settings(self, cfg): return cfg
    # Points come from a known ground-truth homography (+1 px noise) so the
    # residual reported through the API is meaningful. The detector itself is
    # exercised by /state above and by test_detector.py.
    HTRUE = np.array([[1.9, 0.35, -420.0], [0.05, 0.6, 980.0], [2e-5, 9e-4, 1.0]])
    def calib_add(self, X, Y, samples=8):
        q = np.linalg.inv(self.HTRUE) @ np.array([X, Y, 1.0])
        px = [q[0] / q[2] + np.random.normal(0, 1), q[1] / q[2] + np.random.normal(0, 1)]
        self.calib["src"].append(px); self.calib["dst"].append([X, Y])
        return {"ok": True, "px": px, "n": len(self.calib["src"]), "samples": samples}
    def calib_solve(self):
        src, dst = self.calib["src"], self.calib["dst"]
        if len(src) < 4: return {"ok": False, "msg": "need 4"}
        h = hg.solve(src, dst); res = hg.residuals(h, src, dst)
        self.calib["H"] = h
        return {"ok": True, "H": h, "n": len(src),
                "residual_mean": sum(res)/len(res), "residual_max": max(res)}
    def calib_clear(self): self.calib = {"H": None, "src": [], "dst": []}
    def calib_undo(self):
        for k in ("src","dst"): self.calib[k] and self.calib[k].pop()

def get(path):
    with urllib.request.urlopen(f"http://127.0.0.1:{PORT}{path}", timeout=5) as r:
        return r.status, r.headers, r.read()

def post(path, obj):
    req = urllib.request.Request(f"http://127.0.0.1:{PORT}{path}",
                                 data=json.dumps(obj).encode(), method="POST")
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())

async def main():
    st = FakeState()
    srv = Server(st, PORT)
    await srv.start()
    loop = asyncio.get_event_loop()
    fails = []

    s, h, b = await loop.run_in_executor(None, get, "/")
    print(f"GET /          {s} {h['Content-Type']} {len(b)} bytes")
    if s != 200 or b"<canvas" not in b: fails.append("index")

    s, h, b = await loop.run_in_executor(None, get, "/raw")
    print(f"GET /raw       {s} {h['X-Width']}x{h['X-Height']} {len(b)} bytes")
    if len(b) != W * H: fails.append("raw size")

    s, h, b = await loop.run_in_executor(None, get, "/state")
    j = json.loads(b)
    print(f"GET /state     hit={j['hit']} x={j['x']:.2f} base={j['base']} "
          f"score={j['score']}/{j['thresh']}")
    if not j["hit"]: fails.append("no detection through API")

    s, h, b = await loop.run_in_executor(None, get, "/profiles")
    p = json.loads(b)
    print(f"GET /profiles  step={p['step']} n={len(p['dpos'])} peak={max(p['dpos'])}")
    if len(p["dpos"]) < 50: fails.append("profiles")

    # calibration round trip: 5 poses on a known floor mapping
    truth = [(0, 800), (600, 800), (0, 1600), (600, 1600), (300, 1200)]
    for i, (X, Y) in enumerate(truth):
        st.pole_x = 60 + i * 45
        st.Z = 900 + i * 180
        r = await loop.run_in_executor(None, post, "/api/calib/add", {"X": X, "Y": Y})
        if not r["ok"]: fails.append(f"calib add {i}: {r}")
    r = await loop.run_in_executor(None, post, "/api/calib/solve", {})
    print(f"POST solve     ok={r['ok']} n={r.get('n')} "
          f"mean={r.get('residual_mean',0):.1f} mm max={r.get('residual_max',0):.1f} mm")
    if not r["ok"]: fails.append("solve")
    elif r["residual_max"] > 60: fails.append(f"residual too high: {r['residual_max']:.0f} mm")

    r = await loop.run_in_executor(None, post, "/api/calib/undo", {})
    print(f"POST undo      n={r['n']}")
    def get404():
        try:
            return get("/nope")[0]
        except urllib.error.HTTPError as e:
            return e.code
    code = await loop.run_in_executor(None, get404)
    print(f"GET /nope      {code}")
    if code != 404: fails.append("404 handling")

    print("\nFAILURES:", fails if fails else "none")
    return 1 if fails else 0

sys.exit(asyncio.run(main()))
