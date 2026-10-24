"""Pole detector: per-column gradient profiles, 1-D background, matched filter.

The hot loops ask for @micropython.viper (raw pointer access, no bytecode
dispatch).  Not every firmware ships the native emitter -- a build without it
has no micropython.viper at all -- so the decorator degrades: viper, else
native, else plain bytecode.  ACCEL says which one is live, and row_step below
trades vertical sampling for speed when it is the slow one.  The same code runs
on CPython for tests -- see test_detector.py.
"""

ptr8 = ptr32 = int        # viper-only annotation names, never used as types

try:
    import micropython
    accel = getattr(micropython, "viper", None)
    ACCEL = "viper"
    if accel is None:                    # firmware built without the emitter
        accel = getattr(micropython, "native", None)
        ACCEL = "native"
    if accel is None:
        ACCEL = "bytecode"
except ImportError:                      # running on CPython, for tests
    accel = None
    ACCEL = "cpython"

if accel is None:
    def accel(f):
        return f

# Rows to skip in the column profiles.  The pole is vertical and spans the
# frame, so sampling every Nth row costs almost no signal -- but it is the only
# lever that matters when the loop is interpreted.
DEFAULT_ROW_STEP = 1 if ACCEL in ("viper", "cpython") else 4

from array import array


@accel
def accumulate(buf: ptr8, w: int, h: int, pos: ptr32, neg: ptr32, step: int):
    """Sum signed horizontal gradient down each column. ~6 ops/sampled pixel."""
    i = 0
    while i < w:
        pos[i] = 0
        neg[i] = 0
        i += 1
    y = 0
    while y < h:
        row = y * w
        x = 0
        prev = int(buf[row])
        while x < w - 1:
            cur = int(buf[row + x + 1])
            g = cur - prev
            if g > 0:
                pos[x] = int(pos[x]) + g
            else:
                neg[x] = int(neg[x]) - g
            prev = cur
            x += 1
        y += step


@accel
def subtract_bg(pos: ptr32, neg: ptr32, bgp: ptr32, bgn: ptr32,
                dpos: ptr32, dneg: ptr32, w: int):
    x = 0
    while x < w:
        a = int(pos[x]) - int(bgp[x])
        dpos[x] = a if a > 0 else 0
        b = int(neg[x]) - int(bgn[x])
        dneg[x] = b if b > 0 else 0
        x += 1


@accel
def match(dpos: ptr32, dneg: ptr32, w: int, wmin: int, wmax: int, out: ptr32):
    """Best edge pair `ww` apart, both polarities. out = [score, x, width, pol]."""
    best = 0
    bx = -1
    bw = 0
    bp = 0
    ww = wmin
    while ww <= wmax:
        lim = w - ww
        x = 0
        while x < lim:
            s = int(dpos[x]) + int(dneg[x + ww])
            if s > best:
                best = s
                bx = x
                bw = ww
                bp = 0
            s = int(dneg[x]) + int(dpos[x + ww])
            if s > best:
                best = s
                bx = x
                bw = ww
                bp = 1
            x += 1
        ww += 1
    out[0] = best
    out[1] = bx
    out[2] = bw
    out[3] = bp


@accel
def find_base(buf: ptr8, w: int, h: int, xl: int, xr: int, thresh: int, run_min: int) -> int:
    """Lowest row where both pole edges still show: where it meets the floor.

    Scans upward from the bottom and returns the FIRST such row, confirmed by
    run_min consecutive rows above it (so a speck of noise is not the base).
    """
    if xl < 1 or xr >= w - 1:
        return -1
    cand = -1
    run = 0
    y = h - 1
    while y >= 0:
        row = y * w
        a = int(buf[row + xl + 1]) - int(buf[row + xl - 1])
        if a < 0:
            a = -a
        b = int(buf[row + xr + 1]) - int(buf[row + xr - 1])
        if b < 0:
            b = -b
        if a + b > thresh:
            if cand < 0:
                cand = y
            run += 1
            if run >= run_min:
                return cand
        else:
            cand = -1
            run = 0
        y -= 1
    return cand


class Detector:
    def __init__(self, w, h, focal_px, dia_mm, z_min, z_max,
                 margin=1.8, bg_shift=6, row_step=None):
        self.w, self.h = w, h
        self.row_step = DEFAULT_ROW_STEP if row_step is None else max(1, int(row_step))
        self.focal_px, self.dia_mm = focal_px, dia_mm
        self.margin, self.bg_shift = margin, bg_shift
        self.wmin = max(2, int(focal_px * dia_mm / z_max))
        self.wmax = int(focal_px * dia_mm / z_min) + 1
        self.pos = array("i", [0] * w)
        self.neg = array("i", [0] * w)
        self.bgp = array("i", [0] * w)
        self.bgn = array("i", [0] * w)
        self.dpos = array("i", [0] * w)
        self.dneg = array("i", [0] * w)
        self.out = array("i", [0] * 4)
        self.floor = 0
        self.have_bg = False

    def profiles(self, buf):
        accumulate(buf, self.w, self.h, self.pos, self.neg, self.row_step)

    def learn(self, frames):
        """frames: iterable of buffers of an EMPTY scene."""
        w = self.w
        acc_p = [0] * w
        acc_n = [0] * w
        n = 0
        for buf in frames:
            self.profiles(buf)
            for x in range(w):
                acc_p[x] += self.pos[x]
                acc_n[x] += self.neg[x]
            n += 1
        if not n:
            return False
        for x in range(w):
            self.bgp[x] = acc_p[x] // n
            self.bgn[x] = acc_n[x] // n
        self.have_bg = True
        return True

    def set_floor_from(self, buf):
        """Residual score on an empty frame sets the detection floor."""
        self.profiles(buf)
        subtract_bg(self.pos, self.neg, self.bgp, self.bgn,
                    self.dpos, self.dneg, self.w)
        match(self.dpos, self.dneg, self.w, self.wmin, self.wmax, self.out)
        if self.out[0] > self.floor:
            self.floor = self.out[0]

    def drift(self):
        """Follow slow lighting changes while nothing is detected."""
        s = self.bg_shift
        for x in range(self.w):
            self.bgp[x] += (self.pos[x] - self.bgp[x]) >> s
            self.bgn[x] += (self.neg[x] - self.bgn[x]) >> s

    def refine_x(self, x, width):
        """Sub-pixel: energy centroid of both edges, +-2 px."""
        half = width * 0.5
        l, r = int(x - half), int(x + half)
        c = m = 0.0
        for i in range(-2, 3):
            for xi, off in ((l + i, -half), (r + i, half)):
                if 0 <= xi < self.w:
                    e = self.dpos[xi] + self.dneg[xi]
                    c += e * (xi - off)
                    m += e
        return c / m if m else x

    def detect(self, buf):
        """Returns dict, always with 'score'/'thresh'; 'hit' says if it counts."""
        self.profiles(buf)
        subtract_bg(self.pos, self.neg, self.bgp, self.bgn,
                    self.dpos, self.dneg, self.w)
        match(self.dpos, self.dneg, self.w, self.wmin, self.wmax, self.out)
        score, bx, bw, pol = self.out[0], self.out[1], self.out[2], self.out[3]
        thresh = int(self.floor * self.margin)
        res = {"score": score, "thresh": thresh, "floor": self.floor,
               "hit": False, "x": -1.0, "base": -1, "w": bw, "z": 0.0}
        if not self.have_bg or bx < 0 or score <= thresh:
            self.drift()
            return res
        xc = self.refine_x(bx + bw * 0.5, bw)
        base = find_base(buf, self.w, self.h,
                         int(xc - bw * 0.5), int(xc + bw * 0.5), 12, 5)
        res["hit"] = True
        res["x"] = xc
        res["base"] = base
        res["z"] = self.focal_px * self.dia_mm / bw if bw else 0.0
        return res
