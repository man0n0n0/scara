"""Image -> floor-plane homography, solved on the board (no numpy).

Least squares via normal equations (A^T A) h = A^T b, 8x8 Gaussian elimination
with partial pivoting. Exact for 4 points, least-squares for more.
"""

import json


def solve(src, dst):
    """src: [(x_px, y_px), ...]  dst: [(X_mm, Y_mm), ...]  -> 9 floats, h[8]=1."""
    n = len(src)
    if n < 4:
        raise ValueError("need at least 4 points")
    ata = [[0.0] * 8 for _ in range(8)]
    atb = [0.0] * 8
    for (x, y), (u, v) in zip(src, dst):
        for row, rhs in (([x, y, 1, 0, 0, 0, -u * x, -u * y], u),
                         ([0, 0, 0, x, y, 1, -v * x, -v * y], v)):
            for i in range(8):
                atb[i] += row[i] * rhs
                for j in range(8):
                    ata[i][j] += row[i] * row[j]

    for col in range(8):                      # Gaussian elimination, partial pivot
        p = max(range(col, 8), key=lambda r: abs(ata[r][col]))
        if abs(ata[p][col]) < 1e-12:
            raise ValueError("degenerate points (collinear?)")
        if p != col:
            ata[col], ata[p] = ata[p], ata[col]
            atb[col], atb[p] = atb[p], atb[col]
        piv = ata[col][col]
        for r in range(col + 1, 8):
            f = ata[r][col] / piv
            if f:
                for c in range(col, 8):
                    ata[r][c] -= f * ata[col][c]
                atb[r] -= f * atb[col]
    h = [0.0] * 8
    for r in range(7, -1, -1):
        acc = atb[r]
        for c in range(r + 1, 8):
            acc -= ata[r][c] * h[c]
        h[r] = acc / ata[r][r]
    return h + [1.0]


def apply(h, x, y):
    w = h[6] * x + h[7] * y + h[8]
    if not w:
        return (0.0, 0.0)
    return ((h[0] * x + h[1] * y + h[2]) / w,
            (h[3] * x + h[4] * y + h[5]) / w)


def residuals(h, src, dst):
    out = []
    for (x, y), (u, v) in zip(src, dst):
        px, py = apply(h, x, y)
        out.append(((px - u) ** 2 + (py - v) ** 2) ** 0.5)
    return out


def load(path="calib.json"):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"H": None, "src": [], "dst": []}


def save(data, path="calib.json"):
    with open(path, "w") as f:
        json.dump(data, f)
