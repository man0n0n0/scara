// Direct linear transform with h33 = 1. Each pair gives two equations
//   h11 u + h12 v + h13 - x h31 u - x h32 v = x
//   h21 u + h22 v + h23 - y h31 u - y h32 v = y
// solved in the least-squares sense (normal equations, 8x8). Both point sets
// are first moved to their centroid and scaled to a mean distance of sqrt(2),
// which keeps the system well conditioned (image units ~1, arm units ~1000).

#include "homography.h"
#include <math.h>

// T maps p to (p - c) * s.
struct Norm { double cx, cy, s; };

static Norm normaliser(int n, const float* a, const float* b) {
    Norm t = {0, 0, 1};
    for (int i = 0; i < n; i++) t.cx += a[i], t.cy += b[i];
    t.cx /= n, t.cy /= n;
    double d = 0;
    for (int i = 0; i < n; i++) d += hypot(a[i] - t.cx, b[i] - t.cy);
    d /= n;
    t.s = d > 1e-12 ? sqrt(2.0) / d : 1;
    return t;
}

// Solve A z = b (8x8) by Gaussian elimination with partial pivoting.
static bool solve8(double A[8][9], double z[8]) {
    for (int c = 0; c < 8; c++) {
        int p = c;
        for (int r = c + 1; r < 8; r++)
            if (fabs(A[r][c]) > fabs(A[p][c])) p = r;
        if (fabs(A[p][c]) < 1e-9) return false;
        if (p != c)
            for (int k = 0; k < 9; k++) {
                double t = A[c][k];
                A[c][k]  = A[p][k];
                A[p][k]  = t;
            }
        for (int r = c + 1; r < 8; r++) {
            double f = A[r][c] / A[c][c];
            for (int k = c; k < 9; k++) A[r][k] -= f * A[c][k];
        }
    }
    for (int c = 7; c >= 0; c--) {
        double s = A[c][8];
        for (int k = c + 1; k < 8; k++) s -= A[c][k] * z[k];
        z[c] = s / A[c][c];
    }
    return true;
}

// sin of the angle at i in triangle (i, j, k): ~0 when the three are on one line
static float flatness(const float* u, const float* v, int i, int j, int k) {
    float ax = u[j] - u[i], ay = v[j] - v[i], bx = u[k] - u[i], by = v[k] - v[i];
    float d = hypotf(ax, ay) * hypotf(bx, by);
    return d > 0 ? fabsf(ax * by - ay * bx) / d : 0;
}

// Degenerate: all points (nearly) on one line, or, with exactly 4, any 3 of them.
static bool degenerate(int n, const float* u, const float* v) {
    const float FLAT = 0.05f;  // ~3 degrees
    bool any_flat = false, all_flat = true;
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            for (int k = j + 1; k < n; k++) {
                bool f = flatness(u, v, i, j, k) < FLAT && flatness(u, v, j, k, i) < FLAT;
                any_flat |= f;
                all_flat &= f;
            }
    return all_flat || (n == 4 && any_flat);
}

bool homography_fit(int n, const float* u, const float* v, const float* x, const float* y, float H[3][3],
                    float* err, float* rms) {
    if (n < 4 || degenerate(n, u, v) || degenerate(n, x, y)) return false;
    Norm ti = normaliser(n, u, v), ta = normaliser(n, x, y);

    double N[8][9] = {};  // normal equations [A^T A | A^T b]
    for (int i = 0; i < n; i++) {
        double a = (u[i] - ti.cx) * ti.s, b = (v[i] - ti.cy) * ti.s;
        double X = (x[i] - ta.cx) * ta.s, Y = (y[i] - ta.cy) * ta.s;
        double r1[9] = {a, b, 1, 0, 0, 0, -X * a, -X * b, X};
        double r2[9] = {0, 0, 0, a, b, 1, -Y * a, -Y * b, Y};
        for (int r = 0; r < 8; r++)
            for (int c = 0; c < 9; c++) N[r][c] += r1[r] * r1[c] + r2[r] * r2[c];
    }
    double h[8];
    if (!solve8(N, h)) return false;

    // H = Ta^-1 Hn Ti
    double Hn[3][3] = {{h[0], h[1], h[2]}, {h[3], h[4], h[5]}, {h[6], h[7], 1}};
    double Ti[3][3] = {{ti.s, 0, -ti.s * ti.cx}, {0, ti.s, -ti.s * ti.cy}, {0, 0, 1}};
    double Tai[3][3] = {{1 / ta.s, 0, ta.cx}, {0, 1 / ta.s, ta.cy}, {0, 0, 1}};
    double M[3][3], R[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            M[r][c] = 0;
            for (int k = 0; k < 3; k++) M[r][c] += Hn[r][k] * Ti[k][c];
        }
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            R[r][c] = 0;
            for (int k = 0; k < 3; k++) R[r][c] += Tai[r][k] * M[k][c];
        }
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) H[r][c] = (float)(R[r][c] / R[2][2]);

    double sum = 0;
    for (int i = 0; i < n; i++) {
        float px, py;
        float e = homography_apply(H, u[i], v[i], px, py) ? hypotf(px - x[i], py - y[i]) : INFINITY;
        if (err) err[i] = e;
        sum += (double)e * e;
    }
    if (rms) *rms = (float)sqrt(sum / n);
    return true;
}

bool homography_apply(const float H[3][3], float u, float v, float& x, float& y) {
    float w = H[2][0] * u + H[2][1] * v + H[2][2];
    if (fabsf(w) < 1e-9f) return false;
    x = (H[0][0] * u + H[0][1] * v + H[0][2]) / w;
    y = (H[1][0] * u + H[1][1] * v + H[1][2]) / w;
    return true;
}
