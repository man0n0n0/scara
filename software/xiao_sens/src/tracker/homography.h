#pragma once
// Plane-to-plane mapping (homography) for the fixed camera: image (u, v) to
// arm (x, y). Pure math, no Arduino dependencies, so it can be tested on a PC.

// Fit H to n >= 4 pairs (u, v) -> (x, y), least squares, with Hartley
// normalisation. Fills err[i] (mm) and *rms when given. False if degenerate
// (fewer than 4 points, or 3 of them on one line).
bool homography_fit(int n, const float* u, const float* v, const float* x, const float* y, float H[3][3],
                    float* err = nullptr, float* rms = nullptr);

// (u, v) -> (x, y). False if the point maps to infinity (beyond the horizon).
bool homography_apply(const float H[3][3], float u, float v, float& x, float& y);
