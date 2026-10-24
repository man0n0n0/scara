// Guards: block entry into a bad region, but never trap the arm inside one.
#include <cstdio>
#include <cmath>
#include <algorithm>
int fails = 0;
void check(bool ok, const char* w) { if (!ok) { printf("  FAIL: %s\n", w); fails++; } }

const float R_MIN = 40.0f, R_MAX = 230.0f, X_MIN = -100000.0f;

bool envelope_error(float x, float y, float r_floor, float r_ceiling, float x_floor) {
    float r = sqrtf(x*x + y*y);
    return r < r_floor || r > r_ceiling || x < x_floor;
}
// one move: start -> target, with the relaxation applied
bool move_allowed(float x0, float y0, float x1, float y1) {
    float r_start = hypotf(x0, y0);
    float r_floor = std::min(R_MIN, r_start);
    float r_ceil  = std::max(R_MAX, r_start);
    float x_floor = std::min(X_MIN, x0);
    return !envelope_error(x1, y1, r_floor, r_ceil, x_floor);
}

int main() {
    // normal operation, well inside
    check( move_allowed(100, 0, 120, 30), "ordinary move allowed");
    // entering the forbidden centre from outside -> blocked
    check(!move_allowed(100, 0,  10,  0), "move INTO the centre blocked");
    check(!move_allowed(100, 0,  39,  0), "just inside r_min blocked");
    check( move_allowed(100, 0,  41,  0), "just outside r_min allowed");
    // beyond reach limit
    check(!move_allowed(100, 0, 240,  0), "move beyond r_max blocked");
    check( move_allowed(100, 0, 229,  0), "just inside r_max allowed");

    // THE ESCAPE CASE: homed folded at the singularity, r ~ 0
    check( move_allowed(0.001f, 0.0f, 0.001f, -1.0f), "1mm move from the folded home allowed");
    check( move_allowed(0.001f, 0.0f, 100.0f,  0.0f), "escaping outward allowed");
    printf("  from r=0: 1mm move allowed = %s\n",
           move_allowed(0.001f, 0, 0.001f, -1.0f) ? "yes" : "NO - would be trapped");

    // starting outside r_max (e.g. r_max lowered after the fact) can come back
    check( move_allowed(238.0f, 0, 200.0f, 0), "returning inward from beyond r_max allowed");
    check(!move_allowed(238.0f, 0, 239.0f, 0), "from beyond r_max, going FURTHER out is still blocked");
    check( move_allowed(238.0f, 0, 238.0f, 0), "staying put is allowed");

    printf(fails ? "\n%d CHECK(S) FAILED\n" : "\nall checks passed\n", fails);
    return fails != 0;
}
