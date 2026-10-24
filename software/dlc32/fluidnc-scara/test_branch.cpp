// Uses the REAL IK, FK and sync_from_motors extracted from Scara.cpp.
#include <cmath>
#include <cstdio>
#include <initializer_list>
#define MAX_N_AXIS 6
typedef int axis_t;
enum { X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2 };
struct AxesStub { static axis_t _numberAxis; }; axis_t AxesStub::_numberAxis = 3;
#define Axes AxesStub
struct NullLog { template <class T> NullLog& operator<<(const T&) { return *this; } };
#define log_debug(x) do { NullLog n; n << x; } while (0)
template <class D, class S> void copyAxes(D* d, S* s, int n) { for (int i = 0; i < n; i++) d[i] = s[i]; }
static float g_steps[MAX_N_AXIS];                 // stands in for the step counters
float* get_motor_pos() { return g_steps; }

namespace Kinematics {
  struct Scara {
    float _arm1_mm = 120, _arm2_mm = 120;
    float _j1_min_deg = -90, _j1_max_deg = 90, _j2_min_deg = -180, _j2_max_deg = 180;
    bool  _elbow_positive = true;                 // the user's config value
    bool  _coupled_elbow  = true;
    bool  _elbow_positive_current = true;
    float _last_motor_pos[MAX_N_AXIS] = { 0 };
    static float to_degrees(float r) { return r * float(180.0 / M_PI); }
    static float to_radians(float d) { return d * float(M_PI / 180.0); }
    static float wrap_degrees(float, float, float);
    float relative_elbow_deg(const float* m) const { return _coupled_elbow ? m[Y_AXIS] - m[X_AXIS] : m[Y_AXIS]; }
    bool  transform_cartesian_to_motors(float* motors, float* cartesian);
    void  motors_to_cartesian(float* cartesian, float* motors, axis_t n_axis);
    void  sync_from_motors();
  };
#include "wrap.inc"
#include "ik.inc"
#include "fk.inc"
#include "sync.inc"
}
int fails = 0;
void check(bool ok, const char* w) { if (!ok) { printf("  FAIL: %s\n", w); fails++; } }

int main() {
  using namespace Kinematics;
  Scara s;
  // Homed exactly as the user's config says: J1 = 70, relative elbow = -150.
  g_steps[0] = 70; g_steps[1] = 70 - 150; g_steps[2] = 0;
  s.sync_from_motors();                            // what set_homed_mpos now does
  float here[MAX_N_AXIS]; s.motors_to_cartesian(here, g_steps, 3);
  printf("homed: tool (%.2f, %.2f), elbow %.1f, branch %s (config says %s)\n", here[0], here[1],
         s.relative_elbow_deg(g_steps), s._elbow_positive_current ? "positive" : "negative",
         s._elbow_positive ? "positive" : "negative");
  check(!s._elbow_positive_current, "branch taken from the arm, not the config");

  // $J=G91 X-1
  float tgt[MAX_N_AXIS] = { here[0] - 1, here[1], 0 }, m[MAX_N_AXIS];
  check(s.transform_cartesian_to_motors(m, tgt), "1 mm jog is solvable");
  float d1 = m[0] - g_steps[0], d2 = m[1] - g_steps[1];
  printf("jog X-1: J1 %+.3f deg, J2 %+.3f deg\n", d1, d2);
  check(fabsf(d1) < 2 && fabsf(d2) < 2, "1 mm jog moves the joints by < 2 deg");

  // Every direction, 1 mm and 10 mm: never a flip
  float worst = 0;
  for (int a = 0; a < 360; a += 15) for (float step : { 1.0f, 10.0f }) {
    float t[MAX_N_AXIS] = { here[0] + step * cosf(a * M_PI / 180), here[1] + step * sinf(a * M_PI / 180), 0 };
    if (!s.transform_cartesian_to_motors(m, t)) continue;
    worst = fmaxf(worst, fmaxf(fabsf(m[0] - g_steps[0]), fabsf(m[1] - g_steps[1])));
  }
  printf("worst joint change for any 1-10 mm move from home: %.2f deg\n", worst);
  check(worst < 15, "no move near home flips the arm");

  // Stale bookkeeping after a reset: planner resynced to the real steps,
  // _last_motor_pos still holds a target the arm never reached.
  s._last_motor_pos[0] = 170; s._last_motor_pos[1] = 20;
  float fk_last[MAX_N_AXIS]; s.motors_to_cartesian(fk_last, s._last_motor_pos, 3);
  float gap = hypotf(fk_last[0] - here[0], fk_last[1] - here[1]);
  printf("after reset: stale bookkeeping is %.1f mm from the real position\n", gap);
  check(gap > 0.01f, "staleness is detectable by FK");
  s.sync_from_motors();
  check(fabsf(s._last_motor_pos[0] - 70) < 1e-4f && fabsf(s._last_motor_pos[1] + 80) < 1e-4f, "resync restores the real motors");

  printf(fails ? "\n%d CHECK(S) FAILED\n" : "\nall checks passed\n", fails);
  return fails != 0;
}
