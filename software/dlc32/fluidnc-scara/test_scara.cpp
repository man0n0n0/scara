// Host harness: stubs out FluidNC, then #includes the real transform code.
#include <cmath>
#include <cstdio>
#include <cstdlib>

#define MAX_N_AXIS 6
typedef int axis_t;
enum { X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2 };
struct AxesStub { static axis_t _numberAxis; };
axis_t AxesStub::_numberAxis = 3;
#define Axes AxesStub
struct NullLog { template <class T> NullLog& operator<<(const T&) { return *this; } };
#define log_debug(x) do { NullLog n; n << x; } while (0)
#define log_error(x) do { NullLog n; n << x; } while (0)
template <class D, class S> void copyAxes(D* d, S* s, int n) { for (int i = 0; i < n; i++) d[i] = s[i]; }

namespace Kinematics {
  struct Scara {
    float _arm1_mm, _arm2_mm;
    float _j1_min_deg, _j1_max_deg, _j2_min_deg, _j2_max_deg;
    bool  _elbow_positive;
    bool  _coupled_elbow;
    static inline float to_degrees(float r) { return r * float(180.0 / M_PI); }
    static inline float to_radians(float d) { return d * float(M_PI / 180.0); }
    static float wrap_degrees(float degrees, float lo, float hi);
    bool transform_cartesian_to_motors(float* motors, float* cartesian);
    void motors_to_cartesian(float* cartesian, float* motors, axis_t n_axis);
  };
// the older cases pin the branch explicitly per test object
#define _elbow_positive_current _elbow_positive
#include "wrap.inc"
#include "ik.inc"
#include "fk.inc"
}

int fails = 0;
void check(bool ok, const char* what) { if (!ok) { printf("  FAIL: %s\n", what); fails++; } }

int main() {
  using namespace Kinematics;

  // --- Case 1: unequal arms, asymmetric joint limits, round-trip the workspace
  Scara s{ 180.0f, 120.0f, -135.0f, 135.0f, -150.0f, 150.0f, true, false };
  int tested = 0; float worst = 0;
  for (float x = -320; x <= 320; x += 2.5f) {
    for (float y = -320; y <= 320; y += 2.5f) {
      float cart[MAX_N_AXIS] = { x, y, 7.5f };
      float mot[MAX_N_AXIS]  = { 0 };
      if (!s.transform_cartesian_to_motors(mot, cart)) continue;
      float back[MAX_N_AXIS] = { 0 };
      s.motors_to_cartesian(back, mot, 3);
      float err = hypotf(back[0] - x, back[1] - y);
      if (err > worst) worst = err;
      check(err < 0.01f, "round trip error");
      check(back[2] == 7.5f, "Z passthrough");
      check(mot[0] >= s._j1_min_deg - 1e-3f && mot[0] <= s._j1_max_deg + 1e-3f, "J1 in range");
      check(mot[1] >= s._j2_min_deg - 1e-3f && mot[1] <= s._j2_max_deg + 1e-3f, "J2 in range");
      tested++;
    }
  }
  printf("case 1: %d reachable points, worst FK(IK(p)) error %.6f mm\n", tested, worst);
  check(tested > 10000, "workspace not empty");

  // --- Case 2: reach boundaries are respected
  Scara e{ 150.0f, 150.0f, -180.0f, 180.0f, -180.0f, 180.0f, true, false };
  float m[MAX_N_AXIS], c[MAX_N_AXIS] = { 0, 0, 0 };
  c[0] = 299.9f; c[1] = 0; check(e.transform_cartesian_to_motors(m, c),  "just inside max reach");
  c[0] = 300.0f; c[1] = 0; check(e.transform_cartesian_to_motors(m, c),  "exactly at max reach");
  c[0] = 300.5f; c[1] = 0; check(!e.transform_cartesian_to_motors(m, c), "beyond max reach rejected");
  c[0] = 0.0f;   c[1] = 0; check(e.transform_cartesian_to_motors(m, c),  "centre, equal arms (folded)");

  Scara d{ 180.0f, 120.0f, -180.0f, 180.0f, -180.0f, 180.0f, true, false };
  c[0] = 60.0f;  c[1] = 0; check(d.transform_cartesian_to_motors(m, c),  "exactly on the dead-zone edge");
  c[0] = 60.5f;  c[1] = 0; check(d.transform_cartesian_to_motors(m, c),  "just outside the dead zone");
  c[0] = 30.0f;  c[1] = 0; check(!d.transform_cartesian_to_motors(m, c), "well inside dead zone rejected");
  c[0] = 55.0f;  c[1] = 0; check(!d.transform_cartesian_to_motors(m, c), "inside dead zone rejected");
  c[0] = 0.0f;   c[1] = 0; check(!d.transform_cartesian_to_motors(m, c), "centre unreachable, unequal arms");

  // --- Case 3: both elbow branches reach the same XY
  Scara p{ 180.0f, 120.0f, -180.0f, 180.0f, -180.0f, 180.0f, true, false };
  Scara n{ 180.0f, 120.0f, -180.0f, 180.0f, -180.0f, 180.0f, false, false };
  float mp[MAX_N_AXIS], mn[MAX_N_AXIS], bp[MAX_N_AXIS], bn[MAX_N_AXIS];
  c[0] = 120.0f; c[1] = 90.0f;
  check(p.transform_cartesian_to_motors(mp, c) && n.transform_cartesian_to_motors(mn, c), "both branches solve");
  p.motors_to_cartesian(bp, mp, 3); n.motors_to_cartesian(bn, mn, 3);
  check(hypotf(bp[0] - c[0], bp[1] - c[1]) < 0.01f, "elbow+ lands on target");
  check(hypotf(bn[0] - c[0], bn[1] - c[1]) < 0.01f, "elbow- lands on target");
  check(fabsf(mp[1] + mn[1]) < 0.01f, "branches are mirrored in J2");
  printf("case 3: elbow+ J1=%.3f J2=%.3f / elbow- J1=%.3f J2=%.3f\n", mp[0], mp[1], mn[0], mn[1]);

  // --- Case 4: a joint window that does not straddle zero (tests wrap_degrees)
  Scara w{ 150.0f, 150.0f, 20.0f, 300.0f, 0.0f, 170.0f, true, false };
  c[0] = -200.0f; c[1] = -60.0f;   // needs a shoulder angle well past 180
  check(w.transform_cartesian_to_motors(m, c), "solves with a 20..300 deg J1 window");
  check(m[0] >= 20.0f && m[0] <= 300.0f, "J1 wrapped into its window");
  w.motors_to_cartesian(bp, m, 3);
  check(hypotf(bp[0] - c[0], bp[1] - c[1]) < 0.01f, "wrapped solution still lands on target");
  printf("case 4: J1=%.3f J2=%.3f\n", m[0], m[1]);

  // --- Case 5: joint limits actually reject an otherwise reachable point
  Scara l{ 150.0f, 150.0f, -10.0f, 10.0f, -180.0f, 180.0f, true, false };
  c[0] = 0.0f; c[1] = 250.0f;   // straight up: needs J1 near 90 deg
  check(!l.transform_cartesian_to_motors(m, c), "point outside J1 limits rejected");


  // --- Case 6: COUPLED elbow, base-mounted motor, same ratio and direction
  Scara k{ 180.0f, 120.0f, -135.0f, 135.0f, -150.0f, 150.0f, true, true };
  int ktested = 0; float kworst = 0;
  for (float x = -320; x <= 320; x += 2.5f) {
    for (float y = -320; y <= 320; y += 2.5f) {
      float cart[MAX_N_AXIS] = { x, y, 3.0f };
      float mot[MAX_N_AXIS] = { 0 }, back[MAX_N_AXIS] = { 0 };
      if (!k.transform_cartesian_to_motors(mot, cart)) continue;
      k.motors_to_cartesian(back, mot, 3);
      float err = hypotf(back[0] - x, back[1] - y);
      if (err > kworst) kworst = err;
      check(err < 0.01f, "coupled round trip");
      // the RELATIVE elbow angle must still respect the mechanical stops
      float rel = mot[1] - mot[0];
      check(rel >= k._j2_min_deg - 1e-2f && rel <= k._j2_max_deg + 1e-2f, "coupled relative J2 in range");
      ktested++;
    }
  }
  printf("case 6: coupled, %d points, worst error %.6f mm\n", ktested, kworst);
  check(ktested > 10000, "coupled workspace not empty");

  // --- Case 7: the mechanical property. Hold motor Y, sweep motor X:
  //     arm2's absolute orientation must not change.
  {
    float m[MAX_N_AXIS] = { 0, 40.0f, 0 }, p0[MAX_N_AXIS], p1[MAX_N_AXIS];
    m[0] = 10.0f; k.motors_to_cartesian(p0, m, 3);
    m[0] = 55.0f; k.motors_to_cartesian(p1, m, 3);
    // elbow position = shoulder + arm1; tool - elbow = arm2 vector
    float e0x = 180.0f * cosf(10.0f * float(M_PI) / 180.0f), e0y = 180.0f * sinf(10.0f * float(M_PI) / 180.0f);
    float e1x = 180.0f * cosf(55.0f * float(M_PI) / 180.0f), e1y = 180.0f * sinf(55.0f * float(M_PI) / 180.0f);
    float a0 = atan2f(p0[1] - e0y, p0[0] - e0x) * 180.0f / float(M_PI);
    float a1 = atan2f(p1[1] - e1y, p1[0] - e1x) * 180.0f / float(M_PI);
    check(fabsf(a0 - 40.0f) < 0.01f && fabsf(a1 - 40.0f) < 0.01f, "arm2 absolute angle tracks motor Y alone");
    printf("case 7: motor Y=40 deg, shoulder 10->55 deg, arm2 world angle %.3f -> %.3f\n", a0, a1);
  }

  // --- Case 8: coupled and uncoupled disagree (guards against a no-op flag)
  {
    Scara u{ 180.0f, 120.0f, -135.0f, 135.0f, -150.0f, 150.0f, true, false };
    float cc[MAX_N_AXIS] = { 150.0f, 100.0f, 0 }, mu[MAX_N_AXIS], mk[MAX_N_AXIS];
    check(u.transform_cartesian_to_motors(mu, cc) && k.transform_cartesian_to_motors(mk, cc), "both solve");
    check(fabsf(mu[0] - mk[0]) < 1e-3f, "same shoulder angle either way");
    check(fabsf(mk[1] - (mu[0] + mu[1])) < 1e-3f, "coupled Y = uncoupled X + Y");
    printf("case 8: uncoupled J1=%.3f J2rel=%.3f | coupled m1=%.3f m2=%.3f\n", mu[0], mu[1], mk[0], mk[1]);
  }


  // --- Case 9: homing compensation. Adding the SAME displacement to both
  //     motors must rotate the arm rigidly: relative elbow angle unchanged,
  //     and the tool's distance from the shoulder unchanged.
  {
    float m[MAX_N_AXIS] = { -20.0f, 35.0f, 0 };
    float start[MAX_N_AXIS]; k.motors_to_cartesian(start, m, 3);
    float r_start = hypotf(start[0], start[1]);
    float rel_start = m[1] - m[0];
    bool  ok = true;
    // sweep the shoulder the way a homing seek would, carrying Y along
    for (float d = 0; d >= -270.0f; d -= 1.0f) {
      float mm[MAX_N_AXIS] = { -20.0f + d, 35.0f + d, 0 };
      float pos[MAX_N_AXIS]; k.motors_to_cartesian(pos, mm, 3);
      if (fabsf((mm[1] - mm[0]) - rel_start) > 1e-3f) ok = false;
      if (fabsf(hypotf(pos[0], pos[1]) - r_start) > 0.01f) ok = false;
    }
    check(ok, "coupled homing sweep keeps elbow angle and reach constant");
    printf("case 9: rel elbow %.3f deg, reach %.3f mm, held over a 270 deg sweep\n", rel_start, r_start);

    // and WITHOUT compensation the elbow angle runs away, which is the hazard
    float mbad[MAX_N_AXIS] = { -20.0f - 270.0f, 35.0f, 0 };
    float rel_bad = mbad[1] - mbad[0];
    check(fabsf(rel_bad - rel_start) > 200.0f, "uncompensated sweep does wreck the elbow angle");
    printf("case 9: uncompensated, elbow would go %.1f -> %.1f deg\n", rel_start, rel_bad);
  }


  // --- Case 10: REGRESSION. A j2 window of +/-360 must not flip the elbow
  //     sign or shift the motor by a whole turn.
  {
    Scara w{ 120.0f, 120.0f, -100.0f, 100.0f, -360.0f, 360.0f, true, true };
    float cc[MAX_N_AXIS] = { 60.0f, 16.1f, 0 }, m[MAX_N_AXIS];
    check(w.transform_cartesian_to_motors(m, cc), "wide window solves");
    float rel = m[1] - m[0];
    check(rel > 0.0f, "elbow_positive:true really gives a POSITIVE relative angle");
    check(fabsf(rel) < 190.0f, "relative angle not shifted by a whole turn");
    printf("case 10: wide j2 window -> m1=%.3f m2=%.3f rel=%.3f\n", m[0], m[1], rel);

    // same geometry, sane window: must give the same answer
    Scara n2{ 120.0f, 120.0f, -100.0f, 100.0f, -180.0f, 180.0f, true, true };
    float m2[MAX_N_AXIS];
    check(n2.transform_cartesian_to_motors(m2, cc), "sane window solves");
    check(fabsf(m[0] - m2[0]) < 1e-3f && fabsf(m[1] - m2[1]) < 1e-3f,
          "wide and sane windows agree");

    // and elbow_positive:false must mirror it
    Scara nf{ 120.0f, 120.0f, -100.0f, 100.0f, -180.0f, 180.0f, false, true };
    float m3[MAX_N_AXIS];
    check(nf.transform_cartesian_to_motors(m3, cc), "negative branch solves");
    check((m3[1] - m3[0]) < 0.0f, "elbow_positive:false gives a NEGATIVE relative angle");
    printf("case 10: elbow- -> m1=%.3f m2=%.3f rel=%.3f\n", m3[0], m3[1], m3[1] - m3[0]);
  }

  printf(fails ? "\n%d CHECK(S) FAILED\n" : "\nall checks passed\n", fails);
  return fails != 0;
}
