// Compares my SCARA IK against jared-hughes/grbl-scara (grbl/system.c:339).
// Their quick_atan2 is replaced by exact atan2 so we compare the MATH, not
// their polynomial approximation.
#include <cstdio>
#include <cmath>
const float L1 = 120.0f, L2 = 120.0f;   // their defaults, and this machine's
int fails = 0;
void check(bool ok, const char* w) { if (!ok) { printf("  FAIL: %s\n", w); fails++; } }

// ---- grbl-scara, transcribed from system_convert_mpos_to_array_steps ----
void grbl_scara(float x, float y, float& u_motor, float& l_motor) {
    float r_squared = x*x + y*y;
    float l1 = L1, l2 = L2;
    float s  = (l1 + l2)*(l1 + l2);
    float sm = (l1 - l2)*(l1 - l2);
    float theta = 2 * atan2f(sqrtf(r_squared - sm), sqrtf(s - r_squared));
    float angle = atan2f(y, x) + atan2f(2*l1*l2*sinf(theta), l1*l1 + r_squared - l2*l2);
    u_motor = angle;                       // shoulder
    l_motor = theta - float(M_PI) + angle; // forearm
}

// ---- mine: coupled_elbow, elbow_positive = false (their branch) ----
bool mine(float x, float y, float& m1, float& m2) {
    float r2 = x*x + y*y;
    float c = (r2 - L1*L1 - L2*L2) / (2*L1*L2);
    if (c > 1.0f) { if (c > 1.0001f) return false; c = 1.0f; }
    if (c < -1.0f) { if (c < -1.0001f) return false; c = -1.0f; }
    float t2 = -acosf(c);                                        // elbow_positive:false
    float t1 = atan2f(y, x) - atan2f(L2*sinf(t2), L1 + L2*cosf(t2));
    m1 = t1;
    m2 = t1 + t2;                                                // coupled: ABSOLUTE
    return true;
}

float wrap(float a) { while (a > M_PI) a -= 2*M_PI; while (a < -M_PI) a += 2*M_PI; return a; }

int main() {
    int n = 0; float worst1 = 0, worst2 = 0;
    for (float x = -245; x <= 245; x += 2.0f) {
        for (float y = -245; y <= 245; y += 2.0f) {
            float r = hypotf(x, y);
            if (r < 2.0f || r > L1 + L2 - 2.0f) continue;   // skip both singular edges
            float gu, gl, mm1, mm2;
            grbl_scara(x, y, gu, gl);
            if (!mine(x, y, mm1, mm2)) { check(false, "mine failed where theirs solved"); continue; }
            float d1 = fabsf(wrap(gu - mm1)), d2 = fabsf(wrap(gl - mm2));
            if (d1 > worst1) worst1 = d1;
            if (d2 > worst2) worst2 = d2;
            n++;
        }
    }
    printf("%d points compared over the shared workspace\n", n);
    printf("worst shoulder difference : %.6f deg\n", worst1 * 180/M_PI);
    printf("worst forearm  difference : %.6f deg\n", worst2 * 180/M_PI);
    check(worst1 * 180/M_PI < 0.01f, "shoulder angles agree");
    check(worst2 * 180/M_PI < 0.01f, "forearm angles agree");

    // Spot check a few explicit points, printed in degrees
    const float pts[][2] = { {200,0}, {0,200}, {-150,80}, {100,-170}, {60,16.1f} };
    printf("\n  point            grbl-scara (u, l)      mine (m1, m2)\n");
    for (auto& p : pts) {
        float gu, gl, mm1, mm2;
        grbl_scara(p[0], p[1], gu, gl); mine(p[0], p[1], mm1, mm2);
        printf("  (%7.1f,%7.1f)  %8.3f %8.3f    %8.3f %8.3f\n", p[0], p[1],
               gu*180/M_PI, gl*180/M_PI, mm1*180/M_PI, mm2*180/M_PI);
    }
    printf(fails ? "\n%d CHECK(S) FAILED\n" : "\nIDENTICAL\n", fails);
    return fails != 0;
}
