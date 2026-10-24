// Image-based visual servoing over FluidNC jogs.
//
//   e      = filtered target position in the image, (u, v) in -1..1, 0 = centre
//   d      = -J(phi)^-1 e          arm move that would centre the target
//   v_cmd  = kp * d, capped at vmax, slew-limited by amax, zero inside the deadband
//
// J is measured once by calibrate() at forearm angle phi0. The camera rides on
// the forearm, so the image turns with phi = theta1 + theta2:
//   J(phi) = J0 R(phi0 - phi)   =>   J(phi)^-1 = R(phi - phi0) J0^-1
//
// Velocity is turned into short relative jogs ($J=G91 ...), each lasting
// horizon_s, and a new one is queued only when the previous is about to run
// out. At most ~2 segments are ever queued, so if this board crashes, loses
// WiFi or hangs, the arm stops by itself within that distance. On target loss,
// disable, alarm or stale status we also send jog-cancel (0x85).
//
// Everything that touches the telnet socket runs in one task on core 0.

#include "controller.h"
#include <Preferences.h>
#include <math.h>
#include <algorithm>
#include <stdarg.h>
#include "config.h"
#include "fluidnc.h"
#include "homography.h"
#include "lights.h"
#include "secrets.h"
#include "vision.h"

namespace controller {

static const uint32_t PARAMS_MAGIC = 0x5041520A;  // bump when Params defaults/layout change
static const uint32_t PARAMS_MAGIC_V9 = 0x50415209;  // before wander_margin, z_chunk_s
static const uint32_t PARAMS_MAGIC_V8 = 0x50415208;  // before z_max .. z_down_feed
static const uint32_t PARAMS_MAGIC_V7 = 0x50415207;  // before light_hold_s, fast_feed
static const uint32_t PARAMS_MAGIC_V6 = 0x50415206;  // before spiral_pitch, spiral_r_min
static const uint32_t PARAMS_MAGIC_V5 = 0x50415205;  // before orbit_r, orbit_feed; wander_s was 6 s
static const uint32_t PARAMS_MAGIC_V4 = 0x50415204;  // before center_x .. light_s were appended
static const uint32_t PARAMS_MAGIC_V3 = 0x50415203;  // before recover_feed .. wander_feed were appended
static const uint32_t PARAMS_MAGIC_V2 = 0x50415202;  // before off_x .. db_mm_out were appended
static const uint32_t PARAMS_MAGIC_V1 = 0x50415201;  // had 30 mm / 1200 mm/min calibration moves
static const uint32_t CAL_MAGIC    = 0x43414C01;
static const uint32_t MAP_MAGIC    = 0x4D415001;

static FluidNC      fnc;
static Params       P;
static Calibration  cal;
static bool         cal_valid = false;
static FloorMap     fmap;              // fixed camera: taught points + H
static bool         mark_set = false;  // image point waiting for "record arm"
static float        mark_u, mark_v;

static portMUX_TYPE  mux = portMUX_INITIALIZER_UNLOCKED;
static CtlStatus     pub;               // published status, guarded by mux
static volatile bool req_enable = false, req_disable = false, req_cal = false, req_man = false, req_clear = false;
static float         man_th, man_k;     // manual calibration request
// floor map teaching: one request at a time, served by the control task
enum class MapReq : uint8_t { None, Mark, MarkAt, Record, Delete, Solve, Clear };
static volatile MapReq req_map = MapReq::None;
static float           req_map_u, req_map_v;
static int             req_map_i;
static char          req_why[48];

static Mode     mode = Mode::Disabled;
static char     reason[64] = "disabled at boot - press Enable";
// Tracking enables by itself at boot (and again AUTO_ENABLE_RETRY_MS after a
// fault turned it off) unless the operator stopped it: STOP / Disable / stop
// button sets operator_off until reboot, Enable or Solve clears it. RAM only.
static bool     operator_off = false;
// "human 0" on UDP LIGHTS_PORT: everything off (motion, lights) and the state
// before it kept; "human 1": that state back. RAM only.
static bool     human_off = false;
static bool     saved_operator_off, saved_auto_lights;
static char     saved_lamps[32];  // light controller's "spot N light N" at "human 0"
static uint32_t auto_enable_next = 0;
static const uint32_t AUTO_ENABLE_RETRY_MS = 10000;
static uint32_t unlock_ms = 0;  // ALARM:2: $X sent at, 0 = not handling one (see alarm_step)

// tracking state
static bool     have_filt = false, in_deadband = false;
static float    fu = 0, fv = 0;          // filtered target
static uint32_t last_seen = 0, last_seq = 0, last_frame_ms = 0, last_ctl_ms = 0;
static float    vx = 0, vy = 0;          // commanded velocity, jog units/s
static float    fps = 0;
// jog queue model
static bool     moving = false;          // jogs in flight (until FluidNC reports Idle)
static uint32_t queue_end = 0;           // when the queued jogs should finish
static uint32_t last_send_ms = 0;
static float    pred[2] = {0, 0};        // XY once the queue has run out
static uint32_t jogs_sent = 0, cancels = 0;
static uint32_t enabled_ms = 0;          // tracking (re)started: "no target" counts from here
// back inside the soft limits (see recover_step)
static bool     recovering = false;      // that jog is in flight
static uint32_t recover_ms = 0;          // when it was sent
static uint32_t cancel_ms = 0;           // last jog-cancel: new jogs wait for Idle after it
static float    z_want = NAN;            // Z the jogs take the tool to (NAN = leave Z alone)
static float    pred_z = 0;              // Z once the queue has run out
static bool     center_pending = true;   // go to the workspace centre once FluidNC is ready (boot, after homing)
static uint32_t rehome_next = 0;         // FluidNC back in Alarm: next reset + home attempt

// ------------------------------------------------------------ params / NVS --

static void param_defaults(Params& p) {
    p.kp        = 1.0f;
    p.vmax      = 60.0f;
    p.amax      = 150.0f;
    p.db_in     = 0.04f;
    p.db_out    = 0.08f;
    p.alpha     = 0.5f;
    p.min_conf  = 0.5f;
    p.lost_ms   = 400.0f;
    p.horizon_s = 0.25f;
    p.cal_step  = 300.0f;
    p.cal_feed  = 3000.0f;
    p.off_x     = 0.0f;
    p.off_y     = 0.0f;
    p.db_mm_in  = 30.0f;
    p.db_mm_out = 60.0f;
    p.recover_feed = 1500.0f;
    p.wander_s     = 33.0f;
    p.wander_feed  = 1500.0f;
    p.center_x     = 0.0f;
    p.center_y     = 1000.0f;
    p.light_s      = 6.0f;
    p.orbit_r      = 200.0f;
    p.orbit_feed   = 1500.0f;
    p.spiral_pitch = 50.0f;
    p.spiral_r_min = 20.0f;
    p.light_hold_s = 6.0f;
    p.fast_feed    = 7000.0f;
    p.z_max        = 200.0f;
    p.z_min        = 50.0f;
    // FluidNC's Z max_rate is 600 mm/min: a faster Z in a combined XY+Z jog
    // would make FluidNC slow the whole jog, XY included
    p.z_up_feed    = 600.0f;
    p.z_down_feed  = 300.0f;
    p.wander_margin = 500.0f;
    p.z_chunk_s     = 0.1f;
}

struct ParamDef { const char* name; float Params::*field; float lo, hi; };
static const ParamDef PARAM_DEFS[] = {
    {"kp", &Params::kp, 0.0f, 5.0f},
    {"vmax", &Params::vmax, 0.0f, 300.0f},
    {"amax", &Params::amax, 1.0f, 2000.0f},
    {"db_in", &Params::db_in, 0.0f, 0.5f},
    {"db_out", &Params::db_out, 0.0f, 0.6f},
    {"alpha", &Params::alpha, 0.05f, 1.0f},
    {"min_conf", &Params::min_conf, 0.1f, 0.99f},
    {"lost_ms", &Params::lost_ms, 100.0f, 3000.0f},
    {"horizon_s", &Params::horizon_s, 0.1f, 1.0f},
    {"cal_step", &Params::cal_step, 2.0f, 1000.0f},
    {"cal_feed", &Params::cal_feed, 60.0f, 6000.0f},
    {"off_x", &Params::off_x, -500.0f, 500.0f},
    {"off_y", &Params::off_y, -500.0f, 500.0f},
    {"db_mm_in", &Params::db_mm_in, 0.0f, 300.0f},
    {"db_mm_out", &Params::db_mm_out, 0.0f, 400.0f},
    {"recover_feed", &Params::recover_feed, 60.0f, 6000.0f},
    {"wander_s", &Params::wander_s, 0.0f, 600.0f},
    {"wander_feed", &Params::wander_feed, 60.0f, 6000.0f},
    {"center_x", &Params::center_x, -3000.0f, 3000.0f},
    {"center_y", &Params::center_y, -3000.0f, 3000.0f},
    {"light_s", &Params::light_s, 0.0f, 600.0f},
    {"orbit_r", &Params::orbit_r, 30.0f, 800.0f},
    {"orbit_feed", &Params::orbit_feed, 60.0f, 6000.0f},
    {"spiral_pitch", &Params::spiral_pitch, 5.0f, 400.0f},
    {"spiral_r_min", &Params::spiral_r_min, 0.0f, 400.0f},
    {"light_hold_s", &Params::light_hold_s, 0.0f, 600.0f},
    {"fast_feed", &Params::fast_feed, 60.0f, 15000.0f},
    {"z_max", &Params::z_max, 0.0f, 1000.0f},
    {"z_min", &Params::z_min, 0.0f, 1000.0f},
    {"z_up_feed", &Params::z_up_feed, 60.0f, 15000.0f},
    {"z_down_feed", &Params::z_down_feed, 60.0f, 15000.0f},
    {"wander_margin", &Params::wander_margin, 0.0f, 1500.0f},
    {"z_chunk_s", &Params::z_chunk_s, 0.02f, 1.0f},
};

static void nvs_save_params();

// The arm moves in a plane the camera looks straight down on, so the true J is
// a scale times a rotation, with the image +Y axis 90 deg counter-clockwise
// from +X (v points down in the image, hence the reflection form). Replace the
// measured J by the closest such matrix (least squares): noise in calibration
// can otherwise leave the axes far from square, or even mirrored.
static void orthogonalise(float J[2][2]) {
#if CAM_MIRRORED
    // [c -s; s c]
    float th = atan2f(J[1][0] - J[0][1], J[0][0] + J[1][1]);
    float k  = 0.5f * hypotf(J[1][0] - J[0][1], J[0][0] + J[1][1]);
    float c = k * cosf(th), sn = k * sinf(th);
    J[0][0] = c, J[0][1] = -sn, J[1][0] = sn, J[1][1] = c;
#else
    // [c s; s -c]
    float th = atan2f(J[1][0] + J[0][1], J[0][0] - J[1][1]);
    float k  = 0.5f * hypotf(J[1][0] + J[0][1], J[0][0] - J[1][1]);
    float c = k * cosf(th), sn = k * sinf(th);
    J[0][0] = c, J[0][1] = sn, J[1][0] = sn, J[1][1] = -c;
#endif
}

static void nvs_load() {
    Preferences pr;
    param_defaults(P);
    if (!pr.begin("track", true)) return;
    struct { uint32_t magic; Params p; } blob;
    bool migrate = false;
    // Older blobs are shorter (fields are only ever appended): V1/V2 end before
    // off_x, V3 before recover_feed, V4 before center_x, V5 before orbit_r, V6 before
    // spiral_pitch, V7 before light_hold_s, V8 before z_max.
    // The missing tail keeps its defaults.
    const size_t HDR = sizeof(uint32_t), V2_SIZE = HDR + offsetof(Params, off_x);
    size_t got = pr.getBytesLength("params");
    if (got >= V2_SIZE && got <= sizeof(blob) && (got - HDR) % sizeof(float) == 0 &&
        pr.getBytes("params", &blob, got) == got) {
        if (got < sizeof(blob)) {
            Params d;
            param_defaults(d);
            size_t have = got - HDR;
            memcpy((uint8_t*)&blob.p + have, (uint8_t*)&d + have, sizeof(Params) - have);
            migrate = true;
        }
        if (blob.magic == PARAMS_MAGIC || blob.magic == PARAMS_MAGIC_V9 || blob.magic == PARAMS_MAGIC_V8 || blob.magic == PARAMS_MAGIC_V7 ||
            blob.magic == PARAMS_MAGIC_V6 ||
            blob.magic == PARAMS_MAGIC_V5 ||
            blob.magic == PARAMS_MAGIC_V4 ||
            blob.magic == PARAMS_MAGIC_V3 || blob.magic == PARAMS_MAGIC_V2)
            P = blob.p;
        // wandering now starts after 33 s, not the old 6 s default
        if (blob.magic != PARAMS_MAGIC && blob.magic != PARAMS_MAGIC_V9 && blob.magic != PARAMS_MAGIC_V8 && blob.magic != PARAMS_MAGIC_V7 &&
            blob.magic != PARAMS_MAGIC_V6 && got >= HDR + offsetof(Params, wander_s) + sizeof(float)) {
            Params d;
            param_defaults(d);
            P.wander_s = d.wander_s;
            migrate    = true;
        }
        if (blob.magic == PARAMS_MAGIC_V1) {
            // keep the user's tuning, but not the old, too small calibration move
            Params d;
            param_defaults(d);
            P          = blob.p;
            P.cal_step = d.cal_step;
            P.cal_feed = d.cal_feed;
            migrate    = true;
        }
    }
    if (pr.getBytes("cal", &cal, sizeof(cal)) == sizeof(cal) && cal.magic == CAL_MAGIC) {
        orthogonalise(cal.J);  // also squares up calibrations saved before this existed
        cal_valid = true;
    }
    if (pr.getBytes("map", &fmap, sizeof(fmap)) != sizeof(fmap) || fmap.magic != MAP_MAGIC) fmap = {};
    pr.end();
    if (migrate) nvs_save_params();
}

static void nvs_save_params() {
    Preferences pr;
    if (!pr.begin("track", false)) return;
    struct { uint32_t magic; Params p; } blob = {PARAMS_MAGIC, P};
    pr.putBytes("params", &blob, sizeof(blob));
    pr.end();
}

static void nvs_save_map() {
    Preferences pr;
    if (!pr.begin("track", false)) return;
    fmap.magic = MAP_MAGIC;
    pr.putBytes("map", &fmap, sizeof(fmap));
    pr.end();
}

static void nvs_save_cal() {
    Preferences pr;
    if (!pr.begin("track", false)) return;
    pr.putBytes("cal", &cal, sizeof(cal));
    pr.end();
}

// ---------------------------------------------------------------- geometry --

// Absolute forearm angle theta1 + theta2 (rad) for a cartesian tool position.
static float forearm_angle(float x, float y) {
    const float L1 = ARM_L1_MM, L2 = ARM_L2_MM;
    float c  = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    float t2 = ELBOW_SIGN * acosf(fmaxf(-1.0f, fminf(1.0f, c)));
    float t1 = atan2f(y, x) - atan2f(L2 * sinf(t2), L1 + L2 * cosf(t2));
    return t1 + t2;
}

// Inside the soft limits, at least m mm from them.
static bool inside(float x, float y, float m = 0) {
    if (x < LIM_X_MIN + m || x > LIM_X_MAX - m || y < LIM_Y_MIN + m || y > LIM_Y_MAX - m) return false;
#if JOG_CARTESIAN
    float r = hypotf(x, y);
    if (r < LIM_R_MIN + m || r > LIM_R_MAX - m) return false;
#endif
    return true;
}

// Nearest point at least m mm inside the soft limits: into the box, then into
// the annulus, a few rounds so that both hold.
static void nearest_inside(float x, float y, float m, float& tx, float& ty) {
    tx = x, ty = y;
    for (int i = 0; i < 4; i++) {
        tx = fminf(fmaxf(tx, LIM_X_MIN + m), LIM_X_MAX - m);
        ty = fminf(fmaxf(ty, LIM_Y_MIN + m), LIM_Y_MAX - m);
#if JOG_CARTESIAN
        float r = hypotf(tx, ty), rc = fminf(fmaxf(r, LIM_R_MIN + m), LIM_R_MAX - m);
        if (r < 1.0f) tx = 0, ty = rc;
        else if (rc != r) tx *= rc / r, ty *= rc / r;
#endif
    }
}

// Shrink (dx, dy) so that p + d stays inside the soft limits. p must be inside.
static void clamp_step(const float p[2], float& dx, float& dy) {
    if (inside(p[0] + dx, p[1] + dy)) return;
    float lo = 0, hi = 1;
    for (int i = 0; i < 12; i++) {
        float t = 0.5f * (lo + hi);
        (inside(p[0] + t * dx, p[1] + t * dy) ? lo : hi) = t;
    }
    dx *= lo;
    dy *= lo;
}

// ------------------------------------------------------------------ safety --

static bool status_fresh() {
    const auto& s = fnc.status();
    return s.t_ms && millis() - s.t_ms < STATUS_STALE_MS;
}

// FluidNC Idle with nothing outstanding, in a status newer than t: whatever
// was sent at t has been done.
static bool idle_after(uint32_t t) {
    const auto& s = fnc.status();
    return fnc.stateIs("Idle") && fnc.outstanding() == 0 && (int32_t)(s.t_ms - t) > 150;
}

// Conditions for any automatic motion. Fills why when it says no.
static bool motion_allowed(char* why, size_t n, bool need_cal) {
    const auto& s = fnc.status();
    if (!fnc.connected()) return snprintf(why, n, "FluidNC not connected"), false;
    if (!status_fresh()) return snprintf(why, n, "no status from FluidNC"), false;
    if (!fnc.stateIs("Idle") && !fnc.stateIs("Jog"))
        return snprintf(why, n, "FluidNC state %s %s", s.state, fnc.lastAlarm()), false;
    // outside the soft limits is not a reason to stop: recover_step() brings the arm back
#if CAMERA_FIXED
    if (need_cal && !fmap.solved) return snprintf(why, n, "no floor map - teach points and Solve"), false;
#else
    if (need_cal && !cal_valid) return snprintf(why, n, "not calibrated"), false;
    if (need_cal && fabsf(s.mpos[2] - cal.z0) > Z_TOL_MM)
        return snprintf(why, n, "Z moved since calibration (%.1f vs %.1f) - recalibrate", s.mpos[2], cal.z0), false;
#endif
    return true;
}

static void cancel_jog() {
    if (moving || fnc.stateIs("Jog")) {
        fnc.jogCancel();
        cancels++;
        cancel_ms = millis();
    }
    queue_end = millis();
    vx = vy = 0;
}

static void publish();

static void disable(const char* why) {
    cancel_jog();
    z_want = NAN;
    if (mode != Mode::Disabled) Serial.printf("tracking off: %s\n", why);
    mode = Mode::Disabled;
    strlcpy(reason, why, sizeof(reason));
}

static bool stop_button() {
    return digitalRead(PIN_STOP_BUTTON) == LOW;
}

// ---------------------------------------------------------------- jogging --

// Queue one relative jog. Returns the move actually sent (after soft limits).
// Z rides along towards z_want, at most z_up_feed / z_down_feed for the time
// the XY move takes; the feed is raised to the 3D length so XY keeps its speed.
static bool send_jog(float dx, float dy, float feed, float* sent_dx = nullptr, float* sent_dy = nullptr) {
    if (!moving) {
        pred[0] = fnc.status().mpos[0];
        pred[1] = fnc.status().mpos[1];
        pred_z  = fnc.status().mpos[2];
    }
    clamp_step(pred, dx, dy);
    if (sent_dx) *sent_dx = dx;
    if (sent_dy) *sent_dy = dy;
    float len = hypotf(dx, dy);
    if (len < 0.05f) return false;
    feed = fmaxf(feed, 10.0f);
    float dz = 0;
    if (!isnan(z_want)) {
        float e = z_want - pred_z, zmax = (e > 0 ? P.z_up_feed : P.z_down_feed) * len / feed;
        dz = fmaxf(-zmax, fminf(zmax, e));
    }
    char line[96];
    if (fabsf(dz) >= 0.01f)
        snprintf(line, sizeof(line), "$J=G91 G21 X%.3f Y%.3f Z%.3f F%.1f", dx, dy, dz, feed * hypotf(len, dz) / len);
    else
        snprintf(line, sizeof(line), "$J=G91 G21 X%.3f Y%.3f F%.1f", dx, dy, feed), dz = 0;
    if (!fnc.sendLine(line)) return false;
    uint32_t now = millis();
    pred[0] += dx;
    pred[1] += dy;
    pred_z += dz;
    queue_end    = (moving && (int32_t)(queue_end - now) > 0 ? queue_end : now) + (uint32_t)(len / feed * 60000.0f);
    moving       = true;
    last_send_ms = now;
    jogs_sent++;
    return true;
}

// Distance FluidNC still has to travel: end of the queued jogs minus where the
// arm really is. Measured, unlike queue_end, which assumes the jogs run at
// their feed and never learns when they run late.
static float backlog_mm() {
    if (!moving) return 0;
    const auto& s = fnc.status();
    return hypotf(pred[0] - s.mpos[0], pred[1] - s.mpos[1]);
}

// Called every loop: keep about one segment queued ahead.
static void dispatch() {
    float speed = hypotf(vx, vy);
    if (speed < 0.5f) return;  // nothing to do: let the queue drain
    uint32_t now      = millis();
    int32_t  left     = moving ? (int32_t)(queue_end - now) : 0;
    uint32_t horizon  = (uint32_t)(P.horizon_s * 1000);
    if (left > (int32_t)horizon / 2 || fnc.outstanding() >= 2) return;
    // A backlog in FluidNC delays every correction: with seconds of jogs queued
    // the arm circles round the target instead of reaching it.
    float seg = speed * P.horizon_s;
    if (backlog_mm() > fmaxf(1.5f * seg, 5.0f)) return;
    send_jog(vx * P.horizon_s, vy * P.horizon_s, speed * 60.0f);
}

// FluidNC Idle after our last send = queue really empty: resync the model.
static void resync_queue() {
    const auto& s = fnc.status();
    if (moving && fnc.stateIs("Idle") && fnc.outstanding() == 0 && s.t_ms > last_send_ms + 150) {
        moving = false;
        pred[0] = s.mpos[0];
        pred[1] = s.mpos[1];
        pred_z  = s.mpos[2];
    }
}

// ------------------------------------------------------ soft-limit recovery --

// FluidNC ready after boot or homing: one jog to the workspace centre.
// Arm outside the soft limits (after homing, a hand jog...): one jog straight
// to the nearest point LIM_MARGIN_MM inside them. Runs in every mode but
// calibration, tracking or not; tracking jogs wait until it is done. A jog
// FluidNC refuses (unreachable) is tried again every 2 s.
static void recover_step() {
    if (mode == Mode::Calibrating || !fnc.connected() || !status_fresh()) return;
    const auto& s = fnc.status();
    uint32_t now = millis();
    if (recovering) {
        if (!idle_after(recover_ms)) return;
        recovering = false;
        if (inside(s.mpos[0], s.mpos[1])) {
            Serial.printf("back inside the soft limits (%.0f, %.0f)\n", s.mpos[0], s.mpos[1]);
            if (mode == Mode::Disabled) strlcpy(reason, "moved back inside the soft limits", sizeof(reason));
        }
    }
    bool out = !inside(s.mpos[0], s.mpos[1]);
    if (!out && !center_pending) return;
    // wait for a hand jog or our own moves to end, and don't hammer a refused jog
    if (!fnc.stateIs("Idle") || fnc.outstanding() || moving || (out && now - recover_ms < 2000)) return;
    float tx, ty;
    // FluidNC ready after boot or homing: to the workspace centre (inside the limits too)
    if (center_pending) nearest_inside(P.center_x, P.center_y, LIM_MARGIN_MM, tx, ty);
    else nearest_inside(s.mpos[0], s.mpos[1], LIM_MARGIN_MM, tx, ty);
    char line[80];
    snprintf(line, sizeof(line), "$J=G91 G21 X%.3f Y%.3f F%.1f", tx - s.mpos[0], ty - s.mpos[1], P.recover_feed);
    if (!fnc.sendLine(line)) return;
    recovering = true;
    recover_ms = now;
    vx = vy = 0;
    if (center_pending)
        snprintf(reason, sizeof(reason), "machine ready - going to the workspace centre (%.0f, %.0f)", tx, ty);
    else
        snprintf(reason, sizeof(reason), "outside soft limits (%.0f, %.0f) - moving back to (%.0f, %.0f)", s.mpos[0],
                 s.mpos[1], tx, ty);
    center_pending = false;
    Serial.println(reason);
}

// ------------------------------------------------------ orbit and wandering --
//
// A target seen: the tool goes straight over it at fast_feed and follows it.
// Lost: the tool spirals round the last spot, from orbit_r in down to
// spiral_r_min, then back out, and so on, until a target is seen again. No target for
// wander_s: straight moves in random directions. All of it is jogs, so
// jog-cancel switches from one to the other at once.

static const float ORBIT_PIECE_S = 0.5f;
static const float WANDER_D_MIN = 200, WANDER_D_MAX = 800;  // mm per random move

// Last place the tool was sent for a target (floor mm), the orbit centre. Kept until reboot.
static bool  have_spot = false;
static float spot_x, spot_y;
static int   orbit_dir = 1;  // +1 counter-clockwise, picked at random for each new target
static float orbit_rad;      // mm: current spiral radius
static int   spiral_in = 1;  // 1 = radius shrinking (towards the target), 0 = growing back out

static float frand(float lo, float hi) { return lo + (hi - lo) * (esp_random() / 4294967296.0f); }

// After a jog-cancel FluidNC drops jogs sent before it has stopped: wait for Idle.
static bool cancel_pending() { return moving && (int32_t)(cancel_ms - last_send_ms) >= 0; }

static const float GOTO_PIECE_S = 0.25f;

// Straight towards the target spot at fast_feed, in short pieces from where
// the queued jogs end, so a moving target is followed. Stops inside db_mm_in
// of the spot (real tool position), moves again beyond db_mm_out. Returns
// true while over the target.
static bool goto_step() {
    const auto& st = fnc.status();
    float err = hypotf(spot_x - st.mpos[0], spot_y - st.mpos[1]);
    if (in_deadband ? err > P.db_mm_out : err < P.db_mm_in) in_deadband = !in_deadband;
    if (in_deadband || cancel_pending()) return in_deadband;
    uint32_t now   = millis();
    float    speed = P.fast_feed / 60.0f, piece = speed * GOTO_PIECE_S;
    if (moving && (int32_t)(queue_end - now) > (int32_t)(GOTO_PIECE_S * 500)) return false;
    if (fnc.outstanding() >= 2 || backlog_mm() > 2 * piece) return false;
    float px = moving ? pred[0] : st.mpos[0], py = moving ? pred[1] : st.mpos[1];
    float dx = spot_x - px, dy = spot_y - py, d = hypotf(dx, dy);
    if (d < 1.0f) return false;  // the queued jogs already end there
    float k = fminf(1.0f, piece / d);
    send_jog(k * dx, k * dy, P.fast_feed);
    return false;
}

// Z-only pieces towards z_want when no XY jog is carrying Z there (holding over
// the target, no target yet): z_chunk_s each, about one queued ahead. Short
// pieces so the next XY jog never waits long behind a Z-only one.
static void z_step() {
    if (isnan(z_want) || cancel_pending()) return;
    const auto& st = fnc.status();
    if (!moving) pred[0] = st.mpos[0], pred[1] = st.mpos[1], pred_z = st.mpos[2];
    float e = z_want - pred_z;
    if (fabsf(e) < 0.5f) return;
    uint32_t now = millis();
    if (moving && (int32_t)(queue_end - now) > (int32_t)(P.z_chunk_s * 500)) return;
    if (fnc.outstanding() >= 2) return;
    float zf = e > 0 ? P.z_up_feed : P.z_down_feed, piece = zf / 60.0f * P.z_chunk_s;
    float dz = fmaxf(-piece, fminf(piece, e));
    char  line[64];
    snprintf(line, sizeof(line), "$J=G91 G21 Z%.3f F%.1f", dz, zf);
    if (!fnc.sendLine(line)) return;
    pred_z += dz;
    queue_end    = (moving && (int32_t)(queue_end - now) > 0 ? queue_end : now) + (uint32_t)(fabsf(dz) / zf * 60000.0f);
    moving       = true;
    last_send_ms = now;
    jogs_sent++;
}

// One piece of the spiral, from where the queued jogs end to the next point of
// the spiral round the CURRENT spot: it follows a moving target, and when the
// tool is off the spiral (just arrived, target jumped) it first runs straight
// to it. Each piece turns by piece/r and changes the radius by
// spiral_pitch per full turn. About one piece is kept queued ahead.
static void orbit_step() {
    if (cancel_pending()) return;
    uint32_t now   = millis();
    float    speed = P.orbit_feed / 60.0f, piece = speed * ORBIT_PIECE_S;
    if (moving && (int32_t)(queue_end - now) > (int32_t)(ORBIT_PIECE_S * 500)) return;
    if (fnc.outstanding() >= 2 || backlog_mm() > 2 * piece) return;
    float px = moving ? pred[0] : fnc.status().mpos[0], py = moving ? pred[1] : fnc.status().mpos[1];
    // the whole spiral (its outer turn) inside the limits: pull the centre in if needed
    float cx, cy;
    nearest_inside(spot_x, spot_y, LIM_MARGIN_MM + P.orbit_r, cx, cy);
    float r_min = fminf(fmaxf(P.spiral_r_min, 1.0f), P.orbit_r);
    orbit_rad   = fminf(fmaxf(orbit_rad, r_min), P.orbit_r);
    float r     = orbit_rad;
    float dx = px - cx, dy = py - cy, d = hypotf(dx, dy);
    float a = d > 1.0f ? atan2f(dy, dx) : frand(-M_PI, M_PI);
    float tx, ty;
    if (fabsf(d - r) > piece) {
        float nx = cx + r * cosf(a), ny = cy + r * sinf(a), k = piece / hypotf(nx - px, ny - py);
        tx = px + k * (nx - px), ty = py + k * (ny - py);
    } else {
        float da = piece / r, a1 = a + orbit_dir * da;
        float r1 = r + (spiral_in ? -1 : 1) * P.spiral_pitch * da / (2 * (float)M_PI);
        if (r1 <= r_min) r1 = r_min, spiral_in = 0;
        if (r1 >= P.orbit_r) r1 = P.orbit_r, spiral_in = 1;
        tx = cx + r1 * cosf(a1), ty = cy + r1 * sinf(a1);
        orbit_rad = r1;
    }
    send_jog(tx - px, ty - py, P.orbit_feed);
}

// One straight move in a random direction to a random point WANDER_D_MIN..MAX
// away, the whole line wander_margin inside the limits; the next one when it is
// done. Outside that band (or boxed in): towards the workspace centre.
static void wander_step() {
    if (moving || recovering || !fnc.stateIs("Idle") || fnc.outstanding()) return;
    float px = fnc.status().mpos[0], py = fnc.status().mpos[1];
    for (int tries = 0; tries < 20; tries++) {
        float th = frand(-M_PI, M_PI), d = frand(WANDER_D_MIN, WANDER_D_MAX);
        float dx = d * cosf(th), dy = d * sinf(th);
        bool  ok = true;
        for (int k = 1; k <= 16 && ok; k++) ok = inside(px + dx * k / 16, py + dy * k / 16, fmaxf(P.wander_margin, LIM_MARGIN_MM));
        if (!ok) continue;
        Serial.printf("wander: %.0f mm towards %.0f deg\n", d, th * 57.2958f);
        send_jog(dx, dy, P.wander_feed);
        return;
    }
    // boxed in: towards the workspace centre
    Serial.println("wander: no free direction - towards the centre");
    float tx, ty;
    nearest_inside(P.center_x, P.center_y, LIM_MARGIN_MM, tx, ty);
    send_jog(tx - px, ty - py, P.wander_feed);
}

// --------------------------------------------------- FluidNC soft-limit alarm --

// ALARM:2: a move would have left FluidNC's own soft limits. FluidNC
// still knows the position, so unlock ($X) and jog back to the workspace
// centre; tracking waits for that jog as for recover_step(). If $X does not
// clear it, the usual path takes over (disable, then auto-enable resets and homes).
static void alarm_step() {
    if (mode == Mode::Calibrating || !fnc.connected() || !status_fresh()) return;
    uint32_t now = millis();
    if (!unlock_ms) {
        if (!fnc.stateIs("Alarm") || strcmp(fnc.lastAlarm(), "ALARM:2")) return;
        fnc.clearAlarm();
        moving = recovering = false;
        vx = vy = 0;
        if (!fnc.sendLine("$X")) return;
        unlock_ms = now;
        strlcpy(reason, "FluidNC soft limit (ALARM:2) - unlocking", sizeof(reason));
        Serial.println("FluidNC ALARM:2 (soft limit): $X, then back to the centre");
        return;
    }
    if (!idle_after(unlock_ms)) {
        if (now - unlock_ms > 2000) {
            unlock_ms = 0;
            Serial.printf("ALARM:2: $X did not unlock (state %s)\n", fnc.status().state);
        }
        return;
    }
    unlock_ms = 0;
    const auto& s = fnc.status();
    char line[80];
    snprintf(line, sizeof(line), "$J=G91 G21 X%.3f Y%.3f F%.1f", P.center_x - s.mpos[0], P.center_y - s.mpos[1],
             P.recover_feed);
    if (!fnc.sendLine(line)) return;
    recovering = true;
    recover_ms = now;
    snprintf(reason, sizeof(reason), "soft limit - going back to the centre (%.0f, %.0f)", P.center_x, P.center_y);
    Serial.println(reason);
}

// ------------------------------------------------------------------ lights --

// Automatic lights: a target seen -> spot on, light off, at once (but the light
// stays on at least light_hold_s once on); nothing seen for light_s -> light on, spot off. On at every boot; a manual light command
// turns it off (setAutoLights). The state is re-sent every 10 s: UDP can be
// lost and the light controller may have rebooted.
static volatile bool auto_lights = true;
enum class Lamp : uint8_t { Unknown, Spot, Light };
static Lamp     lamp = Lamp::Unknown;
static uint32_t lamp_sent_ms = 0, last_found_ms = 0;
static uint32_t light_on_ms = 0;  // the scene switched to Light
// What the scene calls for, whether or not auto lights are on: the lights
// follow it (when auto), and so does Z (Light = up at z_max, else z_min).
static Lamp     scene = Lamp::Unknown;

static void scene_step() {
    uint32_t now  = millis();
    Lamp     want = scene;
    if (mode == Mode::Wandering) want = Lamp::Spot;  // spot on, light off while wandering
    else if (last_found_ms && now - last_found_ms < (uint32_t)P.lost_ms) want = Lamp::Spot;
    else if (now - last_found_ms > (uint32_t)(P.light_s * 1000)) want = Lamp::Light;
    // the light, once on, stays on at least light_hold_s
    if (want == Lamp::Spot && scene == Lamp::Light && mode != Mode::Wandering &&
        now - light_on_ms < (uint32_t)(P.light_hold_s * 1000))
        want = scene;
    if (want == Lamp::Light && scene != Lamp::Light) light_on_ms = now;
    if (want != scene)
        Serial.printf("scene: %s\n", want == Lamp::Light       ? "no target - light, Z up"
                                      : mode == Mode::Wandering ? "wandering - spot, Z down"
                                                                : "target - spot, Z down");
    scene = want;
}

static void lights_step() {
    if (!auto_lights) {
        lamp = Lamp::Unknown;
        return;
    }
    uint32_t now = millis();
    if (scene == Lamp::Unknown || (scene == lamp && now - lamp_sent_ms < 10000)) return;
    lights::send(scene == Lamp::Spot ? "spot 1" : "light 1");
    lights::send(scene == Lamp::Spot ? "light 0" : "spot 0");
    lamp         = scene;
    lamp_sent_ms = now;
}

// "human 0": stop all motion, lights off, remember the state before; "human 1":
// back to it (tracking resumes unless the operator had stopped it). Repeats are ignored.
static void human_step() {
    char msg[64];
    if (!lights::receive(msg, sizeof(msg))) return;
    int v;
    if (sscanf(msg, " human %d", &v) != 1 || (v != 0 && v != 1)) return;
    if (v == 0 && !human_off) {
        saved_operator_off = operator_off;
        saved_auto_lights  = auto_lights;
        lights::query("state", saved_lamps, sizeof(saved_lamps));
        human_off   = true;
        auto_lights = false;
        disable("human 0 - everything off");
        strlcat(reason, " until human 1", sizeof(reason));
        lights::send("all 0");
        Serial.printf("human 0: everything off (lights were \"%s\", auto %d, operator stop %d)\n", saved_lamps,
                      saved_auto_lights, saved_operator_off);
    } else if (v == 1 && human_off) {
        human_off    = false;
        operator_off = saved_operator_off;
        auto_lights  = saved_auto_lights;
        int spot, light;
        if (auto_lights) lamp_sent_ms = millis() - 20000;  // send the scene at once
        else if (sscanf(saved_lamps, "spot %d light %d", &spot, &light) == 2) {
            lights::send(spot ? "spot 1" : "spot 0");
            lights::send(light ? "light 1" : "light 0");
        }
        auto_enable_next = millis();  // tracking back at once (unless the operator stopped it)
        strlcpy(reason, operator_off ? "human 1 - back (operator stop still on)" : "human 1 - back", sizeof(reason));
        Serial.printf("human 1: back to the state before (lights \"%s\", auto %d)\n", saved_lamps, auto_lights);
    }
}

// ---------------------------------------------------------------- tracking --

static void take_detection() {
    Detection d = vision::latest();
    if (d.seq == last_seq) return;
    last_seq = d.seq;
    if (last_frame_ms) {
        float dt = (d.t_cap - last_frame_ms) / 1000.0f;
        if (dt > 0) fps = fps ? 0.9f * fps + 0.1f / dt : 1.0f / dt;
    }
    last_frame_ms = d.t_cap;
    if (!d.found) return;
    last_found_ms = d.t_cap;
    bool reacquired = !have_filt || d.t_cap - last_seen > (uint32_t)P.lost_ms;
    if (reacquired) {
        fu = d.u;
        fv = d.v;
    } else {
        fu += P.alpha * (d.u - fu);
        fv += P.alpha * (d.v - fv);
    }
    have_filt = true;
    last_seen = d.t_cap;
}

static void start_tracking() {
    have_filt = in_deadband = false;
    vx = vy = 0;
    last_ctl_ms = enabled_ms = millis();
    mode        = Mode::Lost;  // until the first detection
    reason[0]   = 0;
}

static bool reset_and_home();

// FluidNC answers again after Waiting. Idle/Jog: carry on. Still running what
// it had queued: wait. Alarm (it rebooted), Hold, Door...: reset and home,
// tried again every 10 s if that fails.
static bool link_back() {
    uint32_t now = millis();
    if (fnc.stateIs("Run") || fnc.stateIs("Home"))
        return strlcpy(reason, "FluidNC back - waiting for it to finish its move", sizeof(reason)), false;
    if (!fnc.stateIs("Idle") && !fnc.stateIs("Jog")) {
        if ((int32_t)(now - rehome_next) < 0) return false;  // reason still says why the last try failed
        rehome_next = now + 10000;
        if (!reset_and_home()) return false;
    }
    Serial.println("FluidNC back: tracking resumes");
    start_tracking();
    return true;
}

static void track_step() {
    char why[64];
    uint32_t now = millis();
    // FluidNC silent: pause, but stay on and resume when it answers again.
    if (!fnc.connected() || !status_fresh()) {
        if (mode != Mode::Waiting) Serial.println("FluidNC link lost: waiting for it, tracking stays on");
        moving = recovering = false;
        vx = vy = 0;
        z_want    = NAN;
        queue_end = now;
        mode      = Mode::Waiting;
        strlcpy(reason, fnc.connected() ? "no status from FluidNC - retrying" : "FluidNC not connected - retrying",
                sizeof(reason));
        return;
    }
    if (mode == Mode::Waiting && !link_back()) return;
    if (unlock_ms) {  // alarm_step() is unlocking a soft-limit alarm
        vx = vy = 0;
        last_ctl_ms = now;
        return;
    }
    if (!motion_allowed(why, sizeof(why), true)) return disable(why);

    const auto& st0 = fnc.status();
    if (recovering || !inside(st0.mpos[0], st0.mpos[1])) {
        // recover_step() is bringing the arm back inside the soft limits
        vx = vy = 0;
        last_ctl_ms = now;
        return;  // reason set by recover_step()
    }

    // No target for wander_s (or nothing seen since tracking started): random moves.
    uint32_t unseen = now - (have_filt ? last_seen : enabled_ms);
    if (P.wander_s > 0 && (int32_t)unseen > (int32_t)(P.wander_s * 1000)) {
        if (mode != Mode::Wandering) {
            cancel_jog();
            Serial.println("no target: wandering");
        }
        mode = Mode::Wandering;
        snprintf(reason, sizeof(reason), "no target for %.0f s - wandering", P.wander_s);
        vx = vy = 0;
        last_ctl_ms = now;
#if CAMERA_FIXED
        z_want = scene == Lamp::Light ? P.z_max : P.z_min;
#endif
        wander_step();
        if (!moving) z_step();  // Z rides along with the random moves
        return;
    }

#if CAMERA_FIXED
    // Target seen: straight over it, fast. Lost: spiral round the last spot
    // until a target is seen again. Z: up while the scene calls for the light.
    z_want = scene == Lamp::Light ? P.z_max : P.z_min;
    bool seen = have_filt && now - last_seen <= (uint32_t)P.lost_ms;
    vx = vy = 0;
    last_ctl_ms = now;
    if (seen) {
        float gx, gy;
        if (!homography_apply(fmap.H, fu, fv, gx, gy)) return disable("target maps beyond the horizon");
        have_spot = true, spot_x = gx + P.off_x, spot_y = gy + P.off_y;
        if (mode != Mode::Tracking && mode != Mode::Holding) {
            cancel_jog();  // drop the queued spiral / random move: go at once
            in_deadband = false;
            Serial.printf("target at (%.0f, %.0f): going over it\n", spot_x, spot_y);
        }
        mode      = goto_step() ? Mode::Holding : Mode::Tracking;
        reason[0] = 0;
        z_step();
        return;
    }
    if (mode == Mode::Tracking || mode == Mode::Holding) {
        // a new spiral round the spot, from the outside in
        orbit_dir = (esp_random() & 1) ? 1 : -1;
        orbit_rad = P.orbit_r;
        spiral_in = 1;
        Serial.printf("target lost: spiralling round the last spot %s\n", orbit_dir > 0 ? "CCW" : "CW");
    }
    mode = Mode::Lost;
    strlcpy(reason, have_spot ? "target lost - spiralling round the last spot" : "no target yet", sizeof(reason));
    if (have_spot) orbit_step();
    z_step();
    return;
#else
    // Forearm camera: centre the target in the image (image Jacobian).
    bool lost = false;
    if (!have_filt || now - last_seen > (uint32_t)P.lost_ms) {
        if (mode != Mode::Lost) {
            cancel_jog();
            mode = Mode::Lost;
            Serial.println("target lost: jog cancelled, holding");
        }
        vx = vy = 0;
        last_ctl_ms = now;
        return;
    }

    float tx = 0, ty = 0;
    have_spot = true, spot_x = fnc.status().mpos[0], spot_y = fnc.status().mpos[1];
    float err = hypotf(fu, fv);
    if (in_deadband ? err > P.db_out : err < P.db_in) in_deadband = !in_deadband;

    if (!in_deadband) {
        const float(*J)[2] = cal.J;
        float det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
        // move that would bring the target to the centre, calibration frame
        float dx = -(J[1][1] * fu - J[0][1] * fv) / det;
        float dy = -(-J[1][0] * fu + J[0][0] * fv) / det;
#if JOG_CARTESIAN && CAMERA_ROTATES_WITH_FOREARM
        const auto& s = fnc.status();
        float a = forearm_angle(s.mpos[0], s.mpos[1]) - cal.phi0;
        float c = cosf(a), sn = sinf(a);
        float rx = c * dx - sn * dy;
        dy       = sn * dx + c * dy;
        dx       = rx;
#endif
        tx = P.kp * dx;
        ty = P.kp * dy;
        float sp = hypotf(tx, ty);
        if (sp > P.vmax) {
            tx *= P.vmax / sp;
            ty *= P.vmax / sp;
        }
    }

    // slew limit: the 5 kg payload should not see step changes in velocity
    float dt = fminf((now - last_ctl_ms) / 1000.0f, 0.2f);
    last_ctl_ms = now;
    float ax = tx - vx, ay = ty - vy, a = hypotf(ax, ay), amax = P.amax * dt;
    if (a > amax) {
        ax *= amax / a;
        ay *= amax / a;
    }
    vx += ax;
    vy += ay;

    mode = lost ? Mode::Lost : (in_deadband && hypotf(vx, vy) < 0.5f) ? Mode::Holding : Mode::Tracking;
    if (lost)
        strlcpy(reason, in_deadband ? "target lost - holding at the last seen point" : "target lost - going to the last seen point",
                sizeof(reason));
    else
        reason[0] = 0;
    dispatch();
#endif
}

static void enable() {
    char why[64];
#if CAMERA_FIXED
    bool have_cal = fmap.solved;
#else
    bool have_cal = cal_valid;
#endif
    if (have_cal && (!fnc.connected() || !status_fresh())) {
        // FluidNC not there yet: tracking is on, and starts when it answers
        start_tracking();
        mode = Mode::Waiting;
        strlcpy(reason, "waiting for FluidNC", sizeof(reason));
        Serial.println("tracking on, waiting for FluidNC");
        return;
    }
    // not Idle (Alarm, Hold...): reset and home. Our own jog back inside the limits is fine.
    if (fnc.connected() && status_fresh() && !fnc.stateIs("Idle") && !(recovering && fnc.stateIs("Jog")) &&
        !reset_and_home()) {
        if (!fnc.connected()) {
            start_tracking();
            mode = Mode::Waiting;
        }
        return;
    }
    if (!motion_allowed(why, sizeof(why), true)) {
        strlcpy(reason, why, sizeof(reason));
        Serial.printf("enable refused: %s\n", why);
        return;
    }
    start_tracking();
    Serial.println("tracking on");
}

// ------------------------------------------------------------- calibration --

static bool cal_abort = false;

// Keep the link alive and watch for stop requests while calibration waits.
static bool pump(uint32_t ms) {
    uint32_t end = millis() + ms;
    do {
        vision::wait_new(20);
        fnc.poll();
        if (req_disable || stop_button()) cal_abort = true;
        if (!fnc.connected() || fnc.stateIs("Alarm") || fnc.stateIs("Door")) cal_abort = true;
        publish();
    } while (!cal_abort && (int32_t)(end - millis()) > 0);
    return !cal_abort;
}

static void cal_note(const char* fmt, ...);

// FluidNC not Idle (Alarm, Hold, Door, a stray Jog...): soft reset, home, and
// wait for Idle. The reset also loses the position, so homing is needed anyway.
// Stop button / Disable aborts with another reset, which stops the homing.
static bool reset_and_home() {
    Serial.printf("fluidnc %s %s: reset + home\n", fnc.status().state, fnc.lastAlarm());
    snprintf(reason, sizeof(reason), "FluidNC was %s - resetting and homing", fnc.status().state);
    // light on while homing; automatic lights carry on from there
    lights::send("light 1");
    scene = Lamp::Light;
    light_on_ms = millis();
    if (auto_lights) lamp = Lamp::Light, lamp_sent_ms = light_on_ms;
    cancel_jog();
    moving = false;
    fnc.softReset();
    // after the reset FluidNC prints its banner and comes back in Alarm (or Idle)
    uint32_t t0 = millis();
    do {
        vision::wait_new(20);
        fnc.poll();
        publish();
    } while (millis() - t0 < 1000);
    if (!fnc.sendLine("$H")) return strlcpy(reason, "homing: FluidNC not connected", sizeof(reason)), false;
    // wait for Idle; $H answers 'ok' only once homing is done
    t0 = millis();
    for (;;) {
        vision::wait_new(20);
        fnc.poll();
        publish();
        if (req_disable || stop_button()) {
            fnc.softReset();
            return strlcpy(reason, "homing aborted", sizeof(reason)), false;
        }
        if (!fnc.connected()) return strlcpy(reason, "homing: FluidNC disconnected", sizeof(reason)), false;
        const auto& s = fnc.status();
        bool done = fnc.outstanding() == 0 && s.t_ms > t0 + 500;
        if (done && fnc.stateIs("Idle")) break;
        if (done && fnc.stateIs("Alarm"))
            return snprintf(reason, sizeof(reason), "homing failed %s", fnc.lastAlarm()), false;
        if (millis() - t0 > HOME_TIMEOUT_MS) {
            fnc.softReset();
            return strlcpy(reason, "homing timed out", sizeof(reason)), false;
        }
    }
    Serial.println("fluidnc: homed, Idle");
    center_pending = true;  // recover_step() takes the tool to the workspace centre
    reason[0] = 0;
    return true;
}

// Detection is allowed to be patchy during calibration: the target only has to
// show up in a few frames, and the confidence threshold is lowered (see
// detectionThreshold()). The median rejects the odd false detection.
static const int      CAL_MIN_HITS   = 3;     // detections needed per measurement
static const int      CAL_MAX_FRAMES = 40;    // frames looked at before giving up
static const uint32_t CAL_MEASURE_MS = 8000;  // ...or this long
static int   meas_found = 0, meas_frames = 0;  // last measurement, for the error message
static float meas_sharp = 0;                    // its mean image sharpness

static float median(float* a, int n) {
    std::sort(a, a + n);
    return n & 1 ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

// Median target position over frames grabbed after t_after.
static bool measure(uint32_t t_after, float& u, float& v) {
    float us[CAL_MAX_FRAMES], vs[CAL_MAX_FRAMES];
    meas_found = meas_frames = 0;
    float sharp_sum = 0;
    uint32_t deadline = millis() + CAL_MEASURE_MS;
    // a few more hits than the minimum if they come quickly, for a steadier median
    while (meas_found < 2 * CAL_MIN_HITS + 2 && meas_frames < CAL_MAX_FRAMES && (int32_t)(deadline - millis()) > 0) {
        if (!pump(30)) return false;
        Detection d = vision::latest();
        if (d.seq == last_seq || d.t_cap < t_after) continue;
        last_seq = d.seq;
        meas_frames++;
        sharp_sum += d.sharp;
        if (d.found) {
            us[meas_found] = d.u;
            vs[meas_found] = d.v;
            meas_found++;
        }
    }
    meas_sharp = meas_frames ? sharp_sum / meas_frames : 0;
    cal_note("seen %d/%d sharp %.1f", meas_found, meas_frames, meas_sharp);
    if (meas_found < CAL_MIN_HITS) return false;
    u = median(us, meas_found);
    v = median(vs, meas_found);
    return true;
}

static const char* measure_why(const char* what) {
    static char b[64];
    snprintf(b, sizeof(b), "%s (%d/%d frames)", what, meas_found, meas_frames);
    return b;
}

// What calibration did, shown on the dashboard: commanded vs real moves, image shifts.
static char cal_log[320] = "";

static void cal_note(const char* fmt, ...) {
    char    line[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    Serial.printf("cal: %s\n", line);
    if (cal_log[0]) strlcat(cal_log, " | ", sizeof(cal_log));
    strlcat(cal_log, line, sizeof(cal_log));
}

// Jog by (dx, dy), wait until FluidNC is Idle again, and report the move the
// arm REALLY made, from MPos before and after (FluidNC may shorten or reject a jog).
static bool move_and_wait(float dx, float dy, float& mx, float& my) {
    moving = false;  // start from the reported position
    float x0 = fnc.status().mpos[0], y0 = fnc.status().mpos[1], sdx, sdy;
    if (!send_jog(dx, dy, P.cal_feed, &sdx, &sdy)) return false;
    uint32_t t0 = millis();
    uint32_t timeout = (uint32_t)(hypotf(sdx, sdy) / P.cal_feed * 60000.0f) + 5000;
    while (true) {
        if (!pump(20)) return false;
        const auto& s = fnc.status();
        if (fnc.outstanding() == 0 && fnc.stateIs("Idle") && s.t_ms > t0 + 200) break;
        if (millis() - t0 > timeout) return false;
    }
    moving = false;
    if (!pump(CAL_SETTLE_MS)) return false;  // Idle != still: let the arm stop swinging
    mx = fnc.status().mpos[0] - x0;
    my = fnc.status().mpos[1] - y0;
    return true;
}

static void cal_fail(const char* why) {
    cancel_jog();
    mode = Mode::Disabled;
    snprintf(reason, sizeof(reason), "calibration failed: %s", why);
    Serial.println(reason);
}

static void calibrate() {
    char why[64];
    if (!motion_allowed(why, sizeof(why), false) || !fnc.stateIs("Idle")) {
        snprintf(reason, sizeof(reason), "can't calibrate: %s", fnc.stateIs("Jog") ? "arm moving" : why);
        return;
    }
    if (!inside(fnc.status().mpos[0], fnc.status().mpos[1])) {
        strlcpy(reason, "can't calibrate: arm outside the soft limits - it is being moved back", sizeof(reason));
        return;
    }
    mode      = Mode::Calibrating;
    cal_abort = false;
    strlcpy(reason, "calibrating - keep the wormbot still", sizeof(reason));
    cal_log[0] = 0;
    Serial.println(reason);

    float bu, bv;
    if (!pump(300) || !measure(millis(), bu, bv)) return cal_fail(cal_abort ? "aborted" : measure_why("target not seen at start"));
    const auto& s = fnc.status();
    Calibration c = {};
    c.magic = CAL_MAGIC;
    c.z0    = s.mpos[2];
    c.phi0  = JOG_CARTESIAN ? forearm_angle(s.mpos[0], s.mpos[1]) : 0.0f;

    // One move per axis. D holds the real arm moves (columns), S the image shifts.
    float D[2][2], S[2][2];
    for (int axis = 0; axis < 2; axis++) {
        // Try bigger steps while the image shift is small; keep the best usable one.
        struct { bool ok; float mx, my, du, dv, shift; } best = {};
        float step = P.cal_step;
        for (int attempt = 0; attempt < 5; attempt++) {
            float dx = axis == 0 ? step : 0, dy = axis == 1 ? step : 0, mx, my, bx, by, u, v;
            if (!move_and_wait(dx, dy, mx, my)) return cal_fail(cal_abort ? "aborted" : "move failed (soft limit?)");
            bool seen = measure(millis(), u, v);
            if (!move_and_wait(-mx, -my, bx, by)) return cal_fail(cal_abort ? "aborted" : "move back failed");
            float moved   = hypotf(mx, my);
            bool  clipped = moved < 0.9f * step;  // soft limit, reach, or FluidNC shortened the jog
            cal_note("%c asked %.0f moved %.0f", "XY"[axis], step, moved);
            if (moved < 1.0f) return cal_fail("arm did not move - FluidNC refused the jog?");
            if (!seen) {
                cal_note("target lost");
                if (best.ok) break;  // the previous, smaller step was fine
                step *= 0.5f;
                if (step < 2.0f) return cal_fail("target leaves view on any move");
                continue;
            }
            float du = u - bu, dv = v - bv, shift = hypotf(du, dv);
            cal_note("shift %.3f", shift);
            if (shift > best.shift) best = {true, mx, my, du, dv, shift};
            // stop growing: big enough, can't go further, or already 4x the configured step
            if (shift >= 0.15f || clipped || step >= 4 * P.cal_step) break;
            step *= 2;
        }
        if (!best.ok) return cal_fail("target never seen after a move");
        if (best.shift < 0.03f) {
            char b[64];
            snprintf(b, sizeof(b), "%c: image moved only %.3f", "XY"[axis], best.shift);
            return cal_fail(b);
        }
        D[0][axis] = best.mx;
        D[1][axis] = best.my;
        S[0][axis] = best.du;
        S[1][axis] = best.dv;
    }
    // J D = S  =>  J = S D^-1 (real moves need not be exactly along X and Y)
    float dd = D[0][0] * D[1][1] - D[0][1] * D[1][0];
    if (fabsf(dd) < 1e-3f) return cal_fail("X and Y moves were parallel");
    float Di[2][2] = {{D[1][1] / dd, -D[0][1] / dd}, {-D[1][0] / dd, D[0][0] / dd}};
    for (int r = 0; r < 2; r++)
        for (int k = 0; k < 2; k++) c.J[r][k] = S[r][0] * Di[0][k] + S[r][1] * Di[1][k];

    // Back where we started, the target must be back where it was.
    float eu, ev;
    if (!measure(millis(), eu, ev)) return cal_fail(cal_abort ? "aborted" : measure_why("target lost after calibration"));
    if (hypotf(eu - bu, ev - bv) > 0.08f) return cal_fail("target moved, or backlash - retry");

    // The two columns must not be parallel.
    float n0 = hypotf(c.J[0][0], c.J[1][0]), n1 = hypotf(c.J[0][1], c.J[1][1]);
    float det = c.J[0][0] * c.J[1][1] - c.J[0][1] * c.J[1][0];
    if (fabsf(det) / (n0 * n1) < 0.3f) return cal_fail("X and Y move the image the same way");
    cal_note("raw J %.4f %.4f %.4f %.4f", c.J[0][0], c.J[0][1], c.J[1][0], c.J[1][1]);
    orthogonalise(c.J);

    cal       = c;
    cal_valid = true;
    nvs_save_cal();
    mode = Mode::Disabled;
    snprintf(reason, sizeof(reason), "calibrated - press Enable to track");
    Serial.printf("calibrated: J = [%.5f %.5f; %.5f %.5f] /unit, phi0 %.1f deg, z0 %.1f\n", c.J[0][0], c.J[0][1],
                  c.J[1][0], c.J[1][1], c.phi0 * 57.2958f, c.z0);
}

// J from the dashboard instead of measured: an angle and a scale, square axes.
static void manual_cal(float th, float k) {
    if (!fnc.connected() || !status_fresh()) {
        snprintf(reason, sizeof(reason), "can't set calibration: %s",
                 fnc.connected() ? "no status from FluidNC" : "FluidNC not connected");
        return;
    }
    const auto& s = fnc.status();
    Calibration c = {};
    c.magic = CAL_MAGIC;
    c.z0    = s.mpos[2];
    c.phi0  = JOG_CARTESIAN ? forearm_angle(s.mpos[0], s.mpos[1]) : 0.0f;
    float co = k * cosf(th), sn = k * sinf(th);
#if CAM_MIRRORED
    c.J[0][0] = co, c.J[0][1] = -sn, c.J[1][0] = sn, c.J[1][1] = co;
#else
    c.J[0][0] = co, c.J[0][1] = sn, c.J[1][0] = sn, c.J[1][1] = -co;
#endif
    cal       = c;
    cal_valid = true;
    nvs_save_cal();
    snprintf(cal_log, sizeof(cal_log), "manual: X at %.1f deg, %.4f/100mm, phi0 %.1f deg", th * 57.2958f,
             k * 100, c.phi0 * 57.2958f);
    snprintf(reason, sizeof(reason), "calibration set by hand - press Enable to track");
    Serial.println(cal_log);
}

// Forget the calibration, in RAM and in NVS. Tracking can't be enabled until
// a new one is made (Calibrate or manual).
static void clear_cal() {
    cal       = {};
    cal_valid = false;
    Preferences pr;
    if (pr.begin("track", false)) {
        pr.remove("cal");
        pr.end();
    }
    cal_log[0] = 0;
    strlcpy(reason, "calibration cleared - not calibrated", sizeof(reason));
    Serial.println(reason);
}

// ---------------------------------------------------- floor map (fixed camera) --
//
// Teaching one point:
//   1. mark the target's spot in the image: measured from the detection (the
//      head must be visible and still), or clicked on the dashboard;
//   2. jog the tool over that spot with FluidNC, then record: the current arm
//      XY is paired with the marked image point.
// Solve fits H to all the points and reports how far each one is off, in mm.

static void map_note(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(cal_log, sizeof(cal_log), fmt, ap);
    va_end(ap);
    Serial.printf("map: %s\n", cal_log);
}

static void map_mark_measured() {
    // measure() is shared with the Jacobian calibration: same lowered threshold
    // and median over a few frames, while the mode says Calibrating
    mode       = Mode::Calibrating;
    cal_abort  = false;
    cal_log[0] = 0;
    float u, v;
    bool ok = measure(millis(), u, v);
    mode = Mode::Disabled;
    if (!ok) return map_note("mark failed: %s", cal_abort ? "aborted" : measure_why("target not seen"));
    mark_set = true, mark_u = u, mark_v = v;
    map_note("marked image point (%.3f, %.3f) - now jog the tool over it and Record", u, v);
}

static void map_record() {
    if (!mark_set) return map_note("record: mark the image point first");
    if (fmap.n >= MAP_MAX_PTS) return map_note("record: %d points already - delete one", MAP_MAX_PTS);
    if (!fnc.connected() || !status_fresh() || !fnc.stateIs("Idle"))
        return map_note("record: arm must be connected and Idle (state %s)", fnc.status().state);
    const auto& s = fnc.status();
    int i = fmap.n++;
    fmap.u[i] = mark_u, fmap.v[i] = mark_v;
    fmap.x[i] = s.mpos[0], fmap.y[i] = s.mpos[1];
    fmap.err[i] = NAN;
    fmap.solved = false;  // points changed: solve again
    mark_set    = false;
    nvs_save_map();
    map_note("point %d: image (%.3f, %.3f) = arm (%.0f, %.0f) mm. %s", i + 1, mark_u, mark_v, s.mpos[0], s.mpos[1],
             fmap.n < 4 ? "Need at least 4." : "Solve when you have enough.");
}

static void map_delete(int i) {
    if (i < 0 || i >= fmap.n) return map_note("delete: no point %d", i + 1);
    for (int k = i; k < fmap.n - 1; k++) {
        fmap.u[k] = fmap.u[k + 1], fmap.v[k] = fmap.v[k + 1];
        fmap.x[k] = fmap.x[k + 1], fmap.y[k] = fmap.y[k + 1];
        fmap.err[k] = fmap.err[k + 1];
    }
    fmap.n--;
    fmap.solved = false;
    nvs_save_map();
    map_note("point %d deleted, %d left - solve again", i + 1, (int)fmap.n);
}

static void map_solve() {
    if (fmap.n < 4) return map_note("solve: need at least 4 points, have %d", (int)fmap.n);
    float H[3][3], rms;
    if (!homography_fit(fmap.n, fmap.u, fmap.v, fmap.x, fmap.y, H, fmap.err, &rms))
        return map_note("solve failed: points too close to a line - spread them over the workspace");
    memcpy(fmap.H, H, sizeof(H));
    fmap.rms    = rms;
    fmap.solved = true;
    nvs_save_map();
    operator_off = false;  // taught and solved: tracking may start by itself
    int worst = 0;
    for (int i = 1; i < fmap.n; i++)
        if (fmap.err[i] > fmap.err[worst]) worst = i;
    if (fmap.n == 4)
        map_note("solved with 4 points: exact fit, so no error check - add more points to check it");
    else
        map_note("solved: %d points, rms %.0f mm, worst point %d off by %.0f mm", (int)fmap.n, rms, worst + 1,
                 fmap.err[worst]);
}

static void map_serve(MapReq r) {
    if (mode != Mode::Disabled) return map_note("stop tracking before teaching the floor map");
    switch (r) {
        case MapReq::Mark: map_mark_measured(); break;
        case MapReq::MarkAt:
            mark_set = true, mark_u = req_map_u, mark_v = req_map_v;
            map_note("marked image point (%.3f, %.3f) - now jog the tool over it and Record", mark_u, mark_v);
            break;
        case MapReq::Record: map_record(); break;
        case MapReq::Delete: map_delete(req_map_i); break;
        case MapReq::Solve: map_solve(); break;
        case MapReq::Clear:
            fmap     = {};
            mark_set = false;
            nvs_save_map();
            map_note("floor map cleared");
            break;
        default: break;
    }
}

// ------------------------------------------------------------------- task --

static void publish() {
    CtlStatus s = {};
    s.mode          = mode;
    strlcpy(s.reason, reason, sizeof(s.reason));
    strlcpy(s.fnc_state, fnc.status().state, sizeof(s.fnc_state));
    s.fnc_connected = fnc.connected();
    s.fnc_reconnects = fnc.reconnects();
    s.operator_off   = operator_off;
    s.human_off      = human_off;
    s.auto_lights    = auto_lights;
    s.z_goal = z_want;
    strlcpy(s.lamp, lamp == Lamp::Spot ? "spot" : lamp == Lamp::Light ? "light" : "-", sizeof(s.lamp));
    memcpy(s.mpos, fnc.status().mpos, sizeof(s.mpos));
    if (mode == Mode::Calibrating) {
        // the tracking filter is not running: show the raw detection, live
        Detection d = vision::latest();
        s.fu = d.found ? d.u : NAN;
        s.fv = d.found ? d.v : NAN;
    } else {
        s.fu = have_filt ? fu : NAN;
        s.fv = have_filt ? fv : NAN;
    }
    s.vx = vx;
    s.vy = vy;
    s.calibrated = cal_valid;
    s.cal        = cal;
    // J(phi) = J0 R(phi0 - phi): the axes the control law is using right now
    float b = 0;
    s.phi   = NAN;
#if JOG_CARTESIAN && CAMERA_ROTATES_WITH_FOREARM
    s.phi = forearm_angle(s.mpos[0], s.mpos[1]);
    if (cal_valid) b = cal.phi0 - s.phi;
#endif
    float c = cosf(b), sn = sinf(b);
    for (int r = 0; r < 2; r++) {
        s.Jnow[r][0] = cal.J[r][0] * c + cal.J[r][1] * sn;
        s.Jnow[r][1] = -cal.J[r][0] * sn + cal.J[r][1] * c;
    }
    s.jogs_sent  = jogs_sent;
    s.cancels    = cancels;
    s.backlog_mm = backlog_mm();
    s.fps        = fps;
    strlcpy(s.cal_log, cal_log, sizeof(s.cal_log));
    s.map      = fmap;
    s.mark_set = mark_set;
    s.mark_u = mark_u, s.mark_v = mark_v;
    s.target_x = s.target_y = NAN;
    if (fmap.solved && !isnan(s.fu)) {
        float x, y;
        if (homography_apply(fmap.H, s.fu, s.fv, x, y)) s.target_x = x, s.target_y = y;
    }
    portENTER_CRITICAL(&mux);
    pub = s;
    portEXIT_CRITICAL(&mux);
}

static void led() {
    bool on;
    switch (mode) {
        case Mode::Tracking:
        case Mode::Holding: on = true; break;
        case Mode::Lost:
        case Mode::Wandering:
        case Mode::Calibrating: on = (millis() / 150) & 1; break;
        case Mode::Waiting: on = (millis() / 600) & 1; break;
        default: on = false;
    }
    digitalWrite(PIN_LED, on ? LOW : HIGH);
}

static void task(void*) {
    fnc.begin(FLUIDNC_HOST, FLUIDNC_PORT);
    for (;;) {
        vision::wait_new(20);
        fnc.poll();
        take_detection();
        resync_queue();

        if (req_disable || stop_button()) {
            if (!operator_off) Serial.println("operator stop: no automatic motion until reboot or Enable");
            operator_off = true;
            disable(req_disable ? req_why : "stop button");
            strlcat(reason, " - off until reboot or Enable", sizeof(reason));
            req_disable = req_enable = req_cal = req_man = req_clear = false;
            req_map = MapReq::None;
        }
        if (req_cal) {
            req_cal = false;
            if (mode == Mode::Disabled) calibrate();
        }
        if (req_map != MapReq::None) {
            MapReq r = req_map;
            req_map  = MapReq::None;
            map_serve(r);
        }
        if (req_clear) {
            req_clear = false;
            if (mode == Mode::Disabled) clear_cal();
            else strlcpy(reason, "stop tracking before clearing the calibration", sizeof(reason));
        }
        if (req_man) {
            req_man = false;
            if (mode == Mode::Disabled) manual_cal(man_th, man_k);
            else strlcpy(reason, "stop tracking before setting the calibration", sizeof(reason));
        }
        human_step();
        if (req_enable) {
            req_enable   = false;
            operator_off = false;
            human_off    = false;  // the operator's Enable overrides "human 0"
            if (mode == Mode::Disabled) enable();
        }
#if CAMERA_FIXED
        bool have_cal = fmap.solved;
#else
        bool have_cal = cal_valid;
#endif
        if (!operator_off && !human_off && have_cal && mode == Mode::Disabled && !unlock_ms && !recovering &&
            (int32_t)(millis() - auto_enable_next) >= 0) {
            auto_enable_next = millis() + AUTO_ENABLE_RETRY_MS;
            Serial.println("auto-enable");
            enable();
        }
        if (!operator_off && !human_off) {
            alarm_step();
            recover_step();
        }
        scene_step();
        lights_step();
        if (mode != Mode::Disabled && mode != Mode::Calibrating) track_step();

        led();
        publish();
    }
}

// --------------------------------------------------------------------- API --

void begin() {
    pinMode(PIN_STOP_BUTTON, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    nvs_load();
    Serial.printf("calibration: %s\n", cal_valid ? "loaded" : "none - run Calibrate");
    publish();
    xTaskCreatePinnedToCore(task, "control", 8192, nullptr, 5, nullptr, 0);
}

void requestEnable() { req_enable = true; }
void requestCalibrate() { req_cal = true; }
void requestClearCal() { req_clear = true; }
void requestMark() { req_map = MapReq::Mark; }
void requestMarkAt(float u, float v) {
    req_map_u = u, req_map_v = v;
    req_map   = MapReq::MarkAt;
}
void requestRecord() { req_map = MapReq::Record; }
void requestDeletePoint(int i) {
    req_map_i = i;
    req_map   = MapReq::Delete;
}
void requestSolveMap() { req_map = MapReq::Solve; }
void requestClearMap() { req_map = MapReq::Clear; }
void requestManualCal(float th_deg, float k) {
    man_th = th_deg / 57.2958f;
    man_k  = k;
    req_man = true;
}
void setAutoLights(bool on) {
    if (on && !auto_lights) lamp_sent_ms = millis() - 20000;  // send the current state at once
    auto_lights = on;
}

void requestDisable(const char* why) {
    strlcpy(req_why, why, sizeof(req_why));
    req_disable = true;
}

CtlStatus status() {
    portENTER_CRITICAL(&mux);
    CtlStatus s = pub;
    portEXIT_CRITICAL(&mux);
    return s;
}

Params params() {
    portENTER_CRITICAL(&mux);
    Params p = P;
    portEXIT_CRITICAL(&mux);
    return p;
}

bool setParam(const char* name, float value) {
    for (const auto& d : PARAM_DEFS) {
        if (strcmp(d.name, name)) continue;
        portENTER_CRITICAL(&mux);
        P.*d.field = fmaxf(d.lo, fminf(d.hi, value));
        if (P.db_out < P.db_in) P.db_out = P.db_in;
        portEXIT_CRITICAL(&mux);
        nvs_save_params();
        return true;
    }
    return false;
}

float detectionThreshold() {
    // Calibration only needs a few sightings of a still target: accept weaker ones.
    float c = P.min_conf;
    return mode == Mode::Calibrating ? fmaxf(0.2f, 0.6f * c) : c;
}

const char* modeName(Mode m) {
    switch (m) {
        case Mode::Disabled: return "disabled";
        case Mode::Tracking: return "tracking";
        case Mode::Holding: return "holding";
        case Mode::Lost: return "lost";
        case Mode::Calibrating: return "calibrating";
        case Mode::Waiting: return "waiting";
        case Mode::Wandering: return "wandering";
    }
    return "?";
}

}  // namespace controller
