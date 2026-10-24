#pragma once
#include <Arduino.h>

// Runtime-tunable, persisted in NVS (namespace "track").
struct Params {
    float kp;         // 1/s: fraction of the centring move commanded per second
    float vmax;       // mm/s (deg/s in joint mode): speed cap
    float amax;       // mm/s^2: slew limit on the commanded velocity
    float db_in;      // deadband, normalised image units: stop correcting below this...
    float db_out;     // ...and resume above this (hysteresis, db_out > db_in)
    float alpha;      // EMA weight of a new measurement, 0..1 (1 = no smoothing)
    float min_conf;   // FOMO cell confidence threshold
    float lost_ms;    // no detection for this long = target lost -> jog cancel
    float horizon_s;  // duration of each jog segment
    float cal_step;   // initial calibration move, mm (deg)
    float cal_feed;   // calibration feed, mm/min (deg/min)
    // fixed camera (CAMERA_FIXED) only; appended so older saved params still load
    float off_x;      // mm: where the tool goes relative to the target
    float off_y;
    float db_mm_in;   // deadband, mm: stop correcting below this...
    float db_mm_out;  // ...and resume above this
    // appended later, same reason
    float recover_feed;  // mm/min: jog back inside the soft limits
    float wander_s;      // s without a target before drawing arcs (0 = never)
    float wander_feed;   // mm/min along the arcs
    float center_x;      // mm: workspace centre - circles before any target, ALARM:2 return
    float center_y;
    float light_s;       // s without a target before automatic lights switch spot -> light
    float orbit_r;       // mm: outer radius of the spiral round a target (fixed camera)
    float orbit_feed;    // mm/min along the spiral
    float spiral_pitch;  // mm the radius changes per turn
    float spiral_r_min;  // mm: inner radius, where the spiral turns back out
    float light_hold_s;  // s the light stays on at least before the spot may come back
    float fast_feed;     // mm/min: straight move over a detected target
    float z_max;         // mm: Z while the general light is on (no target)
    float z_min;         // mm: Z otherwise (target seen)
    float z_up_feed;     // mm/min: Z going up
    float z_down_feed;   // mm/min: Z going down
    float wander_margin; // mm: random moves stay this far inside the soft limits
    float z_chunk_s;     // s: length of each Z-only jog piece
};

// Fixed camera: taught pairs (image point, arm XY) and the floor map fitted to
// them. H maps the image to the arm: [x y w]^T = H [u v 1]^T, x/w, y/w in mm.
static const int MAP_MAX_PTS = 12;
struct FloorMap {
    uint32_t magic;
    int32_t  n;                      // taught points
    float    u[MAP_MAX_PTS], v[MAP_MAX_PTS];  // image, -1..1
    float    x[MAP_MAX_PTS], y[MAP_MAX_PTS];  // arm MPos, mm
    float    err[MAP_MAX_PTS];       // residual of each point after the fit, mm
    bool     solved;
    float    H[3][3];
    float    rms;                    // mm
    float    z0;                     // unused (wall camera), kept so the saved map still loads
};

// Image Jacobian, persisted. du/dx etc., normalised image units per mm (deg).
struct Calibration {
    uint32_t magic;
    float    J[2][2];   // [du dv]^T = J [dx dy]^T, measured at pose phi0
    float    phi0;      // forearm angle at calibration, rad
    float    z0;        // Z at calibration, mm
};

// Waiting: tracking is on but FluidNC does not answer; resumes by itself.
// Wandering: no target for wander_s, the tool moves in random directions.
// Fixed camera: Tracking = moving over the target, Holding = over it,
// Lost = spiralling round the last spot.
enum class Mode : uint8_t { Disabled, Tracking, Holding, Lost, Calibrating, Waiting, Wandering };

// Snapshot for the dashboard.
struct CtlStatus {
    Mode     mode;
    char     reason[64];   // why we are not tracking
    char     fnc_state[12];
    bool     fnc_connected;
    uint32_t fnc_reconnects;
    bool     operator_off;  // stopped by the operator: no automatic motion until reboot or Enable
    bool     human_off;     // "human 0" received: everything off until "human 1"
    bool     auto_lights;   // automatic light control (off after a manual light command, on at boot)
    char     lamp[8];       // what automatic control last set: "spot", "light", "-"
    float    z_goal;        // Z the tracker is taking the tool to, NAN = leaves Z alone
    float    mpos[3];
    float    fu, fv;       // filtered target
    float    vx, vy;       // commanded velocity
    bool     calibrated;
    Calibration cal;
    float    phi;          // forearm angle now, rad (NAN if not compensated)
    float    Jnow[2][2];   // J at the current pose, after rotation compensation
    uint32_t jogs_sent, cancels;
    float    backlog_mm;   // jog distance FluidNC still has queued
    float    fps;
    char     cal_log[320];  // last calibration: asked vs moved, image shifts
    FloorMap map;
    bool     mark_set;      // fixed camera: image point waiting for "record arm"
    float    mark_u, mark_v;
    float    target_x, target_y;  // fixed camera: target on the floor, mm (NAN if none)
};

namespace controller {
    void        begin();          // starts the control task on core 0
    // Requests from the web UI / button; served by the control task.
    void        requestEnable();
    void        requestDisable(const char* why);
    void        setAutoLights(bool on);
    void        requestCalibrate();
    void        requestClearCal();  // forget the calibration (RAM + NVS)
    // Fixed camera, teaching the floor map (served by the control task, only when disabled):
    void        requestMark();                   // 1a. image point = measured detection
    void        requestMarkAt(float u, float v); // 1b. image point = clicked position
    void        requestRecord();                 // 2. pair it with the current arm XY
    void        requestDeletePoint(int i);
    void        requestSolveMap();               // fit H to the points (>= 4)
    void        requestClearMap();
    // Set J by hand: +X appears at image angle th_deg (atan2(dv, du), v down),
    // moving k image units per mm; +Y at 90 deg from it (see CAM_MIRRORED).
    // Taken as valid at the current arm pose.
    void        requestManualCal(float th_deg, float k);
    CtlStatus   status();
    Params      params();
    bool        setParam(const char* name, float value);  // persists
    float       detectionThreshold();  // min_conf, lowered while calibrating
    const char* modeName(Mode m);
}
