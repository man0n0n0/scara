#pragma once
// Tracker build-time configuration. Runtime-tunable values (gains, deadband...)
// are in Params (controller.h) and can be changed at http://<xiao>/ .

// ---------------------------------------------------------------- arm ----
// The brief gives 1000 / 660 mm. ../dlc32/scara_dlc32_kinematics.yaml says
// arm1_mm 1090 / arm2_mm 1050. Use what FluidNC's kinematics block says.
#define ARM_L1_MM 1090.0f
#define ARM_L2_MM 1050.0f
// Elbow branch: +1 for FluidNC `elbow_positive: true`, -1 for false.
#define ELBOW_SIGN (-1)

// ------------------------------------------------------------- camera ----
// 1: the camera is fixed on a wall (does not move with the arm, nor with Z)
//    and sees the workspace. A floor map taught on the dashboard turns the target's
//    image position into arm X/Y mm, and the tool is driven over the target.
//    Needs JOG_CARTESIAN 1. The Jacobian settings below are then unused.
// 0: the camera rides on the forearm (image Jacobian + rotation compensation).
#define CAMERA_FIXED 1

// 1: FluidNC runs the SCARA kinematics module, so jog X/Y are cartesian mm with
//    origin at the shoulder axis. 0: joints are plain rotary axes, jog X/Y are
//    joint DEGREES (J1, J2) and the limits below are in degrees too.
#define JOG_CARTESIAN 1

// The camera is on the forearm with no wrist, so the image turns with the
// absolute forearm angle (theta1 + theta2). The controller compensates for
// that rotation using the arm pose. Set 0 if the camera does not rotate
// (fixed mount, or a wrist that holds its heading). Only used in cartesian mode.
#define CAMERA_ROTATES_WITH_FOREARM 1

// The measured image Jacobian is squared up after calibration: +X and +Y at
// 90 deg, equal scale, +Y counter-clockwise from +X as seen on the dashboard
// (camera looking down on the work plane). Set 1 if the image is mirrored
// (camera looking up, or via a mirror) so +Y is clockwise from +X.
#define CAM_MIRRORED 0

// --------------------------------------------------------- soft limits ----
// The tracker never sends a jog that would end outside this region. FluidNC's
// SCARA module also rejects unreachable jog targets. StallGuard homing means
// there are no limit switches during tracking, so SET THESE for your exhibit
// layout before enabling tracking. Units: jog units (mm, or deg if !JOG_CARTESIAN).
#define LIM_X_MIN (-900.0f)
#define LIM_X_MAX (900.0f)
#define LIM_Y_MIN (-2100.0f)
#define LIM_Y_MAX (2100.0f)
// Annulus around the shoulder axis (cartesian only): keeps clear of the column
// and of the fully-stretched singularity.
#define LIM_R_MIN (450.0f)
#define LIM_R_MAX (2050.0f)
// The arm found outside the limits (after homing, a hand jog...) is jogged back
// to this far inside them, tracking or not.
#define LIM_MARGIN_MM 20.0f

// Calibration: wait this long after FluidNC reports Idle before using frames.
// Idle = steps done, not arm still: a long arm with a heavy payload keeps
// swinging, and frames taken meanwhile are blurred.
#define CAL_SETTLE_MS 1500

// Forearm camera only (CAMERA_FIXED 0): tracking refuses to run if Z moved
// more than this since calibration (image scale depends on camera height).
#define Z_TOL_MM 5.0f

#if CAMERA_FIXED && !JOG_CARTESIAN
#error "CAMERA_FIXED needs JOG_CARTESIAN 1: the floor map gives X/Y in mm"
#endif

// ---------------------------------------------------------- detection ----
// Edge Impulse class to track (case-sensitive, as named in Studio). Other
// classes in the model are ignored. If several FOMO cells fire on the head,
// their confidence-weighted centroid is the target.
#define TARGET_LABEL "wormbot"

// ------------------------------------------------------------- comms ----
#define FLUIDNC_PORT 23
// Light controller (ESP32-C3, tools/light_c3/main.py): UDP text commands.
// GENERAL IP ON GLi network is 8.255. Broadcast (255.255.255.255) does not
// get through on this network, so reserve this address in the router's DHCP.
#define LIGHTS_HOST "192.168.8.255"
#define LIGHTS_PORT 4210
#define LIGHTS_REPLY_MS 300
// How often to ask FluidNC for a status report ('?', realtime, no line buffer).
#define STATUS_PERIOD_MS 100
// A status older than this pauses tracking until FluidNC answers again.
#define STATUS_STALE_MS 600
// Connected but no status for this long: drop the link and reconnect.
#define FLUIDNC_STALE_RECONNECT_MS 3000
// Enable with FluidNC not Idle: soft reset + $H, then wait this long for Idle.
#define HOME_TIMEOUT_MS 120000

// --------------------------------------------------------------- I/O ----
#define PIN_STOP_BUTTON 0   // BOOT button: press = stop tracking
#define PIN_LED 21          // user LED, active low
