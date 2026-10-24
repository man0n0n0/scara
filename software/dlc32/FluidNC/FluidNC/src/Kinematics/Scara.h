// Copyright (c) 2026 -	compliance_contentio
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
	Scara.h

	Kinematics for a 2-joint (RR) SCARA arm with an independent linear Z.

	  J1 (shoulder)  -> X axis motor, units are DEGREES
	  J2 (elbow)     -> Y axis motor, units are DEGREES, measured RELATIVE to arm1
	  Z              -> passes straight through, units are mm

	G-code stays in cartesian mm. Moves are broken into short segments and each
	segment endpoint is converted to joint angles, the same technique the delta
	kinematics uses.
*/

#include "Kinematics.h"
#include "Cartesian.h"

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

namespace Kinematics {

    class Scara : public Cartesian {
    public:
        Scara(const char* name) : Cartesian(name) {}

        Scara(const Scara&)            = delete;
        Scara(Scara&&)                 = delete;
        Scara& operator=(const Scara&) = delete;
        Scara& operator=(Scara&&)      = delete;

        // Kinematic interface
        void init() override;
        void init_position() override;

        bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) override;
        void motors_to_cartesian(float* cartesian, float* motors, axis_t n_axis) override;
        bool transform_cartesian_to_motors(float* motors, float* cartesian) override;

        void constrain_jog(float* cartesian, plan_line_data_t* pl_data, float* position) override;
        bool invalid_line(float* cartesian, plan_line_data_t* pl_data) override;
        bool invalid_arc(float*            target,
                         plan_line_data_t* pl_data,
                         float*            position,
                         float             center[3],
                         float             radius,
                         axis_t            caxes[3],
                         bool              is_clockwise_arc,
                         uint32_t          rotations) override;

        // Homing happens in joint space: the limit switches sit on the joints,
        // so a homing move cannot be planned in cartesian coordinates.
        void homing_move(AxisMask axes, MotorMask motors, Machine::Homing::Phase phase, uint32_t& settling_ms) override;
        void set_homed_mpos(float* mpos) override;

        // canHome(), limitReached(), releaseMotors() and rearmLimits() are
        // inherited from Cartesian: every joint has its own switch on its own
        // motor, exactly like a cartesian axis.

        // Configuration handlers
        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override {}
        void validate() override {}

        ~Scara() {}

    private:
        // Geometry
        float _arm1_mm = 150.0;  // shoulder pivot -> elbow pivot
        float _arm2_mm = 150.0;  // elbow pivot -> tool centre

        // Joint travel limits, in degrees. These are the real workspace guard:
        // the per-axis soft_limits in the config file cannot protect a SCARA
        // because max_travel_mm there describes joint angles, not XY.
        float _j1_min_deg = -135.0;
        float _j1_max_deg = 135.0;
        float _j2_min_deg = -150.0;
        float _j2_max_deg = 150.0;

        // Which IK branch to use. Both branches reach the same XY; they differ
        // in which way the elbow folds. Flipping mid-job is not possible, so
        // the arm stays in the configured branch for the whole job.
        bool _elbow_positive = true;

        // True when the elbow motor sits at the base and drives arm2 through a
        // belt passing over the shoulder axis, with the SAME reduction and the
        // SAME direction as J1. The Y motor then commands arm2's ABSOLUTE
        // angle, not its angle relative to arm1. See the note in Scara.cpp.
        bool _coupled_elbow = false;

        // True when the J2 sensor is mounted on arm1 and therefore trips at a
        // fixed angle RELATIVE to arm1, rather than at a fixed absolute angle.
        // The Y axis' mpos_mm is then that relative angle, and J2 may be homed
        // before J1. See set_homed_mpos().
        bool _j2_home_relative = false;

        float _kinematic_segment_len_mm = 1.0;

        // Upper bound on how far any joint may turn within one segment. The
        // cartesian segment length alone is not enough: near a singularity a
        // millimetre of tool motion can demand hundreds of degrees of joint
        // motion, and a single segment would then be interpolated straight in
        // JOINT space, sending the tool far off the intended path.
        float _max_segment_deg = 2.0;

        // Work-envelope guards, after jared-hughes/grbl-scara limits.c:359.
        // Reachability alone is not enough: both edges of the annulus are
        // singular, and the arm is least useful exactly there.
        float _r_min_mm = 0.0;        // 0 disables; blocks the folded-up centre
        float _r_max_mm = 0.0;        // 0 means "the arm's own reach"
        float _x_min_mm = -100000.0;  // straight-line blockage, e.g. a column

        // Derived in init()
        float _max_reach_mm = 0.0;
        float _min_reach_mm = 0.0;
        float _r_max_effective_mm = 0.0;

        float _last_motor_pos[MAX_N_AXIS] = { 0 };

        // The elbow branch the arm is ACTUALLY in, read from the motors after
        // homing and on every resync -- the RepRapFirmware approach
        // (ScaraKinematics.cpp: currentArmMode = motorPos >= 0). The IK always
        // solves in this branch and a move can never flip it. _elbow_positive
        // from the config is only the default before any pose is known.
        bool _elbow_positive_current = true;

        // Axes of the homing cycle currently running. Homing::_cycleAxes is
        // private, but homing_move() is handed the same mask every phase, so
        // we latch it here for set_homed_mpos() to use afterwards.
        AxisMask _homing_axes = 0;

        // Relative elbow angle, in degrees, for a set of motor positions.
        float relative_elbow_deg(const float* motors) const { return _coupled_elbow ? motors[Y_AXIS] - motors[X_AXIS] : motors[Y_AXIS]; }

        // Reload _last_motor_pos from the real step counters and re-derive the
        // elbow branch from them.
        void sync_from_motors();

        static inline float to_degrees(float radians) { return radians * float(180.0 / M_PI); }
        static inline float to_radians(float degrees) { return degrees * float(M_PI / 180.0); }

        // Bring an angle into [lo, hi] by whole turns, so a joint whose travel
        // does not straddle 0 (say 20..300 degrees) still gets an in-range
        // solution. A window 360 degrees wide or more admits every angle and
        // has no unique representative, so the natural value is returned
        // untouched -- wrapping there would silently move the angle a whole
        // turn away and flip its sign.
        static float wrap_degrees(float degrees, float lo, float hi);

        // True when the point breaks a work-envelope guard. The floors are
        // passed in rather than read from config so a move that STARTS in a
        // blocked region can still escape it: see cartesian_to_motors().
        bool envelope_error(const float* cartesian, float r_floor, float r_ceiling, float x_floor) const;

        // Resolve the guards for a move starting at `from`. Sets escaping when
        // that start is already inside r_min, which changes the rule: the only
        // legal move from there is one that LEAVES the region.
        void envelope_floors(const float* from, float& r_floor, float& r_ceiling, float& x_floor, bool& escaping) const;
    };
}  //  namespace Kinematics
