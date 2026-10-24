// Copyright (c) 2026 -	compliance_contentio
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Scara.h"

#include "Machine/MachineConfig.h"
#include "Limit.h"
#include "Machine/Homing.h"
#include "Protocol.h"  // protocol_send_event

#include <cmath>
#include <algorithm>

/*
  ==================== How it works ====================================

  Motor space for X and Y is joint angle in DEGREES:

    motor[X] = theta1, the shoulder angle, measured from the +X axis
    motor[Y] = theta2, the elbow angle, measured RELATIVE to arm1
               (0 = arm fully extended / straight)

  Forward kinematics:
    x = L1*cos(t1) + L2*cos(t1 + t2)
    y = L1*sin(t1) + L2*sin(t1 + t2)

  Inverse kinematics (law of cosines):
    cos(t2) = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2)
    t2      = +/- acos(...)                      <- the "elbow" branch
    t1      = atan2(y, x) - atan2(L2*sin(t2), L1 + L2*cos(t2))

  Because the joint-to-XY mapping is nonlinear, a cartesian move is split into
  short segments and each segment endpoint is converted separately, so the tool
  follows a near-straight line. The feed rate is rescaled per segment from
  cartesian distance to joint distance, otherwise commanded mm/min would mean
  nothing near the workspace edge where a small XY move is a large joint move.

  Steps per degree is set per axis in the config file (steps_per_mm on a rotary
  axis means steps per degree), so J1 and J2 may use different gear ratios.

  COUPLED ELBOW (coupled_elbow: true)

  When the elbow motor is mounted at the base and drives arm2 through a belt
  passing over the shoulder axis, with the same reduction and the same
  direction as J1, the two axes are mechanically coupled:

    motor[X] = theta1          the shoulder angle
    motor[Y] = phi             arm2's ABSOLUTE angle, NOT relative to arm1
    relative elbow angle       theta2 = phi - theta1 = motor[Y] - motor[X]

  Hold motor[Y] still and rotate motor[X]: the belt keeps arm2 pointing the
  same way in world space while arm1 swings under it. That is the coupling.

  The forward transform is then simply

    x = L1*cos(motor[X]) + L2*cos(motor[Y])
    y = L1*sin(motor[X]) + L2*sin(motor[Y])

  j2_min_deg / j2_max_deg always describe the RELATIVE elbow angle, because
  that is what the mechanical stops act on. The Y motor's own span is the sum
  of the two windows, so size max_travel_mm on the Y axis accordingly.

  Configuration:

    kinematics:
      SCARA:
        arm1_mm: 150.0
        arm2_mm: 150.0
        j1_min_deg: -135.0
        j1_max_deg: 135.0
        j2_min_deg: -150.0
        j2_max_deg: 150.0
        elbow_positive: true
        kinematic_segment_len_mm: 1.0
*/

namespace Kinematics {

    void Scara::group(Configuration::HandlerBase& handler) {
        // @config arm1_mm
        // @default 150.0
        // @tuning per-machine
        // Length of the first arm link, shoulder pivot to elbow pivot.
        handler.item("arm1_mm", _arm1_mm, 1.0, 100000.0);

        // @config arm2_mm
        // @default 150.0
        // @tuning per-machine
        // Length of the second arm link, elbow pivot to tool centre.
        handler.item("arm2_mm", _arm2_mm, 1.0, 100000.0);

        // @config j1_min_deg
        // @default -135.0
        // @tuning per-machine
        // Most negative shoulder angle the machine can reach without collision.
        handler.item("j1_min_deg", _j1_min_deg, -360.0, 360.0);

        // @config j1_max_deg
        // @default 135.0
        // @tuning per-machine
        // Most positive shoulder angle the machine can reach without collision.
        handler.item("j1_max_deg", _j1_max_deg, -360.0, 360.0);

        // @config j2_min_deg
        // @default -150.0
        // @tuning per-machine
        // Most negative elbow angle, relative to arm1 (0 = straight).
        handler.item("j2_min_deg", _j2_min_deg, -360.0, 360.0);

        // @config j2_max_deg
        // @default 150.0
        // @tuning per-machine
        // Most positive elbow angle, relative to arm1 (0 = straight).
        handler.item("j2_max_deg", _j2_max_deg, -360.0, 360.0);

        // @config elbow_positive
        // @default true
        // @tuning per-machine
        // Default elbow branch, used only until the arm's pose is known. After
        // homing the branch is read from the motors themselves and this value
        // is no longer consulted, so it cannot contradict the real arm.
        handler.item("elbow_positive", _elbow_positive);

        // @config coupled_elbow
        // @default false
        // @tuning per-machine
        // true when the elbow motor is mounted at the base and drives arm2 by a
        // belt over the shoulder axis, same reduction and same direction as J1.
        // false for a motor carried on arm1 driving the elbow directly.
        handler.item("coupled_elbow", _coupled_elbow);

        // @config j2_home_relative
        // @default false
        // @tuning per-machine
        // true when the J2 sensor sits on arm1 and so detects a fixed angle
        // RELATIVE to arm1 (the usual mounting, since no cable then crosses a
        // rotating joint). The Y axis' mpos_mm is then read as that relative
        // elbow angle instead of an absolute one, which also allows J2 to be
        // homed BEFORE J1. Only meaningful with coupled_elbow.
        handler.item("j2_home_relative", _j2_home_relative);

        // @config kinematic_segment_len_mm
        // @default 1.0
        // @tuning typical
        // Maximum length of the short segments a cartesian move is split into
        // before conversion to joint angles. Smaller is straighter but produces
        // more planner blocks; below roughly 0.5 mm the planner becomes the
        // bottleneck and motion gets choppy.
        handler.item("kinematic_segment_len_mm", _kinematic_segment_len_mm, 0.05, 20.0);

        // @config r_min_mm
        // @default 0.0
        // @tuning per-machine
        // Smallest tool radius from the shoulder axis that is allowed. Keeps
        // the arm out of the folded-up centre, where the kinematics is
        // singular: there the tool position no longer determines the joint
        // angles, so a millimetre of tool motion can demand a half turn of the
        // shoulder. 0 disables the check.
        handler.item("r_min_mm", _r_min_mm, 0.0, 100000.0);

        // @config r_max_mm
        // @default 0.0
        // @tuning per-machine
        // Largest tool radius allowed. 0 means the arm's own reach, arm1+arm2.
        // Setting it a little short keeps the arm away from the fully extended
        // singularity, where accuracy collapses in the radial direction.
        handler.item("r_max_mm", _r_max_mm, 0.0, 100000.0);

        // @config x_min_mm
        // @default -100000.0
        // @tuning per-machine
        // Straight-line blockage: refuse any point with x below this. For a
        // column, a wall or the edge of a bench the arm must not cross.
        handler.item("x_min_mm", _x_min_mm, -100000.0, 100000.0);

        // @config max_segment_deg
        // @default 2.0
        // @tuning typical
        // Largest joint rotation allowed in a single segment. Guards the case
        // the cartesian segment length cannot see: close to a singularity a
        // tiny tool move needs a huge joint move, and one segment of that is
        // interpolated linearly in joint space, so the tool leaves the
        // commanded path entirely. Lower is more faithful and more blocks.
        handler.item("max_segment_deg", _max_segment_deg, 0.1, 45.0);
    }

    void Scara::init() {
        log_info("Kinematic system: " << name());

        _max_reach_mm = _arm1_mm + _arm2_mm;
        _min_reach_mm = fabsf(_arm1_mm - _arm2_mm);

        _r_max_effective_mm = (_r_max_mm > 0.0f) ? std::min(_r_max_mm, _max_reach_mm) : _max_reach_mm;

        log_info("  SCARA arm1:" << _arm1_mm << "mm arm2:" << _arm2_mm << "mm reach:" << _min_reach_mm << ".." << _max_reach_mm << "mm");
        if (_j2_home_relative && !_coupled_elbow) {
            log_warn("SCARA j2_home_relative has no effect without coupled_elbow");
        }
        log_info("  J2 home sensor:" << (_coupled_elbow && _j2_home_relative ? "relative to arm1 (J2 may home first)"
                                                                             : "absolute (J2 must home after J1)"));
        log_info("  Elbow drive:" << (_coupled_elbow ? "coupled (Y motor = absolute arm2 angle)"
                                                     : "independent (Y motor = relative elbow angle)"));
        log_info("  J1 range:" << _j1_min_deg << ".." << _j1_max_deg << "deg  J2 range:" << _j2_min_deg << ".." << _j2_max_deg << "deg");

        if (_j1_min_deg >= _j1_max_deg || _j2_min_deg >= _j2_max_deg) {
            log_error("SCARA joint limits are inverted; min must be less than max");
        }
        log_info("  Work envelope: r " << _r_min_mm << ".." << _r_max_effective_mm << "mm");
        if (_x_min_mm > -99999.0f) {
            log_info("  Straight-line blockage: x >= " << _x_min_mm << "mm");
        }
        if (_r_min_mm <= 0.0f) {
            log_warn("SCARA r_min_mm is 0, so the singular folded-up centre is not guarded");
        }
        if ((_j1_max_deg - _j1_min_deg) >= 360.0f || (_j2_max_deg - _j2_min_deg) >= 360.0f) {
            log_warn("SCARA joint window spans a full turn, so it protects nothing. Set the real mechanical stops.");
        }

        init_position();
    }

    void Scara::init_position() {
        auto n_axis = Axes::_numberAxis;

        for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
            set_steps(axis, 0);
        }

        // Motor-space limits. For the two joints these are the joint angle
        // limits; everything else is linear so its cartesian limits apply
        // directly. Only servo-type motors consult these, but they must still
        // be sane rather than the result of running a cartesian extreme through
        // the IK, which is what the Cartesian base class would have done.
        _min_motor_pos[X_AXIS] = _j1_min_deg;
        _max_motor_pos[X_AXIS] = _j1_max_deg;
        if (_coupled_elbow) {
            // The Y motor carries an absolute angle, so its span is the sum of
            // both joint windows.
            _min_motor_pos[Y_AXIS] = _j1_min_deg + _j2_min_deg;
            _max_motor_pos[Y_AXIS] = _j1_max_deg + _j2_max_deg;
        } else {
            _min_motor_pos[Y_AXIS] = _j2_min_deg;
            _max_motor_pos[Y_AXIS] = _j2_max_deg;
        }
        for (axis_t axis = Z_AXIS; axis < n_axis; axis++) {
            _min_motor_pos[axis] = limitsMinPosition(axis);
            _max_motor_pos[axis] = limitsMaxPosition(axis);
        }

        setArray(_last_motor_pos, 0.0f, n_axis);

        // No pose is known yet (the elbow reads 0, straight), so fall back to
        // the configured branch until homing establishes the real one.
        _elbow_positive_current = _elbow_positive;
    }

    void Scara::sync_from_motors() {
        auto n_axis = Axes::_numberAxis;
        copyAxes(_last_motor_pos, get_motor_pos(), n_axis);

        // Within half a degree of straight the branch is undefined; keep what
        // we had rather than guess from rounding noise.
        const float elbow = relative_elbow_deg(_last_motor_pos);
        if (fabsf(elbow) > 0.5f) {
            _elbow_positive_current = elbow > 0.0f;
        }
    }

    float Scara::wrap_degrees(float degrees, float lo, float hi) {
        // A window that spans a full turn or more accepts any angle, and has no
        // unique representative to pick. Keep the value the IK produced: its
        // sign carries the elbow branch, and shifting it by a turn would both
        // invert that sign and command the motor a full revolution away.
        if ((hi - lo) >= 360.0f) {
            return degrees;
        }
        while (degrees < lo) {
            degrees += 360.0f;
        }
        while (degrees > hi) {
            degrees -= 360.0f;
        }
        return degrees;
    }

    bool Scara::envelope_error(const float* cartesian, float r_floor, float r_ceiling, float x_floor) const {
        const float x = cartesian[X_AXIS];
        const float y = cartesian[Y_AXIS];
        const float r = sqrtf(x * x + y * y);

        if (r < r_floor) {
            log_warn("SCARA r " << r << "mm is inside r_min " << r_floor << "mm, near the folded singularity");
            return true;
        }
        if (r > r_ceiling) {
            log_warn("SCARA r " << r << "mm is beyond r_max " << r_ceiling << "mm");
            return true;
        }
        if (x < x_floor) {
            log_warn("SCARA x " << x << "mm is beyond x_min " << x_floor << "mm");
            return true;
        }
        return false;
    }

    void Scara::envelope_floors(const float* from, float& r_floor, float& r_ceiling, float& x_floor, bool& escaping) const {
        const float r_start = sqrtf(from[X_AXIS] * from[X_AXIS] + from[Y_AXIS] * from[Y_AXIS]);

        escaping = r_start < _r_min_mm;

        // A guard must block entry into a bad region without trapping the arm
        // inside one, so each floor is relaxed to admit where the arm already
        // is. r_min is the exception: merely "not going further in" is not
        // enough there, because every point near the folded centre needs an
        // arbitrary shoulder angle. So when the arm starts inside r_min the
        // target must land OUTSIDE it -- a real escape, not a nudge.
        r_floor   = escaping ? _r_min_mm : std::min(_r_min_mm, r_start);
        r_ceiling = std::max(_r_max_effective_mm, r_start);
        x_floor   = std::min(_x_min_mm, from[X_AXIS]);
    }

    bool Scara::transform_cartesian_to_motors(float* motors, float* cartesian) {
        auto n_axis = Axes::_numberAxis;

        // Z and anything above it are untransformed
        for (axis_t axis = Z_AXIS; axis < n_axis; axis++) {
            motors[axis] = cartesian[axis];
        }

        const float x  = cartesian[X_AXIS];
        const float y  = cartesian[Y_AXIS];
        const float r2 = x * x + y * y;

        float cos_t2 = (r2 - _arm1_mm * _arm1_mm - _arm2_mm * _arm2_mm) / (2.0f * _arm1_mm * _arm2_mm);

        // Outside the annulus the arm simply cannot reach the point. Allow a
        // hair of slop so a target sitting exactly on the boundary, where the
        // rounding can push cos_t2 a few ulp past 1, is not rejected.
        if (cos_t2 > 1.0f) {
            if (cos_t2 > 1.0f + 1e-4f) {
                log_debug("SCARA target out of reach, too far: " << sqrtf(r2) << "mm");
                return false;
            }
            cos_t2 = 1.0f;
        } else if (cos_t2 < -1.0f) {
            if (cos_t2 < -1.0f - 1e-4f) {
                log_debug("SCARA target out of reach, inside dead zone: " << sqrtf(r2) << "mm");
                return false;
            }
            cos_t2 = -1.0f;
        }

        float t2 = acosf(cos_t2);
        // Solve in the branch the arm is actually in. Using a fixed config flag
        // here is what let a 1 mm move demand the mirror-image arm: +-150
        // degrees on both joints at full speed.
        if (!_elbow_positive_current) {
            t2 = -t2;
        }

        // atan2(0, 0) is 0 by convention, which is the degenerate case of equal
        // arms commanded to the centre: the shoulder angle is then arbitrary and
        // 0 is as good an answer as any.
        const float t1 = atan2f(y, x) - atan2f(_arm2_mm * sinf(t2), _arm1_mm + _arm2_mm * cosf(t2));

        const float j1 = wrap_degrees(to_degrees(t1), _j1_min_deg, _j1_max_deg);
        const float j2 = wrap_degrees(to_degrees(t2), _j2_min_deg, _j2_max_deg);

        if (j1 < _j1_min_deg || j1 > _j1_max_deg) {
            log_debug("SCARA J1 " << j1 << "deg outside " << _j1_min_deg << ".." << _j1_max_deg);
            return false;
        }
        if (j2 < _j2_min_deg || j2 > _j2_max_deg) {
            log_debug("SCARA J2 " << j2 << "deg outside " << _j2_min_deg << ".." << _j2_max_deg);
            return false;
        }

        motors[X_AXIS] = j1;

        // With a coupled elbow the Y motor commands arm2's ABSOLUTE angle, so
        // the shoulder angle has to be added back in. The sum is deliberately
        // not wrapped: both terms are already inside their own windows, and
        // leaving it unwrapped keeps the motor value continuous across the
        // +/-180 boundary, which matters because the planner interpolates it.
        motors[Y_AXIS] = _coupled_elbow ? (j1 + j2) : j2;

        return true;
    }

    void Scara::motors_to_cartesian(float* cartesian, float* motors, axis_t n_axis) {
        const float t1 = to_radians(motors[X_AXIS]);

        // phi is arm2's absolute angle. When the elbow is coupled the Y motor
        // already holds it; otherwise it is relative and adds to the shoulder.
        const float phi = _coupled_elbow ? to_radians(motors[Y_AXIS]) : t1 + to_radians(motors[Y_AXIS]);

        cartesian[X_AXIS] = _arm1_mm * cosf(t1) + _arm2_mm * cosf(phi);
        cartesian[Y_AXIS] = _arm1_mm * sinf(t1) + _arm2_mm * sinf(phi);

        for (axis_t axis = Z_AXIS; axis < n_axis; axis++) {
            cartesian[axis] = motors[axis];
        }
    }

    bool Scara::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        auto n_axis = Axes::_numberAxis;

        // _last_motor_pos must describe where this move STARTS. It is only
        // written by our own segments and by homing, so a reset, an alarm or a
        // cancelled jog -- each of which resyncs the planner to the real step
        // counters -- leaves it pointing at a place the arm never reached. The
        // first segment would then measure a bogus joint distance, scale its
        // feed rate by it, and run at max_rate. Forward kinematics is unique,
        // so comparing FK(_last_motor_pos) with the planner's start is a safe
        // staleness test even near a singularity.
        {
            float fk_last[MAX_N_AXIS];
            motors_to_cartesian(fk_last, _last_motor_pos, n_axis);
            if (vector_distance(fk_last, position, 2) > 0.01f) {
                log_debug("SCARA motor bookkeeping was stale, resyncing from the step counters");
                sync_from_motors();
            }
        }

        float r_floor, r_ceil, x_floor;
        bool  escaping;
        envelope_floors(position, r_floor, r_ceil, x_floor, escaping);

        if (escaping) {
            const float r_target = sqrtf(target[X_AXIS] * target[X_AXIS] + target[Y_AXIS] * target[Y_AXIS]);
            if (r_target < _r_min_mm) {
                log_error("SCARA is inside the singular centre. A small move from here needs an arbitrary, "
                          "possibly huge joint rotation. Command a point at r >= "
                          << _r_min_mm << "mm to leave it.");
                return false;
            }
            log_warn("SCARA leaving the singular centre: expect a large joint motion");
        }

        float motors[MAX_N_AXIS];
        if (!transform_cartesian_to_motors(motors, target)) {
            log_warn("SCARA target unreachable (" << target[X_AXIS] << "," << target[Y_AXIS] << ")");
            return false;
        }
        if (envelope_error(target, r_floor, r_ceil, x_floor)) {
            return false;
        }

        float d[MAX_N_AXIS];
        copyAxes(d, target, n_axis);
        subtractAxes(d, position, n_axis);

        // Only XY bend, so only XY decides how finely the cartesian path must
        // be chopped. A pure Z or pure rotary move stays a single segment.
        uint32_t segment_count = ceil(vector_length(d, 2) / _kinematic_segment_len_mm);
        if (segment_count == 0) {
            segment_count = 1;
        }

        // Now bound the JOINT travel per segment as well. The two criteria
        // answer different questions: the cartesian one keeps a long move from
        // bowing between segments, this one keeps a SHORT move near a
        // singularity from being flung across joint space in one hop.
        // Measure the joint travel from where the motors ACTUALLY are, not from
        // the IK of the cartesian start point. The two differ exactly where it
        // matters: at a singularity the cartesian position does not determine
        // the joint angles at all -- with the elbow at 180 degrees every
        // shoulder angle puts the tool on the shoulder axis -- so IK-ing the
        // start point returns some arbitrary member of that family and would
        // under-count the real motion by any amount.
        float joint_travel = 0.0f;
        for (axis_t axis = X_AXIS; axis <= Y_AXIS; axis++) {
            joint_travel = std::max(joint_travel, fabsf(motors[axis] - _last_motor_pos[axis]));
        }
        uint32_t joint_segments = ceil(joint_travel / _max_segment_deg);
        if (joint_segments > segment_count) {
            log_debug("SCARA " << joint_travel << "deg of joint travel over " << vector_length(d, 2)
                               << "mm: raising segments from " << segment_count << " to " << joint_segments);
            segment_count = joint_segments;
        }
        if (joint_travel > 90.0f && vector_length(d, 2) < 5.0f) {
            log_warn("SCARA near a singularity: " << vector_length(d, 2) << "mm of tool motion needs "
                                                  << joint_travel << "deg of joint motion");
        }

        const float segment_dist = vector_length(d, n_axis) / segment_count;

        float seg_target[MAX_N_AXIS];
        copyAxes(seg_target, position, n_axis);

        float delta_d[MAX_N_AXIS];
        copyArray(delta_d, d, n_axis);
        multiplyArray(delta_d, 1.0f / segment_count, n_axis);

        const float feed_rate = pl_data->feed_rate;  // cartesian units, save the original

        for (uint32_t segment = 1; segment <= segment_count; segment++) {
            if (sys.abort()) {
                return true;
            }

            addAxes(seg_target, delta_d, n_axis);

            if (!transform_cartesian_to_motors(motors, seg_target)) {
                log_error("SCARA unreachable mid-move at (" << seg_target[X_AXIS] << "," << seg_target[Y_AXIS] << ")");
                return false;
            }
            // While escaping, the path necessarily crosses the blocked centre,
            // so r_min is not applied to the intermediate points -- only to the
            // destination, which was checked above.
            if (envelope_error(seg_target, escaping ? 0.0f : r_floor, r_ceil, x_floor)) {
                log_error("SCARA work envelope breached mid-move");
                return false;
            }

            // Convert the cartesian feed rate into a joint feed rate for this
            // segment, in the same proportion as joint distance to cartesian
            // distance. Rapids are left alone; the planner rates those itself.
            if (!pl_data->motion.rapidMotion && segment_dist > 0.0f) {
                const float joint_distance = vector_distance(motors, _last_motor_pos, n_axis);
                pl_data->feed_rate         = feed_rate * joint_distance / segment_dist;
            }

            // Returns false when a jog is cancelled: stop feeding the planner.
            if (!mc_move_motors(motors, pl_data)) {
                return false;
            }

            // After mc_move_motors(), so a discarded segment does not corrupt
            // the reference used for the next distance calculation.
            copyAxes(_last_motor_pos, motors, n_axis);
        }

        return true;
    }

    bool Scara::invalid_line(float* cartesian, plan_line_data_t* pl_data) {
        // The per-axis soft limits cannot be used here: for X and Y they are
        // expressed in joint degrees. Reachability plus the joint limits is the
        // real test, and transform_cartesian_to_motors applies both.
        float motors[MAX_N_AXIS] = { 0.0 };

        if (!transform_cartesian_to_motors(motors, cartesian)) {
            limit_error(cartesian, pl_data);
            return true;
        }

        // Same guards as cartesian_to_motors, against the current machine
        // position.
        float r_floor, r_ceil, x_floor;
        bool  escaping;
        envelope_floors(get_mpos(), r_floor, r_ceil, x_floor, escaping);
        if (envelope_error(cartesian, r_floor, r_ceil, x_floor)) {
            limit_error(cartesian, pl_data);
            return true;
        }

        return false;
    }

    bool Scara::invalid_arc(float*            target,
                            plan_line_data_t* pl_data,
                            float*            position,
                            float             center[3],
                            float             radius,
                            axis_t            caxes[3],
                            bool              is_clockwise_arc,
                            uint32_t          rotations) {
        // Only the arc endpoint is pre-checked, by invalid_line(). A point part
        // way round the arc that leaves the workspace is caught mid-move by
        // cartesian_to_motors(), which aborts the move.
        return false;
    }

    void Scara::constrain_jog(float* target, plan_line_data_t* pl_data, float* position) {
        float motors[MAX_N_AXIS] = { 0.0 };

        float r_floor, r_ceil, x_floor;
        bool  escaping;
        envelope_floors(position, r_floor, r_ceil, x_floor, escaping);

        if (transform_cartesian_to_motors(motors, target) && !envelope_error(target, r_floor, r_ceil, x_floor)) {
            return;  // reachable and inside the envelope, jog as commanded
        }

        // Rather than clipping to the nearest reachable point, which on a SCARA
        // means solving for the intersection with a nonlinear boundary, refuse
        // the jog and hold position.
        log_warn("SCARA jog target unreachable, jog rejected");
        copyAxes(target, position);
    }

    void Scara::homing_move(AxisMask axisMask, MotorMask motors, Machine::Homing::Phase phase, uint32_t& settling_ms) {
        _homing_axes = axisMask;

        releaseMotors(axisMask, motors);

        float rate = 0.0f;
        float axis_target[MAX_N_AXIS];

        // axesVector() plans the phase in AXIS units, starting from the current
        // cartesian mpos. For the joints those axis units are already degrees,
        // so the useful part is the displacement it computed, not the absolute
        // target. Subtracting the starting point recovers that displacement
        // together with all of the seek/feed rate and scaler handling.
        axesVector(axisMask, motors, phase, axis_target, rate, settling_ms);

        if (rate == 0.0f) {
            return;  // nothing to do this phase, or axesVector reported an error
        }

        auto n_axis = Axes::_numberAxis;

        float displacement[MAX_N_AXIS];
        copyAxes(displacement, axis_target, n_axis);
        subtractAxes(displacement, get_mpos(), n_axis);

        float motor_target[MAX_N_AXIS];
        copyAxes(motor_target, get_motor_pos(), n_axis);
        addAxes(motor_target, displacement, n_axis);

        // Coupled elbow: the Y motor commands arm2's ABSOLUTE angle, so if the
        // shoulder sweeps while Y stands still, the relative elbow angle
        // changes by the whole sweep and the elbow runs into its stop. Drive Y
        // by the same amount, in the same direction, and the relative angle
        // (motor Y - motor X) is preserved exactly: the arm homes rigid,
        // holding whatever elbow angle it started at.
        if (_coupled_elbow && bitnum_is_true(axisMask, X_AXIS)) {
            if (bitnum_is_true(axisMask, Y_AXIS)) {
                // Y would need to both track X and seek its own switch.
                // Returning quietly would stall the homing state machine, so
                // fail it outright.
                log_error("SCARA coupled elbow: J1 and J2 must home in separate cycles");
                Machine::Homing::fail(ExecAlarm::HomingFailApproach);
                return;
            }
            motor_target[Y_AXIS] += displacement[X_AXIS];

            // Two motors now travel the same distance instead of one, so the
            // move vector is sqrt(2) longer. Scale the rate to keep the
            // configured seek/feed speed of the shoulder itself.
            rate *= 1.41421356f;
        }

        plan_line_data_t plan_data      = {};
        plan_data.spindle_speed         = 0;
        plan_data.motion                = {};
        plan_data.motion.systemMotion   = 1;
        plan_data.motion.noFeedOverride = 1;
        plan_data.spindle               = SpindleState::Disable;
        plan_data.coolant               = {};
        plan_data.line_number           = 0;
        plan_data.is_jog                = false;
        plan_data.feed_rate             = rate;

        mc_move_motors(motor_target, &plan_data);

        copyAxes(_last_motor_pos, motor_target, n_axis);

        protocol_send_event(&cycleStartEvent);
    }

    void Scara::set_homed_mpos(float* mpos) {
        // Homing::set_mpos() hands us the live cartesian mpos array with ONLY
        // the just-homed axes overwritten by their configured mpos_mm. For a
        // homed joint that entry is the angle at the switch and is written to
        // the motor as-is -- no transform, since running a pair of angles
        // through the IK would treat them as XY coordinates.
        //
        // Every other entry is still a cartesian coordinate and must NOT be
        // written to a motor. That matters here beyond the usual reason: with
        // a coupled elbow the Y motor physically moves while J1 homes, and its
        // step count is already correct.
        auto n_axis = Axes::_numberAxis;

        // get_motor_pos() hands back a shared buffer, so snapshot it before
        // any set_steps() call invalidates what we are still reading.
        float current[MAX_N_AXIS];
        copyAxes(current, get_motor_pos(), n_axis);

        const bool relative_elbow = _coupled_elbow && _j2_home_relative;

        if (bitnum_is_true(_homing_axes, Y_AXIS)) {
            float m2 = mpos[Y_AXIS];
            if (relative_elbow) {
                // The sensor measured a RELATIVE angle, so turn it into the
                // absolute angle the Y motor carries by adding wherever the
                // shoulder happens to be. If J1 has not homed yet that
                // reference is arbitrary and m2 inherits its error -- which
                // the shift below cancels exactly when J1 does home. This is
                // what lets J2 home FIRST, to swing the elbow clear before the
                // shoulder sweeps.
                m2 += current[X_AXIS];
            }
            set_steps(Y_AXIS, motor_pos_to_steps(m2, Y_AXIS));
        }

        if (bitnum_is_true(_homing_axes, X_AXIS)) {
            // How far our idea of the shoulder angle was off.
            const float correction = mpos[X_AXIS] - current[X_AXIS];

            set_steps(X_AXIS, motor_pos_to_steps(mpos[X_AXIS], X_AXIS));

            if (relative_elbow) {
                // J2's absolute angle was derived from an unhomed shoulder, so
                // it carries the same error. Shifting it by the same
                // correction leaves the relative elbow angle -- the quantity
                // the sensor actually measured, and the one the homing
                // compensation held constant throughout this sweep -- exactly
                // intact, while both angles become absolutely correct.
                //
                // Harmless when J2 has not been homed yet: its value is
                // arbitrary either way, and its own homing will overwrite it.
                set_steps(Y_AXIS, motor_pos_to_steps(current[Y_AXIS] + correction, Y_AXIS));
            }
        }

        // Z and anything above it are linear and need no such treatment.
        for (axis_t axis = Z_AXIS; axis < n_axis; axis++) {
            if (bitnum_is_true(_homing_axes, axis)) {
                set_steps(axis, motor_pos_to_steps(mpos[axis], axis));
            }
        }

        sync_from_motors();

        if (bitnum_is_true(_homing_axes, X_AXIS) || bitnum_is_true(_homing_axes, Y_AXIS)) {
            const float elbow = relative_elbow_deg(_last_motor_pos);
            log_info("SCARA homed elbow " << elbow << "deg, branch " << (_elbow_positive_current ? "positive" : "negative"));
            if (fabsf(elbow) > 0.5f && _elbow_positive_current != _elbow_positive) {
                log_warn("SCARA elbow_positive in the config disagrees with the homed pose; following the arm. "
                         "Set elbow_positive: " << (_elbow_positive_current ? "true" : "false") << " to silence this.");
            }
        }
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<Scara> registration("SCARA");
    }
}
