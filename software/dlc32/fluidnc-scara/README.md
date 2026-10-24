# SCARA kinematics for FluidNC

A `KinematicSystem` implementation for a 2-joint (RR) SCARA arm with an
independent linear Z. With it, G-code stays in cartesian mm and the firmware
solves the joint angles.

## Files

| File | Purpose |
| --- | --- |
| `Scara.h` / `Scara.cpp` | The kinematics module. Drop into `FluidNC/src/Kinematics/`. |
| `test_scara.cpp` | Host-side test of the IK/FK math. Not part of the firmware. |
| `test_vs_grbl.cpp` | Compares the IK against jared-hughes/grbl-scara. |
| `test_envelope.cpp` | Work-envelope guards, including the escape rule. |
| `../scara_dlc32_kinematics.yaml` | Machine config that uses this module. |
| `../scara_dlc32.yaml` | Stock-firmware fallback: joints as rotary axes, IK done by the post-processor. |

## Building

```sh
git clone https://github.com/bdring/FluidNC.git
cp Scara.h Scara.cpp FluidNC/FluidNC/src/Kinematics/
cd FluidNC && pio run -e wifi          # PlatformIO globs src/, no build file to edit
```

Then select it in the machine config:

```yaml
kinematics:
  SCARA:
    arm1_mm: 180.0
    arm2_mm: 120.0
```

## How it works

Motor space for X and Y is joint angle in **degrees**:

- `motor[X]` = θ1, shoulder angle measured from the +X axis
- `motor[Y]` = θ2, elbow angle measured **relative to arm1** (0 = arm straight)
- Z and any axis above it pass through untransformed

Forward:

```
x = L1·cos(θ1) + L2·cos(θ1 + θ2)
y = L1·sin(θ1) + L2·sin(θ1 + θ2)
```

Inverse, by the law of cosines:

```
cos(θ2) = (x² + y² − L1² − L2²) / (2·L1·L2)
θ2      = ±acos(…)                            ← the elbow branch
θ1      = atan2(y, x) − atan2(L2·sin θ2, L1 + L2·cos θ2)
```

Because the mapping is nonlinear, `cartesian_to_motors()` splits each move into
segments of at most `kinematic_segment_len_mm` and converts each endpoint, so
the tool follows a near-straight line. The feed rate is rescaled per segment
from cartesian distance to joint distance — without that, a commanded mm/min
would mean nothing near the workspace edge, where a small XY move is a large
joint move.

Homing runs in joint space (`homing_move()` / `set_homed_mpos()`), because the
switches are on the joints and a homing move has no meaningful cartesian
target. The displacement computed by `Cartesian::axesVector()` is reused, then
applied to motor positions instead of cartesian ones — that keeps all of the
stock seek/feed rate and scaler handling.

## Config items

| Item | Default | Meaning |
| --- | --- | --- |
| `arm1_mm` | 150.0 | Shoulder pivot → elbow pivot |
| `arm2_mm` | 150.0 | Elbow pivot → tool centre |
| `j1_min_deg` / `j1_max_deg` | −135 / 135 | Shoulder travel |
| `j2_min_deg` / `j2_max_deg` | −150 / 150 | Elbow travel, relative to arm1 |
| `elbow_positive` | true | Which IK branch (which way the elbow folds) |
| `coupled_elbow` | false | Elbow motor at the base, belt over the shoulder axis |
| `j2_home_relative` | false | J2 sensor on arm1, so it reads a relative angle |
| `kinematic_segment_len_mm` | 1.0 | Max cartesian segment length |
| `max_segment_deg` | 2.0 | Max joint rotation per segment |
| `r_min_mm` | 0.0 | Smallest tool radius; guards the folded singularity |
| `r_max_mm` | 0.0 | Largest tool radius; 0 means arm1+arm2 |
| `x_min_mm` | −100000 | Straight-line blockage |

Joint windows need not straddle zero: `j1_min_deg: 20, j1_max_deg: 300` works,
angles are wrapped into the configured window.

## Things to get right

- **`mpos_mm` on a joint is an angle**, the joint's degrees at its limit
  switch. Wrong value ⇒ the whole coordinate system is rotated.
- **`soft_limits` must be `false` on the joints.** That machinery compares
  cartesian targets against `max_travel_mm`, which is an angle here. Reach and
  joint range are enforced by this module instead.
- **`steps_per_mm` on a joint is steps per degree:**
  `(steps_per_rev × microsteps × gear_ratio) / 360`.
- **Workspace is an annulus**, from `|L1−L2|` to `L1+L2` from the shoulder.
  Targets in the inner dead zone are rejected.
- **The elbow branch is fixed for the whole job.** The arm cannot flip branches
  mid-move; pick the one that keeps the arm clear over your entire work area.

## Testing

`test_scara.cpp` stubs out FluidNC and `#include`s the transform functions
extracted verbatim from `Scara.cpp`, so it exercises the real math:

```sh
# regenerate the extracted fragments, then build and run
awk '/^    float Scara::wrap_degrees/,/^    }$/'                 Scara.cpp > wrap.inc
awk '/^    bool Scara::transform_cartesian_to_motors/,/^    }$/' Scara.cpp > ik.inc
awk '/^    void Scara::motors_to_cartesian/,/^    }$/'           Scara.cpp > fk.inc
g++ -O2 -Wall -o test_scara test_scara.cpp && ./test_scara
```

Covers: FK(IK(p)) round-trip over the workspace, reach and dead-zone
boundaries, both elbow branches, angle wrapping for a non-zero-straddling joint
window, and joint-limit rejection.

Current result: 30,387 reachable points, worst round-trip error 0.000123 mm.

### Cross-checked against an independent implementation

`test_vs_grbl.cpp` transcribes the inverse kinematics of
[jared-hughes/grbl-scara](https://github.com/jared-hughes/grbl-scara)
(`grbl/system.c:339`) and compares it to this one over 44,508 points. Their
`quick_atan2` polynomial is replaced by exact `atan2` so the comparison is of
the maths, not their approximation.

Worst difference: **0.000055°** on both joints — float rounding.

The algebra is written differently: they use a half-angle form,
`theta = 2·atan2(√(r²−sm), √(s−r²))` with `theta` the *interior* elbow angle,
while this module uses the law of cosines on the *exterior* angle. The two are
related by `θ2 = ±(π − theta)`.

Their `target_steps[L_MOTOR] = steps_per_mm · (theta − π + angle)` is
`θ1 + θ2`, the **absolute** forearm angle — independently confirming the
`coupled_elbow` convention this module implements. Their default arm lengths
are 120/120, the same as this machine.

The work-envelope guards are modelled on their `limits.c:359`.

## Comparison with other SCARA implementations

| | this module | grbl-scara | Marlin | RepRapFirmware |
| --- | --- | --- | --- | --- |
| IK maths | law of cosines | half-angle `atan2` | adapted from RRF | law of cosines |
| agreement with this module | — | 0.000055° over 44,508 pts | same formula family | same formula family |
| coupled (absolute) forearm | `coupled_elbow` | always | `SCARA_CROSSTALK_FACTOR` | `crosstalk[0]` |
| elbow branch | **read from the motors** | fixed | fixed, `SCARA_ELBOW_DIR` | **read from the motors** |
| a move may flip the branch | never | n/a | n/a | only uncoordinated moves |
| singularity guard | `r_min_mm` / `r_max_mm` | `$31` / `$32` radius | none in the IK | `|cos ψ| > 0.995` rejected |
| segmentation | by mm **and** by degrees | by mm | per second | per segment time |
| state after a reset | FK staleness check, resync | n/a | n/a | resynced from motors |

Two defects were found through this comparison and fixed:

1. **Elbow branch taken from the config, not from the arm.** A fixed flag that
   disagreed with the homed pose made every move, even 1 mm, solve for the
   mirror-image arm: about 150° on each joint, at max rate. RRF derives the
   branch from the motor positions (`currentArmMode = motorPos[Y] >= 0`); this
   module now does the same after homing and on every resync, and the IK never
   leaves that branch.
2. **Stale motor bookkeeping.** `_last_motor_pos` was only written by segments
   and homing, so after a reset, alarm or cancelled jog the next move measured
   its joint travel — and scaled its feed rate — from a position the arm never
   reached. FluidNC's own `ParallelDelta` follows the same pattern. It is now
   compared with the planner's start through forward kinematics (unique, so
   safe even near a singularity) and resynced from the step counters when stale.

`test_branch.cpp` covers both, using the functions extracted from `Scara.cpp`.

## Status

The math is tested on the host as described. The module has **not** been
compiled against FluidNC or run on hardware here — no ESP32 toolchain in this
environment. Expect to work through compile fixes on the first build, and
verify homing direction and `mpos_mm` with the arm clear of the work area.
