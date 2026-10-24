#!/usr/bin/env python3
"""Compute steps_per_mm (= steps per degree) for the SCARA joints.

On a rotary axis FluidNC still calls the setting steps_per_mm, but the unit is
whatever the axis moves in - degrees here. For a GT2 belt reduction the ratio is
just the tooth count, since both pulleys share the same 2 mm pitch:

    steps_per_degree = motor_steps_per_rev * microsteps * (driven / driver) / 360

Arm lengths do not enter that number at all; they only say what the resulting
angular resolution is worth at the tool, which is what you actually care about:

    tip_resolution = radius * radians(degrees_per_step)

For J1 the radius is the whole arm extended (arm1 + arm2), because the shoulder
swings everything. For J2 it is arm2 alone.
"""

import argparse
import math


def steps_per_degree(pulses_per_rev, driver_teeth, driven_teeth):
    return pulses_per_rev * (driven_teeth / driver_teeth) / 360.0


def report(label, spd, radius):
    deg_per_step = 1.0 / spd
    print(f"{label}:")
    print(f"  steps_per_mm:  {spd:.4f}      # steps per DEGREE")
    print(f"  resolution:    {deg_per_step:.5f} deg/step")
    print(f"  at r={radius:g}mm:   {radius * math.radians(deg_per_step) * 1000:.1f} um/step")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arm1", type=float, default=180.0, help="shoulder to elbow, mm (default: 180)")
    p.add_argument("--arm2", type=float, default=120.0, help="elbow to tool, mm (default: 120)")
    p.add_argument("--j1", type=int, nargs=2, metavar=("DRIVER", "DRIVEN"), default=(20, 60),
                   help="J1 GT2 tooth counts, motor pulley then joint pulley (default: 20 60)")
    p.add_argument("--j2", type=int, nargs=2, metavar=("DRIVER", "DRIVEN"), default=(20, 60),
                   help="J2 GT2 tooth counts (default: 20 60)")
    p.add_argument("--ppr", type=int, default=None,
                   help="driver PULSE/REV, straight off the DIP table (e.g. 3200). "
                        "Already includes the motor's full steps, so it overrides --steps/--microsteps.")
    p.add_argument("--steps", type=int, default=200, help="motor full steps per rev (default: 200)")
    p.add_argument("--microsteps", type=int, default=16, help="driver microstepping (default: 16)")
    p.add_argument("--lead", type=float, default=8.0, help="Z leadscrew LEAD in mm, not pitch (default: 8)")
    args = p.parse_args()

    if args.ppr:
        ppr = args.ppr
        source = f"driver set to {ppr} pulse/rev"
    else:
        ppr = args.steps * args.microsteps
        source = f"motor {args.steps} steps/rev at 1/{args.microsteps}"

    j1 = steps_per_degree(ppr, *args.j1)
    j2 = steps_per_degree(ppr, *args.j2)
    z = ppr / args.lead

    print(f"{source}  ->  {ppr} pulses/rev\n")
    report(f"J1 (x)  {args.j1[1]}:{args.j1[0]} = {args.j1[1] / args.j1[0]:g}:1", j1, args.arm1 + args.arm2)
    report(f"J2 (y)  {args.j2[1]}:{args.j2[0]} = {args.j2[1] / args.j2[0]:g}:1", j2, args.arm2)
    print(f"Z       {args.lead:g}mm lead:")
    print(f"  steps_per_mm:  {z:.4f}")
    print(f"  resolution:    {1000 / z:.1f} um/step")

    print("\n--- config ---")
    print(f"  x:\n    steps_per_mm: {j1:.4f}")
    print(f"  y:\n    steps_per_mm: {j2:.4f}")
    print(f"  z:\n    steps_per_mm: {z:.4f}")


if __name__ == "__main__":
    main()
