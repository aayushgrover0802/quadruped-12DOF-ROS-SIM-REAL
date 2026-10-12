#!/usr/bin/env python3
"""
rezero_urdf.py -- make "all joints = 0" mean "robot is STANDING".

WHY
---
The CAD export put joint zero wherever the legs happened to be in SolidWorks.
On the real robot you set every servo's middle position (tick 2048) with the
legs in the standing pose. To make sim and real agree, joint angle 0 must mean
the SAME pose in both places -> we rotate each joint's zero in the URDF.

HOW (the maths, in one line per joint)
--------------------------------------
For a revolute joint with axis `a` and standing angle q0 (in the OLD URDF):
    new_origin_rotation = old_origin_rotation @ Rot(a, q0)
    new_limits          = old_limits - q0
The child link (its mesh, mass, and the next joint) is attached to the child
frame, so it simply moves with it. Nothing else in the file changes.
After this, FK(new, q) == FK(old, q + q0) for every q.

WHICH POSE IS "STANDING"?
-------------------------
Feet below their CAD rest position (same X as the old zero pose), moved
Y_SHIFT forward/back so the 4 feet are centred under the centre of mass
(with the 2 kg payload the feet must sit ~16 mm further back), and the body
STAND_HEIGHT above the feet (same number as stand_height in
config/quad_params.yaml). Result: the controller's STAND pose is (almost
exactly) all-zeros. If you later add payload that moves the COM, the
controller re-centres automatically on top of this -- no need to re-run.

USAGE (run once; already done for you -- only re-run after a new CAD export)
    cd src/QuadPkg
    python3 tools/rezero_urdf.py urdf/QuadURDFcad4_0_cadzero.urdf \
                                 urdf/QuadURDFcad4_0.urdf --stand-height 0.155 --y-shift -0.016
ALWAYS feed it the CAD-zero file, never its own output (it would shift twice).
"""
import argparse
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from QuadPkg.kinematics.leg_kinematics import (QuadKinematics, _axis_angle_matrix,  # noqa: E402
                                               _rpy_to_matrix)


def matrix_to_rpy(R):
    """Inverse of the URDF convention R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) < 1e-9:          # gimbal lock, never happens for this robot
        roll, yaw = 0.0, math.atan2(-R[0, 1], R[1, 1])
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def standing_angles(urdf_path, stand_height, y_shift=0.0):
    kin = QuadKinematics(urdf_path)
    targets = {}
    for name, leg in kin.legs.items():
        targets[name] = np.array([leg.neutral_foot[0], leg.neutral_foot[1] + y_shift, -stand_height])
    angles, bad = kin.solve(targets)
    if bad:
        raise RuntimeError(f"standing pose unreachable for legs {bad}")
    return kin, angles


def rezero(src, dst, stand_height, y_shift=0.0):
    kin, q0 = standing_angles(src, stand_height, y_shift)
    text = open(src, encoding="utf-8").read()

    for jname, q in q0.items():
        if abs(q) < 1e-12:
            continue
        joint = kin.model.joints[jname]
        block_re = re.compile(r'<joint\s+name="%s".*?</joint>' % re.escape(jname), re.S)
        m = block_re.search(text)
        if m is None:
            raise RuntimeError(f"joint {jname} not found in {src}")
        block = m.group(0)

        # 1) rotate the joint origin by q0 about the joint axis
        om = re.search(r'(<origin\s+xyz="[^"]*"\s+rpy=")([^"]*)(")', block)
        rpy = [float(v) for v in om.group(2).split()]
        R_new = _rpy_to_matrix(*rpy) @ _axis_angle_matrix(joint["axis"], q)
        new_rpy = " ".join(f"{v:.8g}" for v in matrix_to_rpy(R_new))
        block = block[:om.start(2)] + new_rpy + block[om.end(2):]

        # 2) shift the limits so the physical range stays the same
        def shift(attr, blk):
            lm = re.search(r'(%s=")([^"]*)(")' % attr, blk)
            val = float(lm.group(2)) - q
            return blk[:lm.start(2)] + f"{val:.6g}" + blk[lm.end(2):]
        block = shift("lower", block)
        block = shift("upper", block)
        text = text[:m.start()] + block + text[m.end():]

    note = ["  <!-- RE-ZEROED by tools/rezero_urdf.py: all joints = 0 is now the STANDING pose",
            f"       (body {stand_height} m above the feet, feet shifted {y_shift * 1000:+.1f} mm in Y to sit",
            f"       under the COM). Source: {os.path.basename(src)}.",
            "       Standing angles in the OLD (CAD) zero that became the new 0 [rad]:"]
    for jname, q in q0.items():
        note.append(f"         {jname:12s} {q:+.6f}")
    note.append("  -->")
    text = re.sub(r'(<robot\s+name="[^"]*">)', lambda mm: mm.group(1) + "\n" + "\n".join(note), text, count=1)
    with open(dst, "w", encoding="utf-8") as f:
        f.write(text)
    return q0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", help="URDF with the CAD zero (QuadURDFcad4_0_cadzero.urdf)")
    ap.add_argument("dst", help="output URDF (zero = standing)")
    ap.add_argument("--stand-height", type=float, default=0.155)
    ap.add_argument("--y-shift", type=float, default=-0.016,
                    help="move feet forward(+)/back(-) so they are centred under the COM [m]")
    a = ap.parse_args()
    if os.path.abspath(a.src) == os.path.abspath(a.dst):
        sys.exit("src and dst must differ (keep the CAD-zero file as the source of truth)")
    q0 = rezero(a.src, a.dst, a.stand_height, a.y_shift)

    # verify: FK(new, 0) == FK(old, q0)
    old, new = QuadKinematics(a.src), QuadKinematics(a.dst)
    f_old, f_new = old.foot_positions(q0), new.foot_positions({})
    err = max(np.linalg.norm(f_old[n] - f_new[n]) for n in f_old)
    print(f"wrote {a.dst}; max foot mismatch {err * 1000:.4f} mm")
    for n, leg in new.legs.items():
        print(f"  {n}: foot at zero = {np.round(leg.neutral_foot, 4)}")


if __name__ == "__main__":
    main()
