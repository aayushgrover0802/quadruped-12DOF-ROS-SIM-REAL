"""
test_ik.py -- verifies ik.py the same way you'd verify it in Isaac Sim:
solve for a target foot position, then feed the result BACK through FK and
confirm you land back on that same target. If FK(IK(target)) != target,
something's wrong.
"""
import sys
import os
import yaml
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk import FKSolver
from ik import IKSolver


def main():
    descriptor_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_descriptor.yaml")
    with open(descriptor_path) as f:
        descriptor = yaml.safe_load(f)

    ik = IKSolver(descriptor)
    fk = FKSolver(descriptor)

    print("=" * 70)
    print("Test: solve IK for a target, then verify via FK round-trip")
    print("=" * 70)

    # Use FR_Foot's own Test-1 neutral position as a real, known-reachable
    # target -- if IK can't even hit a point we KNOW is reachable (since we
    # got it from FK in the first place), something is fundamentally broken.
    q_neutral = fk.neutral_q()
    known_reachable_target = fk.foot_position("FR", q_neutral)
    print(f"\nTarget (FR_Foot's own neutral-pose position): {known_reachable_target}")

    success, q_solution, error_mm = ik.solve("FR", known_reachable_target)
    print(f"IK converged: {success}   final error: {error_mm:.4f} mm")

    achieved_pos = fk.foot_position("FR", q_solution)
    print(f"FK(IK solution) achieved position: {achieved_pos}")
    print(f"Round-trip error: {np.linalg.norm(achieved_pos - known_reachable_target)*1000:.4f} mm")

    fr_angles = ik.get_leg_angles("FR", q_solution)
    print(f"\nSolved joint angles:")
    for jname, angle in fr_angles.items():
        print(f"  {jname}: {angle:+.4f} rad ({np.degrees(angle):+.2f} deg)")

    print()
    print("=" * 70)
    print("Test 2: a genuinely different, REACHABLE target")
    print("(derived from a real valid joint configuration, not an arbitrary offset)")
    print("=" * 70)
    # Pick angles well inside FR's actual limits, get FK's honest answer for
    # where that puts the foot -- guarantees the target is truly reachable,
    # unlike an arbitrary xyz offset that might violate this robot's
    # (fairly narrow) real joint ranges.
    q_valid = fk.neutral_q()
    q_valid[fk.joint_index("FR_HipRoll")] = 0.0
    q_valid[fk.joint_index("FR_ThighP")] = 0.09948377
    q_valid[fk.joint_index("FR_KneeP")] = 0.09948377
    new_target = fk.foot_position("FR", q_valid)
    print(f"Target (from FR_HipRoll=0.0, FR_ThighP=0.09948377, FR_KneeP=0.09948377): {new_target}")

    # Solve starting from neutral (0,0,0) -- IK has to find its own way there,
    # doesn't get to start from the answer like Test 1 did.
    success, q_solution, error_mm = ik.solve("FR", new_target)
    print(f"IK converged: {success}   final error: {error_mm:.4f} mm")

    achieved_pos = fk.foot_position("FR", q_solution)
    print(f"FK(IK solution) achieved position: {achieved_pos}")
    print(f"Round-trip error: {np.linalg.norm(achieved_pos - new_target)*1000:.4f} mm")

    solved_angles = ik.get_leg_angles("FR", q_solution)
    print(f"Solved angles: {solved_angles}")
    print(f"(Original angles were: HipRoll=0.09948377, ThighP=0.09948377, KneeP=0.09948377 -- IK "
          f"doesn't need to match these exactly, just reach the same position,"
          f" but close agreement is a good sign)")


if __name__ == "__main__":
    main()
