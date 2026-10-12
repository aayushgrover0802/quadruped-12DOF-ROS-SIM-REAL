"""
test_fk.py -- loads robot_descriptor.yaml and runs forward kinematics at a
few joint configurations, for every leg, to sanity-check the whole pipeline
before wiring up IK or gait code.

Usage (from workspace/):
    python3 test_fk.py
"""
import sys
import os
import yaml
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk import FKSolver


def main():
    descriptor_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_descriptor.yaml")
    with open(descriptor_path) as f:
        descriptor = yaml.safe_load(f)

    solver = FKSolver(descriptor)

    print(f"Robot: {descriptor['robot_name']}")
    print(f"Legs: {list(descriptor['legs'].keys())}\n")

    # Test 1: all joints at neutral (0)
    print("=" * 60)
    print("Test 1: all joints at q=0")
    print("=" * 60)
    q = solver.neutral_q()
    positions = solver.all_foot_positions(q)
    for leg_name, pos in positions.items():
        print(f"  {leg_name}_Foot: x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}  (meters)")

    # Test 2: bend FR's knee joint partway through its valid range and watch
    # the foot move -- confirms the kinematic chain actually responds, and
    # gives you a real number to sanity-check against your CAD by hand.
    print()
    print("=" * 60)
    print("Test 2: FR_KneeP swept from lower to upper limit")
    print("=" * 60)
    fr_knee_limits = descriptor["legs"]["FR"]["joint_limits"]["FR_KneeP"]
    knee_idx = solver.joint_index("FR_KneeP")
    for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        q = solver.neutral_q()
        angle = fr_knee_limits["lower"] + frac * (fr_knee_limits["upper"] - fr_knee_limits["lower"])
        q[knee_idx] = angle
        pos = solver.foot_position("FR", q)
        print(f"  knee={angle:+.4f} rad ({np.degrees(angle):+6.1f} deg)  ->  "
              f"FR_Foot: x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}")

    print()
    print("If Test 2's foot position changes smoothly and doesn't jump/NaN "
          "anywhere, FK is working correctly end to end.")


if __name__ == "__main__":
    main()
