"""
verify_sim_ik.py -- Validates IK against ground truth Isaac Sim telemetry.
Tests:
  1. IK convergence given Isaac Sim target foot positions
  2. Round-trip FK error (achieved position vs sim target position)
  3. Direct angle comparison (calculated angles vs sim angles)
"""

import sys
import os
import yaml
import numpy as np

# Ensure kinematics modules are visible
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk import FKSolver
from ik import IKSolver


def run_verification():
    descriptor_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_descriptor.yaml")
    if not os.path.exists(descriptor_path):
        print(f"[ERROR] Descriptor file not found at: {descriptor_path}")
        return

    with open(descriptor_path, "r") as f:
        descriptor = yaml.safe_load(f)

    fk = FKSolver(descriptor)
    ik = IKSolver(descriptor)

    # Leg to verify (defaulting to Front Right - FR)
    leg_name = "FR"

    # Joint order in descriptor: [HipRoll, ThighP, KneeP]
    # Input angles: [(knee, thigh, hip)] in degrees -> converted to [hip, thigh, knee] in radians
    sim_data = [
        {
            "test_name": "Test Case 1 (Neutral Pose)",
            "sim_angles_deg": {"knee": 0.0, "thigh": 0.0, "hip": 0.0},
            "sim_foot_pos": np.array([0.1348406, 0.12740332, -0.13273036])
        },
        {
            "test_name": "Test Case 2 (Dynamic Pitch Pose)",
            "sim_angles_deg": {"knee": 5.7, "thigh": 5.7, "hip": 0.0},
            "sim_foot_pos": np.array([0.13433112, 0.1511074, -0.11912753])
        }
    ]

    leg_joints = descriptor["legs"][leg_name]["joints"]
    # Mapping descriptor joint names
    hip_joint = leg_joints[0]
    thigh_joint = leg_joints[1]
    knee_joint = leg_joints[2]

    print("=" * 80)
    print(f"  ISAAC SIM vs PINOCCHIO IK/FK VERIFICATION PIPELINE [{leg_name} LEG]")
    print("=" * 80)

    for i, test in enumerate(sim_data, 1):
        print(f"\n>>> [{test['test_name']}]")
        target_pos = test["sim_foot_pos"]
        angles_deg = test["sim_angles_deg"]

        # Convert input degrees to radians in standard joint mapping
        sim_q_rad = {
            hip_joint: np.radians(angles_deg["hip"]),
            thigh_joint: np.radians(angles_deg["thigh"]),
            knee_joint: np.radians(angles_deg["knee"])
        }

        print(f"Target Isaac Sim Foot Pos (XYZ): [{target_pos[0]:+.6f}, {target_pos[1]:+.6f}, {target_pos[2]:+.6f}] m")

        # Step 1: Solve IK for target foot pos starting from neutral q
        success, q_solution, ik_error_mm = ik.solve(leg_name, target_pos, max_iters=150, tol=1e-5)
        solved_angles = ik.get_leg_angles(leg_name, q_solution)

        # Step 2: Feed calculated IK angles back through FK
        achieved_pos = fk.foot_position(leg_name, q_solution)
        round_trip_err_mm = np.linalg.norm(achieved_pos - target_pos) * 1000.0

        # Step 3: Print IK & Round-Trip verification
        print(f"\n  [IK Solver Result]")
        print(f"  * Converged:          {success}")
        print(f"  * Solver Error:       {ik_error_mm:.5f} mm")
        print(f"  * FK Round-Trip Pos:  [{achieved_pos[0]:+.6f}, {achieved_pos[1]:+.6f}, {achieved_pos[2]:+.6f}] m")
        print(f"  * Positional Error:   {round_trip_err_mm:.5f} mm")

        # Step 4: Joint Angle Comparison
        print(f"\n  [Joint Angle Comparison]")
        print(f"  {'Joint Name':<16} | {'Sim Angle (deg)':<16} | {'IK Solved (deg)':<16} | {'Diff (deg)':<10}")
        print("  " + "-" * 66)

        for j_name in leg_joints:
            sim_deg = np.degrees(sim_q_rad[j_name])
            calc_deg = np.degrees(solved_angles[j_name])
            diff_deg = abs(sim_deg - calc_deg)
            print(f"  {j_name:<16} | {sim_deg:>15.3f}° | {calc_deg:>15.3f}° | {diff_deg:>9.3f}°")

        # Validation Thresholds
        if round_trip_err_mm < 1.0:
            print("\n  >> STATUS: [PASSED] - Round-trip foot position verified (< 1.0 mm).")
        else:
            print("\n  >> STATUS: [FAILED] - High positional offset detected.")

        print("-" * 80)


if __name__ == "__main__":
    run_verification()