"""
Run with:   cd src/QuadPkg && python3 -m pytest test/test_kinematics_and_gait.py -v
(No ROS needed -- only numpy + pytest.)
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from QuadPkg.gait.controller import GaitController, GaitParams  # noqa: E402
from QuadPkg.kinematics.leg_kinematics import QuadKinematics  # noqa: E402

URDF = os.path.join(os.path.dirname(__file__), "..", "urdf", "QuadURDFcad4_0.urdf")


@pytest.fixture(scope="module")
def kin():
    return QuadKinematics(URDF)


def test_ik_round_trip_over_walking_workspace(kin):
    """FK(IK(p)) must land back on p everywhere the gait can ask for."""
    rng = np.random.default_rng(0)
    for leg in kin.legs.values():
        for _ in range(500):
            p = leg.neutral_foot + [rng.uniform(-0.02, 0.02), rng.uniform(-0.05, 0.05), 0.0]
            p[2] = -rng.uniform(0.08, 0.17)
            angles, ok = leg.ik(p)
            assert ok, (leg.name, p)
            assert np.linalg.norm(leg.fk(angles) - p) < 1e-6


def test_neutral_pose_is_zero_angles(kin):
    for leg in kin.legs.values():
        angles, ok = leg.ik(leg.neutral_foot)
        assert ok and np.allclose(angles, 0.0, atol=1e-6)


def test_fk_matches_pinocchio_if_available(kin):
    pin = pytest.importorskip("pinocchio")
    model = pin.buildModelFromUrdf(URDF)
    data = model.createData()
    rng = np.random.default_rng(1)
    for _ in range(200):
        q = pin.neutral(model)
        qd = {}
        for j in kin.joint_names:
            qd[j] = rng.uniform(-0.7, 0.7)
            q[model.joints[model.getJointId(j)].idx_q] = qd[j]
        pin.framesForwardKinematics(model, data, q)
        ours = kin.foot_positions(qd)
        for name in kin.legs:
            ref = data.oMf[model.getFrameId(f"{name}_Foot")].translation
            assert np.linalg.norm(ours[name] - ref) < 1e-4


def test_gait_is_smooth_and_reachable(kin):
    """Whole scenario: no joint may jump more than 0.2 rad between 20 ms ticks."""
    ctrl = GaitController(kin, GaitParams())
    script = [(3, "sit", (0, 0, 0)), (2.5, "stand", (0, 0, 0)), (3, "stand", (0.25, 0, 0)),
              (2, "stand", (0, 0, 1.0)), (2, "stand", (0, 0.12, 0)), (2, "stand", (0, 0, 0)),
              (2.5, "sit", (0, 0, 0))]
    prev, dt, states = None, 0.02, []
    for dur, mode, cmd in script:
        ctrl.set_mode(mode)
        ctrl.set_command(*cmd)
        for _ in range(int(dur / dt)):
            angles = ctrl.update(dt)
            if prev is not None:
                assert max(abs(angles[j] - prev[j]) for j in angles) < 0.2
            prev = angles
        states.append(ctrl.state)
    assert ctrl.unreachable_count == 0
    assert states == ["SIT", "STAND", "WALK", "WALK", "WALK", "STAND", "SIT"]
