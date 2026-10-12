"""
Tests for the real-robot additions (no ROS, no hardware needed):
    cd src/QuadPkg && python3 -m pytest test/ -v

  * re-zeroed URDF: all joints 0 == standing pose, same geometry as the CAD URDF
  * payload.yaml is merged into the mass model
  * Jetson <-> ESP32 frame encoding / decoding
  * joint angle <-> servo tick conversion
"""
import math
import os
import struct
import sys

import numpy as np
import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, ROOT)

from QuadPkg.description.robot_description import load_robot_xml, load_payload  # noqa: E402
from QuadPkg.gait.controller import GaitController, GaitParams  # noqa: E402
from QuadPkg.hardware import protocol as P  # noqa: E402
from QuadPkg.hardware.servo_map import DEFAULT_IDS, DEFAULT_ORDER, ServoMap  # noqa: E402
from QuadPkg.kinematics.leg_kinematics import QuadKinematics  # noqa: E402

PARSER = os.path.join(ROOT, "urdf")
URDF = os.path.join(PARSER, "QuadURDFcad4_0.urdf")
URDF_CAD = os.path.join(PARSER, "QuadURDFcad4_0_cadzero.urdf")
PAYLOAD = os.path.join(ROOT, "config", "payload.yaml")


def standing_offsets():
    """Read the q0 values that rezero_urdf.py wrote into the URDF header comment."""
    q0 = {}
    for line in open(URDF):
        parts = line.split()
        if len(parts) == 2 and parts[0].endswith(("HipRoll", "ThighP", "KneeP")):
            q0[parts[0]] = float(parts[1])
    assert len(q0) == 12
    return q0


def test_rezeroed_urdf_matches_cad_geometry():
    """FK(new, q) must equal FK(cad, q + q0) everywhere -> only the zero moved."""
    new, cad, q0 = QuadKinematics(URDF), QuadKinematics(URDF_CAD), standing_offsets()
    rng = np.random.default_rng(3)
    for _ in range(200):
        q = {j: rng.uniform(-0.6, 0.6) for j in new.joint_names}
        a, b = new.foot_positions(q), cad.foot_positions({j: q[j] + q0[j] for j in q})
        for leg in a:
            assert np.linalg.norm(a[leg] - b[leg]) < 1e-6


def test_zero_is_the_controller_stand_pose_with_payload():
    kin = QuadKinematics(load_robot_xml(URDF, PAYLOAD))
    ctrl = GaitController(kin, GaitParams())
    stand = ctrl._pose_angles(ctrl.p.stand_height)
    assert max(abs(v) for v in stand.values()) < 0.01      # < 0.6 degree everywhere


def test_controller_starts_standing():
    kin = QuadKinematics(load_robot_xml(URDF, PAYLOAD))
    ctrl = GaitController(kin, GaitParams())
    ctrl.reset({j: 0.3 for j in kin.joint_names})         # robot somewhere else at boot
    for _ in range(200):
        angles = ctrl.update(0.02)
    assert ctrl.state == "STAND"
    assert max(abs(v) for v in angles.values()) < 0.01


def test_payload_adds_mass():
    bare = QuadKinematics(load_robot_xml(URDF, ""))
    full = QuadKinematics(load_robot_xml(URDF, PAYLOAD))
    extra = sum(it["mass"] for it in load_payload(PAYLOAD))
    assert extra == pytest.approx(2.0)
    assert full.center_of_mass({})[1] == pytest.approx(bare.center_of_mass({})[1] + extra)


def test_payload_item_can_be_added(tmp_path):
    p = tmp_path / "p.yaml"
    p.write_text(open(PAYLOAD).read() + "\n  - name: lidar\n    mass: 0.17\n    xyz: [0, 0.08, 0.1]\n"
                 "    size: [0.07, 0.1, 0.06]\n")
    kin = QuadKinematics(load_robot_xml(URDF, str(p)))
    assert kin.center_of_mass({})[1] == pytest.approx(QuadKinematics(load_robot_xml(URDF, PAYLOAD))
                                                      .center_of_mass({})[1] + 0.17)


def test_frame_round_trip():
    frame = P.build_targets([2048, 100, 4095], speed=0, acc=10)
    frames = P.FrameParser().feed(b"\x00\x13garbage" + frame[:5])  # split in two chunks
    assert frames == []
    # a broken header (with a huge fake length) must not lose the good frames behind it
    parser = P.FrameParser()
    out = parser.feed(b"\xAA\x55\x01junk" + frame * 6)
    assert len(out) == 6 and all(f[0] == P.TARGETS for f in out)
    n = out[0][1][0]
    vals = [struct.unpack_from("<hHB", out[0][1], 1 + 5 * i) for i in range(n)]
    assert vals == [(2048, 0, 10), (100, 0, 10), (4095, 0, 10)]


def test_bad_crc_is_rejected():
    frame = bytearray(P.encode(P.HELLO, b"\x01\x02"))
    frame[-1] ^= 0xFF
    assert P.FrameParser().feed(bytes(frame)) == []


def test_decode_state():
    payload = struct.pack("<IB", 1234, 2)
    payload += struct.pack("<BhhhBBh", 7, 2048, -15, 300, 120, 41, 20)
    payload += struct.pack("<BhhhBBh", 0, 0, 0, 0, 0, 0, 0)
    t, s = P.decode_state(payload)
    assert t == 1234 and s[0].online and s[0].torque and s[0].pos == 2048
    assert s[0].voltage == pytest.approx(12.0) and s[0].temp == 41 and not s[1].online


def test_servo_map_zero_is_2048_and_round_trips():
    m = ServoMap(DEFAULT_ORDER, DEFAULT_IDS, [1.0, -1.0] * 6, [5] * 12, [1.0] * 12)
    assert list(m.joint_to_ticks(np.zeros(12))) == [2053] * 12
    q = np.linspace(-1.0, 1.0, 12)
    back = m.ticks_to_joint(m.joint_to_ticks(q))
    assert np.max(np.abs(back - q)) <= math.pi / 4096 + 1e-9    # within half a tick
    t = m.joint_to_ticks(np.full(12, 10.0))                      # crazy command -> clamped
    assert t.min() >= 0 and t.max() <= 4095


def test_default_ids_follow_wiring():
    """IDs 1..12 = FL knee, thigh, hip, FR knee, thigh, hip, BL ..., BR ..."""
    by_id = dict(zip(DEFAULT_IDS, DEFAULT_ORDER))
    assert [by_id[i] for i in range(1, 13)] == [
        "FL_KneeP", "FL_ThighP", "FL_HipRoll", "FR_KneeP", "FR_ThighP", "FR_HipRoll",
        "BL_KneeP", "BL_ThighP", "BL_HipRoll", "BR_KneeP", "BR_ThighP", "BR_HipRoll"]
