"""Tests for the debugging helpers (no ROS needed):  python3 -m pytest test/test_debug_tools.py -v"""
import math
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from QuadPkg.debug import check_robot  # noqa: E402
from QuadPkg.debug.monitor_table import ORDER, _status_dict, build_table  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..")
URDF = os.path.join(ROOT, "urdf", "QuadURDFcad4_0.urdf")
PAYLOAD = os.path.join(ROOT, "config", "payload.yaml")
PARAMS = os.path.join(ROOT, "config", "quad_params.yaml")


def _diag(rows):
    """Fake DiagnosticArray: rows = {joint: {key: value}}"""
    return NS(status=[NS(name=j, message="", hardware_id="servo id 1", level=0,
                         values=[NS(key=k, value=v) for k, v in kv.items()]) for j, kv in rows.items()])


def test_table_real_robot_flags():
    ctrl = {j: 0.0 for j in ORDER}
    ctrl["FL_KneeP"] = 1.2
    cmd_rows = {j: {"sent_deg": "+0.00", "sent_ticks": "2048"} for j in ORDER}
    cmd_rows["FL_KneeP"] = {"sent_deg": "+45.84", "sent_ticks": "2570",
                            "clamped_by_limit": "True", "slew_limited": "False"}
    cmd = _status_dict(_diag(cmd_rows))
    fb_rows = {j: {"online": "True", "torque_on": "True", "ticks": "2048", "angle_deg": "+0.00",
                   "load_pct": "+5", "temp_c": "35", "voltage_v": "11.8"} for j in ORDER}
    fb_rows["FL_KneeP"]["angle_deg"] = "+30.00"                      # lagging 15.8 deg
    fb_rows["BR_KneeP"] = {"online": "False", "ticks": "2048", "angle_deg": "+0.00"}
    text = build_table("both", ctrl, cmd, _status_dict(_diag(fb_rows)), {})
    knee = next(line for line in text.splitlines() if line.startswith("FL_KneeP"))
    assert "LIM" in knee and "ERR" in knee and "+15.8" in knee
    dead = next(line for line in text.splitlines() if line.startswith("BR_KneeP"))
    assert "OFF" in dead and "+0.0" not in dead.split("2048")[-1]   # stale angle hidden


def test_table_sim_mode_uses_joint_states():
    ctrl = {j: 0.1 for j in ORDER}
    meas = {j: 0.1 for j in ORDER}
    meas["FR_ThighP"] = 0.1 + math.radians(9)
    text = build_table("both", ctrl, {}, {}, meas)
    row = next(line for line in text.splitlines() if line.startswith("FR_ThighP"))
    assert "ERR" in row and "-9.0" in row


def test_check_robot_runs_and_flags_bad_payload(capsys):
    rc = check_robot.main(["--urdf", URDF, "--payload", PAYLOAD, "--params", PARAMS])
    assert rc in (0, 1, 2)
    rc2 = check_robot.main(["--urdf", URDF, "--payload", PAYLOAD, "--params", PARAMS,
                            "--add", "camera", "0.3", "0.15", "0.25", "0.0"])
    out = capsys.readouterr().out
    assert "outside the body footprint" in out and "to the right of the feet centre" in out
    assert rc2 >= 1


def test_lag_math_finds_delay_and_reach():
    import numpy as np
    from QuadPkg.debug.lag_math import estimate_lag, odd_joints, recommend
    t = np.arange(0, 8, 0.02)
    cmd = 0.2 * np.sin(2 * np.pi * t / 0.85)
    late = 0.2 * 0.7 * np.sin(2 * np.pi * (t - 0.10) / 0.85)        # 100 ms late, 70 % reach
    r = estimate_lag(t, cmd, t, late)
    assert r["ok"] and abs(r["lag"] - 0.10) < 0.011 and abs(r["amp_ratio"] - 0.7) < 0.05
    ok = estimate_lag(t, cmd, t, 0.2 * np.sin(2 * np.pi * (t - 0.04) / 0.85))
    res = {"FL_KneeP": r, "FR_KneeP": ok}
    assert any("FL_KneeP" in line for line in odd_joints(res))
    thigh = estimate_lag(t, cmd, t, 0.2 * np.sin(2 * np.pi * (t - 0.03) / 0.85))
    rec = recommend({"FL_ThighP": thigh, "FR_ThighP": thigh, "FL_KneeP": r, "FR_KneeP": r})
    assert abs(rec["knee_lead"] - 0.07) < 0.011 and "FL_KneeP" in rec["low_reach"]


def test_knee_lead_moves_only_knees_earlier():
    import numpy as np
    from QuadPkg.gait.controller import GaitController, GaitParams
    from QuadPkg.kinematics.leg_kinematics import QuadKinematics
    kin = QuadKinematics(URDF)

    def run(lead):
        c = GaitController(kin, GaitParams(gait_period=0.85, max_vx=0.1, knee_lead=lead))
        c.reset({})
        c.set_command(0.1, 0, 0)
        out = [c.update(0.02) for _ in range(400)]
        return (np.array([a["FR_KneeP"] for a in out]), np.array([a["FR_ThighP"] for a in out]))
    k0, t0 = run(0.0)
    k1, t1 = run(0.08)
    assert np.allclose(t0, t1)                                   # thighs untouched
    assert np.allclose(k1[300:350], k0[304:354], atol=1e-9)     # knees 4 ticks (80 ms) early
