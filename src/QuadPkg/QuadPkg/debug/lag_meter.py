"""
lag_meter.py -- `ros2 run QuadPkg lag_meter [--seconds 10]`

Measures how late each servo follows its command and how much of the movement it really makes,
then suggests a gait_period that gives the servos enough time.
Read-only: it only listens. Start it, then walk the robot forward for the whole recording.

  real robot : compares /debug/joint_sent (what hardware_bridge sent, 50 Hz) with /joint_states
  Isaac Sim  : compares /joint_command with /joint_states (if /debug/joint_sent doesn't exist)

Output per joint:
  lag ms      how late the servo follows the command (best time shift)
  err now     RMS error as servo_monitor sees it (deg)
  err @lag    RMS error left after removing the delay (deg) -- sag, friction, too weak
  reach %     measured swing size / sent swing size (100 % = full movement)

What matters is NOT the lag itself (the same delay on every joint does no harm, the whole
trot just happens a bit later), but:
  * knees later than thighs   -> the foot path gets bent: it swings forward before it is lifted
                                 and is still up when it should land. Fix: knee_lead (it prints it)
  * reach % well under 100    -> the swing is too short for the servos, steps shrink. Fix: longer
                                 gait_period and/or lower step_height
  * one joint worse than its left-right twin -> that servo or leg has its own problem
Also run `ros2 run QuadPkg servo_tool step <joint> 10` (robot lifted) to see how fast ONE servo
reacts on its own, without ROS in between.

Options
  --seconds S         recording length (default 10)
  --params FILE       quad_params.yaml to read gait_period / knee_lead (default: installed one)
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

from QuadPkg.debug.lag_math import estimate_lag, odd_joints, recommend
from QuadPkg.debug.monitor_table import ORDER


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class LagMeter(Node):
    def __init__(self):
        super().__init__("lag_meter")
        self.cmd = {j: ([], []) for j in ORDER}
        self.meas = {j: ([], []) for j in ORDER}
        self.n_sent = self.n_cmd = 0
        self.create_subscription(JointState, "/debug/joint_sent", self.on_sent, 50)
        self.create_subscription(JointState, "/joint_command", self.on_cmd, 50)
        self.create_subscription(JointState, "/joint_states", self.on_meas, 50)
        self.cmd_cmdtopic = {j: ([], []) for j in ORDER}

    @staticmethod
    def _add(store, msg):
        t = _stamp(msg)
        for name, x in zip(msg.name, msg.position):
            if name in store:
                store[name][0].append(t)
                store[name][1].append(x)

    def on_sent(self, msg):
        self.n_sent += 1
        self._add(self.cmd, msg)

    def on_cmd(self, msg):
        self.n_cmd += 1
        self._add(self.cmd_cmdtopic, msg)

    def on_meas(self, msg):
        self._add(self.meas, msg)


def _gait_params(path):
    if path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("QuadPkg"), "config", "quad_params.yaml")
        except Exception:
            path = os.path.join(os.path.dirname(__file__), "..", "..", "config", "quad_params.yaml")
    try:
        with open(path, encoding="utf-8") as f:
            p = yaml.safe_load(f)["quad_controller"]["ros__parameters"]
        return (float(p.get("gait_period", 0.85)), float(p.get("duty_factor", 0.65)),
                float(p.get("knee_lead", 0.0)))
    except Exception:
        return 0.85, 0.65, 0.0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    own = argv[:argv.index("--ros-args")] if "--ros-args" in argv else argv
    ap = argparse.ArgumentParser(prog="lag_meter", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=10.0, help="how long to record (walk the whole time)")
    ap.add_argument("--params", default=None, help="quad_params.yaml (default: installed)")
    a = ap.parse_args(own)

    rclpy.init(args=None)
    node = LagMeter()
    print(f"Recording {a.seconds:.0f} s -- make the robot WALK FORWARD now (press W in teleop)...", flush=True)
    end = time.monotonic() + a.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass

    if node.n_sent > 20:
        src, cmd = "/debug/joint_sent (real robot)", node.cmd
    else:
        src, cmd = "/joint_command (sim, or bridge debug_topics off)", node.cmd_cmdtopic
    n_meas = len(node.meas[ORDER[0]][0])
    print(f"\ncommand source: {src}   samples: {len(cmd[ORDER[0]][0])} sent, {n_meas} measured")
    if n_meas < 20:
        print("Not enough /joint_states -- is hardware_bridge (or Isaac) running?")
        return 1

    results = {}
    print(f"\n{'joint':11s} {'lag ms':>7s} {'err now':>8s} {'err @lag':>9s} {'reach %':>8s}")
    print("-" * 47)
    for j in ORDER:
        r = estimate_lag(*cmd[j], *node.meas[j])
        results[j] = r
        if not r["ok"]:
            print(f"{j:11s} {'-':>7s}   (hardly moved -- walk forward during the recording)")
            continue
        flag = "  <-- late" if r["lag"] > 0.08 else ""
        print(f"{j:11s} {r['lag'] * 1000:7.0f} {math.degrees(r['err_now']):7.1f}° "
              f"{math.degrees(r['err_at_lag']):8.1f}° {r['amp_ratio'] * 100:7.0f}%{flag}")

    n_sent = len(cmd[ORDER[0]][0])
    if n_meas > 1.3 * max(n_sent, 1):
        print(f"\nWARNING: {n_meas} /joint_states messages but only {n_sent} commands -- is a SECOND node "
              "publishing /joint_states?\n  Check:  ros2 topic info -v /joint_states   (must be 1 publisher: "
              "hardware_bridge). Isaac Sim on the same ROS_DOMAIN_ID does this.")

    period, duty, knee_lead_now = _gait_params(a.params)
    rec = recommend(results)
    print()
    for line in odd_joints(results):
        print("ODD LEG: " + line + "  -> check that servo's power, connector, horn and binding")
    if rec is None:
        print("No thigh/knee moved enough. Record again while walking (W) for the whole time.")
    else:
        print(f"thighs follow {rec['thigh_lag'] * 1000:.0f} ms late, knees {rec['knee_lag'] * 1000:.0f} ms late "
              f"-> knees are {(rec['knee_lag'] - rec['thigh_lag']) * 1000:.0f} ms behind their thighs")
        print(f"swing time now: {(1 - duty) * period * 1000:.0f} ms (gait_period {period}, duty_factor {duty})")
        if abs(rec["knee_lead"] - knee_lead_now) >= 0.01:
            print(f"SET in config/quad_params.yaml:  knee_lead: {rec['knee_lead']:.2f}   (now {knee_lead_now:.2f})")
        else:
            print(f"knee_lead {knee_lead_now:.2f} already matches -- thigh and knee arrive together")
        if rec["low_reach"]:
            print("Short movements (reach < 85%): " + ", ".join(rec["low_reach"]))
            print(f"  -> give the servos more time:  gait_period: {min(period + 0.15, 1.3):.2f}  "
                  "and/or  step_height: 0.015")
    print("Rebuild after editing, then run lag_meter again.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
