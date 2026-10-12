"""
servo_monitor.py -- `ros2 run QuadPkg servo_monitor [--view both|cmd|fb] [--once] [--no-clear]`

A live terminal table of every servo: what you SEND vs. what comes BACK.
Read-only: it only subscribes, it never moves anything. Safe to run any time.

    --view cmd    only the command side   (/joint_command + /debug/servo_command)
    --view fb     only the feedback side  (/debug/servo_feedback)
    --view both   side by side (default)
    --once        print one table and quit (good for copy-pasting into a bug report)
    --no-clear    don't clear the screen, keep printing (good for `| tee log.txt`)
    --rate HZ     refresh rate of the table (default 4)

Where the numbers come from
  ctrl deg   /joint_command          what quad_controller wants (radians -> degrees)
  sent deg   /debug/servo_command    what hardware_bridge really sent after URDF limits + slew limit
  ticks>     /debug/servo_command    the raw servo position sent over USB (0..4095, 2048 = middle)
  <ticks     /debug/servo_feedback   the raw position the servo reports back
  meas deg   /debug/servo_feedback   that position converted back to a joint angle
  err deg    sent - meas             how far the servo is behind the command
  load %     servo load register     +-100 % of max torque
  flags      OFF = not answering, TQ0 = torque off, LIM = command clamped by URDF limit,
             SLEW = slew limiter active, HOT, LOWV, ERR = error above debug_error_warn_deg

In Isaac Sim there is no hardware_bridge, so only "ctrl" (/joint_command) and
"meas" (/joint_states) are shown -- still useful to see whether the joints follow.
"""
import argparse
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String

from QuadPkg.debug.monitor_table import _status_dict, build_table


class ServoMonitor(Node):
    def __init__(self, view):
        super().__init__("servo_monitor")
        self.view = view
        self.ctrl, self.meas, self.cmd, self.fb = {}, {}, {}, {}
        self.state, self.enabled, self.volts = "?", None, None
        self.t_cmd = self.t_fb = self.t_ctrl = self.t_meas = None
        self.create_subscription(JointState, "/joint_command", self.on_ctrl, 10)
        self.create_subscription(JointState, "/joint_states", self.on_meas, 10)
        self.create_subscription(DiagnosticArray, "/debug/servo_command", self.on_cmd, 10)
        self.create_subscription(DiagnosticArray, "/debug/servo_feedback", self.on_fb, 10)
        self.create_subscription(String, "/quad/state", lambda m: setattr(self, "state", m.data), 10)
        self.create_subscription(Bool, "/quad/motors_enabled", lambda m: setattr(self, "enabled", m.data), 10)
        self.create_subscription(Float32, "/quad/battery_voltage", lambda m: setattr(self, "volts", m.data), 10)

    def on_ctrl(self, m):
        self.ctrl, self.t_ctrl = dict(zip(m.name, m.position)), time.monotonic()

    def on_meas(self, m):
        self.meas, self.t_meas = dict(zip(m.name, m.position)), time.monotonic()

    def on_cmd(self, m):
        self.cmd, self.t_cmd = _status_dict(m), time.monotonic()

    def on_fb(self, m):
        self.fb, self.t_fb = _status_dict(m), time.monotonic()

    @staticmethod
    def _age(t):
        return "never" if t is None else f"{time.monotonic() - t:4.1f}s ago"

    def render(self):
        mode = "REAL ROBOT (hardware_bridge debug topics)" if (self.cmd or self.fb) else \
            "SIM / no bridge: ctrl=/joint_command vs meas=/joint_states"
        hdr = [
            f"servo_monitor  [{mode}]   {time.strftime('%H:%M:%S')}",
            f"controller state: {self.state}   motors enabled: {self.enabled}   "
            f"battery: {'-' if self.volts is None else f'{self.volts:.1f} V'}",
            f"last msg  /joint_command {self._age(self.t_ctrl)} | /debug/servo_command {self._age(self.t_cmd)}"
            f" | /debug/servo_feedback {self._age(self.t_fb)} | /joint_states {self._age(self.t_meas)}",
        ]
        not_sending = [s["_msg"] for s in self.cmd.values() if s.get("_msg", "").startswith("NOT SENDING")]
        if not_sending:
            hdr.append("!! " + not_sending[0])
        hdr.append("")
        return build_table(self.view, self.ctrl, self.cmd, self.fb, self.meas, hdr)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    ros_args = argv[argv.index("--ros-args"):] if "--ros-args" in argv else []
    own = argv[:argv.index("--ros-args")] if "--ros-args" in argv else argv
    ap = argparse.ArgumentParser(prog="servo_monitor", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--view", choices=["both", "cmd", "fb"], default="both")
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--no-clear", action="store_true")
    ap.add_argument("--rate", type=float, default=4.0)
    a = ap.parse_args(own)

    rclpy.init(args=["servo_monitor"] + ros_args)
    node = ServoMonitor(a.view)
    clear = not a.no_clear and not a.once and sys.stdout.isatty()

    def show():
        text = node.render()
        if clear:
            sys.stdout.write("\033[2J\033[H")   # ANSI: clear screen, cursor to top-left
        print(text + ("\n\n(Ctrl-C to quit)" if not a.once else ""), flush=True)

    try:
        if a.once:
            end = time.monotonic() + 2.0          # collect messages for 2 s, then print once
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            show()
        else:
            node.create_timer(1.0 / max(0.5, a.rate), show)
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
