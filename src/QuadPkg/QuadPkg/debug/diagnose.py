"""
diagnose.py -- `ros2 run QuadPkg diagnose`

Prints a health report every 2 seconds while the sim (or robot) is running:
  * how many nodes publish /joint_command  (should be exactly 1)
  * whether /clock is ticking and how fast sim time runs vs real time
  * controller state (/quad/state)
  * per-joint TRACKING ERROR = commanded angle - measured angle
      small (< 0.05 rad)  -> motors follow fine; any drifting is a contact/friction issue
      large (> 0.15 rad)  -> drives too weak / torque-limited  -> gains problem
"""
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class Diagnose(Node):
    def __init__(self):
        super().__init__("quad_diagnose")
        self.cmd, self.meas, self.state = {}, {}, "?"
        self.clock_first = None   # (sim_t, wall_t)
        self.clock_last = None
        self.n_cmd = 0
        self.worst = {}           # joint -> worst |error| in this window
        self.create_subscription(JointState, "/joint_command", self.on_cmd, 10)
        self.create_subscription(JointState, "/joint_states", self.on_meas, 10)
        self.create_subscription(String, "/quad/state", lambda m: setattr(self, "state", m.data), 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_timer(2.0, self.report)

    def on_cmd(self, msg):
        self.cmd = dict(zip(msg.name, msg.position))
        self.n_cmd += 1

    def on_meas(self, msg):
        self.meas = dict(zip(msg.name, msg.position))
        for j, c in self.cmd.items():
            if j in self.meas:
                e = abs(c - self.meas[j])
                self.worst[j] = max(self.worst.get(j, 0.0), e)

    def on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        now = time.time()
        if self.clock_first is None:
            self.clock_first = (t, now)
        self.clock_last = (t, now)

    def report(self):
        pubs = self.count_publishers("/joint_command")
        print("\n================ quad diagnose ================")
        print(f"/joint_command publishers : {pubs}" + ("   <-- PROBLEM: should be 1" if pubs != 1 else "   ok"))
        print(f"/joint_command msgs (2 s) : {self.n_cmd}  (~100 expected at 50 Hz)")
        if self.clock_last and self.clock_first and self.clock_last[1] > self.clock_first[1]:
            ratio = (self.clock_last[0] - self.clock_first[0]) / (self.clock_last[1] - self.clock_first[1])
            print(f"/clock                    : ticking, sim runs at {ratio:.2f}x real time")
        else:
            print("/clock                    : NOT received  <-- controller won't run with use_sim_time")
        print(f"controller state          : {self.state}")
        if not self.meas:
            print("/joint_states             : NOT received (Isaac joint-state publisher missing?)")
        else:
            print("tracking error |cmd - measured| (worst in last 2 s), radians:")
            for j in sorted(self.worst):
                flag = "  <-- LARGE" if self.worst[j] > 0.15 else ""
                print(f"   {j:12s} {self.worst[j]:.3f}{flag}")
        self.n_cmd, self.worst = 0, {}


def main(args=None):
    rclpy.init(args=args)
    node = Diagnose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
