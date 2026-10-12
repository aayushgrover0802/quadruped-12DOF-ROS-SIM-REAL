"""
gait.py -- ROS 2 node "quad_controller".

This node is a thin ROS wrapper around gait/controller.py.  It is the SAME node
for Isaac Sim and for the real robot -- only where /joint_command ends up differs:
  * Isaac Sim  : the Action Graph subscribes to /joint_command directly.
  * Real robot : hardware/bridge.py subscribes to /joint_command and forwards to the ESP32.

Subscribes
  /cmd_vel        geometry_msgs/Twist   walk command (x fwd, y left, z-rot turn left)
  /quad/mode      std_msgs/String       "sit" or "stand"   (starts in start_mode, default stand)
  /joint_states   sensor_msgs/JointState  (optional) measured joints, used once at startup
Publishes
  /joint_command  sensor_msgs/JointState  12 joint targets in radians, fixed order
  /quad/state     std_msgs/String         STARTUP / SIT / STAND / WALK (for debugging)
"""
import dataclasses
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from QuadPkg.description.robot_description import load_robot_xml, mass_report
from QuadPkg.gait.controller import GaitController, GaitParams
from QuadPkg.kinematics.leg_kinematics import QuadKinematics


class QuadControllerNode(Node):
    def __init__(self):
        super().__init__("quad_controller")

        # ---------- parameters (all overridable from config/quad_params.yaml) ----------
        share = get_package_share_directory("QuadPkg")
        self.declare_parameter("urdf_path", os.path.join(share, "urdf", "QuadURDFcad4_0.urdf"))
        # extra mass on the body (battery, Jetson, lidar...). "" = bare robot.
        self.declare_parameter("payload_file", os.path.join(share, "config", "payload.yaml"))
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("startup_wait", 1.0)
        params = GaitParams()
        for f in dataclasses.fields(GaitParams):
            if f.name == "phase_offsets":
                continue
            self.declare_parameter(f.name, getattr(params, f.name))
            setattr(params, f.name, self.get_parameter(f.name).value)

        urdf = self.get_parameter("urdf_path").value
        payload = self.get_parameter("payload_file").value
        self.kin = QuadKinematics(load_robot_xml(urdf, payload))
        self.ctrl = GaitController(self.kin, params)
        self.get_logger().info(f"Loaded {urdf} + payload '{payload or 'none'}': {mass_report(self.kin)}")
        self.get_logger().info(
            f"stance shifted {params.stance_y_shift * 1000:+.1f} mm to centre feet under COM; "
            f"start pose = {self.ctrl.mode_request.upper()}")

        # ---------- ROS interfaces ----------
        self.cmd_pub = self.create_publisher(JointState, "/joint_command", 10)
        self.state_pub = self.create_publisher(String, "/quad/state", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(String, "/quad/mode", self.on_mode, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)

        self.measured = None
        self.started = False
        self.t_created = self.get_clock().now()
        self.last_cmd_time = None
        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(self.dt, self.tick)
        self._last_state = None

    # ---------- callbacks ----------
    def on_cmd_vel(self, msg):
        self.ctrl.set_command(msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def on_mode(self, msg):
        self.ctrl.set_mode(msg.data)
        self.get_logger().info(f"mode request -> {msg.data}")

    def on_joint_states(self, msg):
        self.measured = dict(zip(msg.name, msg.position))

    # ---------- main loop ----------
    def tick(self):
        now = self.get_clock().now()

        # Wait briefly for /joint_states so we can start from where the robot really is
        if not self.started:
            waited = (now - self.t_created).nanoseconds * 1e-9
            if self.measured is None and waited < self.get_parameter("startup_wait").value:
                return
            self.ctrl.reset(self.measured)
            src = "measured /joint_states" if self.measured else "zero pose (no /joint_states)"
            self.get_logger().info(f"Startup: blending from {src} to {self.ctrl.mode_request.upper()}")
            self.started = True

        # Safety: stop walking if /cmd_vel goes quiet (teleop crashed, Wi-Fi dropped...)
        if self.last_cmd_time is not None:
            if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.get_parameter("cmd_timeout").value:
                self.ctrl.set_command(0.0, 0.0, 0.0)
                self.last_cmd_time = None

        angles = self.ctrl.update(self.dt)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(self.kin.joint_names)
        msg.position = [angles[j] for j in self.kin.joint_names]
        self.cmd_pub.publish(msg)

        if self.ctrl.state != self._last_state:
            self.get_logger().info(f"state -> {self.ctrl.state}")
            self._last_state = self.ctrl.state
        self.state_pub.publish(String(data=self.ctrl.state))
        if self.ctrl.unreachable_count:
            self.get_logger().warn(
                f"{self.ctrl.unreachable_count} foot targets were out of reach (clamped)",
                throttle_duration_sec=2.0)
            self.ctrl.unreachable_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = QuadControllerNode()
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
