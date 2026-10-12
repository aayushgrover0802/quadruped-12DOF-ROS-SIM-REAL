"""
bridge.py -- ROS 2 node "hardware_bridge" (real robot only; runs on the Jetson / your laptop).

The controller speaks URDF joint angles (radians, 0 = standing). The servos speak
ticks (0..4095, 2048 = middle). This node translates, adds safety, and talks to the
ESP32 over USB serial. ALL the processing is here -- the ESP32 only relays.

    quad_controller --/joint_command--> [hardware_bridge] ==USB==> ESP32 ==bus==> 12 servos
    quad_controller <--/joint_states--- [hardware_bridge] <=USB=== ESP32 <==bus== 12 servos

Subscribes
  /joint_command   sensor_msgs/JointState  targets from the controller (rad)
  /quad/enable     std_msgs/Bool           true = torque on, false = torque off (limp)
Publishes
  /joint_states    sensor_msgs/JointState  measured angles (rad), speeds (rad/s), effort = load (%)
  /quad/battery_voltage  std_msgs/Float32  average servo supply voltage (V)
  /quad/servo_temperature std_msgs/Float32MultiArray  deg C, in joint_order
  /quad/motors_enabled   std_msgs/Bool     current torque state

Debug topics (only for LOOKING at; nothing subscribes to them. Off with debug_topics: false)
  /debug/servo_command   diagnostic_msgs/DiagnosticArray  per servo: what the controller asked,
                         what survived the limits + slew limiter, and the ticks actually sent
  /debug/servo_feedback  diagnostic_msgs/DiagnosticArray  per servo: what came back (ticks, angle,
                         error vs. sent, speed, load, voltage, temperature, current, online/torque)
  /debug/joint_sent      sensor_msgs/JointState  angles actually sent (rad) -> plot against /joint_states
  Easiest way to read them:  ros2 run QuadPkg servo_monitor   (see docs/07_debug_topics.md)

Safety
  * Torque stays OFF until /quad/enable gets `true` (unless auto_enable).
  * When enabling, the ESP32 first sets every goal = current position (no jump), and the
    bridge then glides to the controller's pose at only `enable_velocity` rad/s.
  * Every command is clamped to the URDF joint limits AND the servo tick range.
  * Slew-rate limit `max_velocity` on every joint, always.
  * If /joint_command stops, the last command is held. If the Jetson dies, the ESP32
    watchdog holds the pose (or goes limp, see watchdog_torque_off).
"""
import math
import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool, Float32, Float32MultiArray

from QuadPkg.hardware import protocol as P
from QuadPkg.hardware.esp32_link import Esp32Link
from QuadPkg.hardware.servo_map import DEFAULT_IDS, DEFAULT_ORDER, ServoMap
from QuadPkg.kinematics.leg_kinematics import QuadKinematics


class HardwareBridge(Node):
    def __init__(self):
        super().__init__("hardware_bridge")
        share = get_package_share_directory("QuadPkg")
        decl = self.declare_parameter
        decl("urdf_path", os.path.join(share, "urdf", "QuadURDFcad4_0.urdf"))
        decl("port", "/dev/ttyUSB0")
        decl("baud", 921600)
        decl("feedback_hz", 50)
        decl("watchdog_ms", 500)
        decl("watchdog_torque_off", False)
        decl("joint_order", DEFAULT_ORDER)
        decl("servo_id", DEFAULT_IDS)
        decl("direction", [1.0] * 12)
        decl("offset_ticks", [0] * 12)
        decl("gear_ratio", [1.0] * 12)
        decl("tick_min", 0)
        decl("tick_max", 4095)
        decl("rate_hz", 50.0)
        decl("max_velocity", 8.0)
        decl("enable_velocity", 0.6)
        decl("enable_ramp_time", 4.0)
        decl("servo_speed", 0)
        decl("servo_acc", 0)
        decl("auto_enable", False)
        decl("torque_off_on_exit", False)
        decl("max_temp_c", 65)
        decl("min_voltage", 10.5)
        decl("debug_topics", True)       # publish /debug/servo_command + /debug/servo_feedback
        decl("debug_rate_hz", 10.0)      # how often (10 Hz is easy to read; 50 = every tick)
        decl("debug_error_warn_deg", 5.0)   # |sent - measured| above this -> WARN in feedback
        P = lambda n: self.get_parameter(n).value  # noqa: E731
        self.P = P

        self.map = ServoMap(P("joint_order"), P("servo_id"), P("direction"), P("offset_ticks"),
                            P("gear_ratio"), P("tick_min"), P("tick_max"))
        self.order = self.map.order

        # URDF joint limits (the re-zeroed URDF: 0 = standing)
        kin = QuadKinematics(P("urdf_path"))
        lower, upper = {}, {}
        for leg in kin.legs.values():
            for j, lo, hi in zip(leg.joint_names, leg.lower, leg.upper):
                lower[j], upper[j] = lo, hi
        self.lower = np.array([lower[j] for j in self.order])
        self.upper = np.array([upper[j] for j in self.order])

        # ---------- connect to the ESP32 ----------
        self.link = Esp32Link(P("port"), int(P("baud")), log=self.get_logger().info)
        hello = self.link.hello(timeout=4.0)
        if hello is None:
            raise RuntimeError(f"No answer from the ESP32 on {P('port')} @ {P('baud')} baud. "
                               "Is the firmware flashed (firmware/README.md) and the port right?")
        self.get_logger().info(f"ESP32 firmware v{hello[1]} (protocol v{hello[0]}) on {P('port')}")
        online = self.link.configure(self.map.ids, int(P("feedback_hz")), int(P("watchdog_ms")),
                                     1 if P("watchdog_torque_off") else 0)
        if online is None:
            raise RuntimeError("ESP32 did not acknowledge CONFIG")
        missing = [f"{j}(id {i})" for j, i, ok in zip(self.order, self.map.ids, online) if not ok]
        if missing:
            self.get_logger().error(f"servos NOT answering: {', '.join(missing)} -- check wiring/IDs "
                                    "(ros2 run QuadPkg servo_tool scan). Motors will not be enabled.")
        else:
            self.get_logger().info("all 12 servos answered")
        self.all_online = not missing

        # ---------- state ----------
        self.target = None        # latest /joint_command (rad, joint_order)
        self.sent = None          # last value actually sent (rad)
        self.measured = None      # latest feedback (rad)
        self.last_state_count = 0
        self.enabled = False
        self.enabled_at = None
        # debug bookkeeping (what happened to the command on its way to the servos)
        self.dbg_goal = None          # target after URDF-limit clamp (rad)
        self.dbg_slewed = None        # bool per joint: slew limiter cut the step this tick
        self.dbg_ticks = None         # ticks actually sent
        self.dbg_vmax = 0.0

        # ---------- ROS ----------
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.volt_pub = self.create_publisher(Float32, "/quad/battery_voltage", 10)
        self.temp_pub = self.create_publisher(Float32MultiArray, "/quad/servo_temperature", 10)
        self.en_pub = self.create_publisher(Bool, "/quad/motors_enabled", 10)
        self.create_subscription(JointState, "/joint_command", self.on_command, 10)
        if P("debug_topics"):
            self.dbg_cmd_pub = self.create_publisher(DiagnosticArray, "/debug/servo_command", 10)
            self.dbg_fb_pub = self.create_publisher(DiagnosticArray, "/debug/servo_feedback", 10)
            self.dbg_sent_pub = self.create_publisher(JointState, "/debug/joint_sent", 10)
            self.create_timer(1.0 / max(0.5, float(P("debug_rate_hz"))), self.publish_debug)
        self.create_subscription(Bool, "/quad/enable", self.on_enable, 10)

        self.dt = 1.0 / float(P("rate_hz"))
        self.create_timer(self.dt, self.tick)
        self.create_timer(1.0, self.health)

        if P("auto_enable"):
            self.create_timer(1.0, self._auto_enable_once)
        self.get_logger().info("hardware_bridge ready. Motors are OFF until:  "
                               "ros2 topic pub --once /quad/enable std_msgs/msg/Bool \"{data: true}\"  "
                               "(or press M in teleop)")

    def _auto_enable_once(self):
        if not self.enabled and self.measured is not None:
            self.on_enable(Bool(data=True))

    # ---------------- callbacks ----------------
    def on_command(self, msg):
        lookup = dict(zip(msg.name, msg.position))
        if all(j in lookup for j in self.order):
            self.target = np.array([lookup[j] for j in self.order])

    def on_enable(self, msg):
        want = bool(msg.data)
        if want:
            if self.link.state is not None:          # re-check: a servo may have come back
                self.all_online = all(s.online for s in self.link.state[1])
            if not self.all_online:
                self.get_logger().error("refusing to enable: not all servos are answering")
                return
            if self.measured is None:
                self.get_logger().error("refusing to enable: no feedback from the ESP32 yet")
                return
            if not self.link.torque(True):
                self.get_logger().error("ESP32 did not confirm torque ON")
                return
            self.sent = self.measured.copy()     # start the slew limiter where the legs really are
            self.enabled_at = self.get_clock().now()
            self.enabled = True
            self.get_logger().info("motors ENABLED -- gliding to the commanded pose")
        else:
            self.link.torque(False)
            self.enabled = False
            self.get_logger().warn("motors DISABLED (limp)")
        self.en_pub.publish(Bool(data=self.enabled))

    # ---------------- main loop ----------------
    def tick(self):
        # 1) feedback -> /joint_states (published even while disabled, so the controller
        #    starts its blend from the robot's real pose)
        if self.link.n_state != self.last_state_count and self.link.state is not None:
            self.last_state_count = self.link.n_state
            _, servos = self.link.state
            ok = [s.online for s in servos]
            q = self.map.ticks_to_joint([s.pos for s in servos])
            if self.measured is not None:          # keep the old value for a servo that is offline
                q = np.where(ok, q, self.measured)
            self.measured = q
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self.order)
            js.position = q.tolist()
            js.velocity = self.map.speed_to_joint([s.speed for s in servos]).tolist()
            js.effort = [s.load / 10.0 for s in servos]        # % of max torque
            self.js_pub.publish(js)

        # 2) commands -> ESP32
        if not self.enabled or self.target is None or self.sent is None:
            # nothing to send: a small "I'm alive" so the ESP32 watchdog stays quiet
            self._idle_ticks = getattr(self, "_idle_ticks", 0) + 1
            if self._idle_ticks % 10 == 0:
                self.link.send(P.encode(P.HELLO))
            return
        since = (self.get_clock().now() - self.enabled_at).nanoseconds * 1e-9
        vmax = self.P("enable_velocity") if since < self.P("enable_ramp_time") else self.P("max_velocity")
        goal = np.clip(self.target, self.lower, self.upper)
        want = goal - self.sent
        step = np.clip(want, -vmax * self.dt, vmax * self.dt)
        self.sent = self.sent + step
        ticks = self.map.joint_to_ticks(self.sent)
        self.link.set_targets(ticks.tolist(), int(self.P("servo_speed")), int(self.P("servo_acc")))
        self.dbg_goal, self.dbg_ticks, self.dbg_vmax = goal, ticks, vmax
        self.dbg_slewed = np.abs(want - step) > 1e-9

    def health(self):
        age = time.monotonic() - self.link.state_time if self.link.state_time else math.inf
        if age > 0.5:
            self.get_logger().warn(f"no feedback from the ESP32 for {age:.1f} s", throttle_duration_sec=5.0)
            return
        _, servos = self.link.state
        offline = [j for j, s in zip(self.order, servos) if not s.online]
        if offline:
            self.get_logger().warn(f"servos not answering: {offline}", throttle_duration_sec=5.0)
        volts = [s.voltage for s in servos if s.online]
        temps = [float(s.temp) for s in servos]
        if volts:
            v = sum(volts) / len(volts)
            self.volt_pub.publish(Float32(data=float(v)))
            if v < self.P("min_voltage"):
                self.get_logger().warn(f"LOW BATTERY: {v:.1f} V", throttle_duration_sec=10.0)
        self.temp_pub.publish(Float32MultiArray(data=temps))
        hot = [f"{j} {t:.0f}C" for j, t in zip(self.order, temps) if t > self.P("max_temp_c")]
        if hot:
            self.get_logger().warn(f"HOT servos: {', '.join(hot)} -- let them cool down",
                                   throttle_duration_sec=10.0)
        self.en_pub.publish(Bool(data=self.enabled))

    # ---------------- debug topics (for humans) ----------------
    @staticmethod
    def _kv(**kw):
        return [KeyValue(key=k, value=v) for k, v in kw.items()]

    def publish_debug(self):
        now = self.get_clock().now().to_msg()
        deg = math.degrees
        OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR

        # ---- 1) COMMAND side: controller wish -> limits -> slew -> ticks on the wire
        cmd = DiagnosticArray()
        cmd.header.stamp = now
        sending = self.enabled and self.target is not None and self.sent is not None
        for i, j in enumerate(self.order):
            st = DiagnosticStatus(name=j, hardware_id=f"servo id {self.map.ids[i]}")
            ctrl = "none" if self.target is None else f"{deg(self.target[i]):+.2f}"
            if sending and self.dbg_ticks is not None:
                clamped = abs(self.dbg_goal[i] - self.target[i]) > 1e-6
                slewed = bool(self.dbg_slewed[i])
                st.level = WARN if clamped else OK
                st.message = (f"sent {deg(self.sent[i]):+.2f} deg = {int(self.dbg_ticks[i])} ticks"
                              + ("  [CLAMPED by URDF limit]" if clamped else "")
                              + ("  [slew-limited]" if slewed else ""))
                st.values = self._kv(
                    controller_target_deg=ctrl,
                    after_limits_deg=f"{deg(self.dbg_goal[i]):+.2f}",
                    sent_deg=f"{deg(self.sent[i]):+.2f}",
                    sent_ticks=str(int(self.dbg_ticks[i])),
                    clamped_by_limit=str(clamped),
                    slew_limited=str(slewed),
                    slew_limit_deg_s=f"{deg(self.dbg_vmax):.0f}",
                    limit_low_deg=f"{deg(self.lower[i]):+.1f}",
                    limit_high_deg=f"{deg(self.upper[i]):+.1f}")
            else:
                why = ("motors OFF (press M in teleop)" if not self.enabled else
                       "no /joint_command received (is quad_controller running?)")
                st.level = WARN
                st.message = f"NOT SENDING: {why}"
                st.values = self._kv(controller_target_deg=ctrl, sent_deg="none", sent_ticks="none")
            cmd.status.append(st)
        self.dbg_cmd_pub.publish(cmd)

        if sending:
            js = JointState()
            js.header.stamp = now
            js.name = list(self.order)
            js.position = self.sent.tolist()
            self.dbg_sent_pub.publish(js)

        # ---- 2) FEEDBACK side: what the servos report back
        fb = DiagnosticArray()
        fb.header.stamp = now
        if self.link.state is None:
            for i, j in enumerate(self.order):
                fb.status.append(DiagnosticStatus(level=ERROR, name=j, hardware_id=f"servo id {self.map.ids[i]}",
                                                  message="no STATE frame from the ESP32 yet"))
            self.dbg_fb_pub.publish(fb)
            return
        age = time.monotonic() - self.link.state_time
        _, servos = self.link.state
        q = self.map.ticks_to_joint([s.pos for s in servos])
        spd = self.map.speed_to_joint([s.speed for s in servos])
        warn_err = float(self.P("debug_error_warn_deg"))
        for i, (j, s) in enumerate(zip(self.order, servos)):
            st = DiagnosticStatus(name=j, hardware_id=f"servo id {self.map.ids[i]}")
            err = deg(self.sent[i] - q[i]) if (sending and s.online) else None
            problems = []
            if not s.online:
                problems.append("NOT ANSWERING")
            if self.enabled and s.online and not s.torque:
                problems.append("torque OFF while motors enabled")
            if s.temp > self.P("max_temp_c"):
                problems.append("HOT")
            if s.online and s.voltage < self.P("min_voltage"):
                problems.append("low voltage")
            if err is not None and abs(err) > warn_err:
                problems.append(f"not following (err {err:+.1f} deg)")
            if abs(s.load) >= 900:
                problems.append("load near 100% (stalling?)")
            if age > 0.5:
                problems.append(f"feedback is {age:.1f} s old")
            st.level = ERROR if (not s.online or age > 0.5) else (WARN if problems else OK)
            st.message = (f"{deg(q[i]):+.2f} deg = {s.pos} ticks, "
                          + (f"err {err:+.2f} deg, " if err is not None else "")
                          + f"load {s.load / 10:+.0f}%, {s.temp}C, {s.voltage:.1f}V"
                          + (("  <-- " + "; ".join(problems)) if problems else ""))
            st.values = self._kv(
                online=str(s.online), fresh_this_poll=str(s.fresh), torque_on=str(s.torque),
                ticks=str(s.pos), angle_deg=f"{deg(q[i]):+.2f}",
                error_deg="n/a" if err is None else f"{err:+.2f}",
                speed_deg_s=f"{deg(spd[i]):+.1f}", load_pct=f"{s.load / 10:+.1f}",
                voltage_v=f"{s.voltage:.1f}", temp_c=str(s.temp), current_ma=f"{s.current_ma:.0f}",
                feedback_age_s=f"{age:.2f}")
            fb.status.append(st)
        self.dbg_fb_pub.publish(fb)

    def shutdown(self):
        try:
            if self.P("torque_off_on_exit"):
                self.link.torque(False)
            self.link.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HardwareBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
