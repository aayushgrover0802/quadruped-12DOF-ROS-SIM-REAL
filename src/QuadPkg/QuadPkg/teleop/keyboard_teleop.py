"""
keyboard_teleop.py -- drive the quadruped from a terminal.

Run it in its OWN terminal (it needs the keyboard):
    ros2 run QuadPkg teleop

It only publishes /cmd_vel and /quad/mode, so it works the same whether the
controller is running on this laptop (simulation) or on the Jetson (real robot),
as long as both machines share the same ROS_DOMAIN_ID on the same network.
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

HELP = """
---------------- QUADRUPED TELEOP ----------------
  T : stand up            G : sit down
  W / X : faster forward / backward
  A / D : strafe left / right
  Q / E : turn left / right
  SPACE : stop (zero velocity)
  M : motors ON  (real robot)  K : motors OFF / limp (real robot)
  CTRL-C: quit
--------------------------------------------------
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameter("lin_step", 0.02)   # m/s added per key press
        self.declare_parameter("ang_step", 0.25)   # rad/s added per key press
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "/quad/mode", 10)
        self.enable_pub = self.create_publisher(Bool, "/quad/enable", 10)   # real robot only
        self.vx = self.vy = self.wz = 0.0
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print(HELP)
        # Re-publish at 10 Hz so the controller's cmd_timeout safety never trips while we hold a speed
        self.create_timer(0.1, self.tick)

    def read_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return None

    def tick(self):
        lin = self.get_parameter("lin_step").value
        ang = self.get_parameter("ang_step").value
        key = self.read_key()
        if key == "t":
            self.mode_pub.publish(String(data="stand"))
            print("-> stand")
        elif key == "g":
            self.vx = self.vy = self.wz = 0.0
            self.mode_pub.publish(String(data="sit"))
            print("-> sit")
        elif key == "w":
            self.vx += lin
        elif key == "x":
            self.vx -= lin
        elif key == "a":
            self.vy += lin
        elif key == "d":
            self.vy -= lin
        elif key == "q":
            self.wz += ang
        elif key == "e":
            self.wz -= ang
        elif key == " ":
            self.vx = self.vy = self.wz = 0.0
        elif key == "m":
            self.enable_pub.publish(Bool(data=True))
            print("-> motors ON (hardware_bridge glides to the commanded pose)")
        elif key == "k":
            self.vx = self.vy = self.wz = 0.0
            self.enable_pub.publish(Bool(data=False))
            print("-> motors OFF (robot goes limp -- support it!)")
        if key in ("w", "x", "a", "d", "q", "e", " "):
            print(f"cmd_vel  vx={self.vx:+.2f} m/s  vy={self.vy:+.2f} m/s  wz={self.wz:+.2f} rad/s")

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = self.vx, self.vy, self.wz
        self.cmd_pub.publish(msg)

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # send a final stop
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
