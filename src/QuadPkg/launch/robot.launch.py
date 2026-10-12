"""Real robot: run on the Jetson (or on your laptop with the ESP32 plugged into it).

    ros2 launch QuadPkg robot.launch.py                          # bridge + controller
    ros2 launch QuadPkg robot.launch.py port:=/dev/ttyACM0       # other USB port
    ros2 launch QuadPkg robot.launch.py auto_enable:=true        # torque on at start (after calibration!)

Then, in a second terminal (same machine, or another one with the same ROS_DOMAIN_ID):
    ros2 run QuadPkg teleop          # M = motors on, T = stand, G = sit, W/A/S/D/Q/E = walk, K = limp

No micro-ROS agent is needed any more: the ESP32 runs firmware/esp32_servo_bridge
and the hardware_bridge node talks to it directly over USB serial.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = get_package_share_directory("QuadPkg")
    overrides = {}
    port = LaunchConfiguration("port").perform(context)
    if port:
        overrides["port"] = port
    if LaunchConfiguration("auto_enable").perform(context).lower() == "true":
        overrides["auto_enable"] = True
    payload = LaunchConfiguration("payload_file").perform(context)
    controller_params = [os.path.join(share, "config", "quad_params.yaml"), {"use_sim_time": False}]
    if payload:
        controller_params.append({"payload_file": payload})
    return [
        Node(
            package="QuadPkg",
            executable="hardware_bridge",
            name="hardware_bridge",
            output="screen",
            parameters=[os.path.join(share, "config", "hardware.yaml"), overrides],
        ),
        Node(
            package="QuadPkg",
            executable="controller",
            name="quad_controller",
            output="screen",
            parameters=controller_params,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="",
                              description="ESP32 serial port; empty = use config/hardware.yaml"),
        DeclareLaunchArgument("auto_enable", default_value="false",
                              description="switch servo torque on automatically"),
        DeclareLaunchArgument("payload_file", default_value="",
                              description="payload yaml; empty = installed config/payload.yaml"),
        OpaqueFunction(function=_nodes),
    ])
