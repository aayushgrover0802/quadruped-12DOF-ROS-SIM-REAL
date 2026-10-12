"""Simulation: run on the laptop next to Isaac Sim.

    ros2 launch QuadPkg sim.launch.py
then, in a second terminal:
    ros2 run QuadPkg teleop

The controller includes config/payload.yaml (battery, Jetson, ...) in its centre-of-mass
maths. For Isaac to feel the same weight, import the URDF made by
    ros2 run QuadPkg export_urdf
(see README, "Isaac Sim after this update").
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("QuadPkg")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Follow Isaac Sim's /clock (needs the clock publisher in the Action Graph)"),
        Node(
            package="QuadPkg",
            executable="controller",
            name="quad_controller",
            output="screen",
            parameters=[os.path.join(share, "config", "quad_params.yaml"),
                        {"use_sim_time": use_sim_time}],
        ),
    ])
