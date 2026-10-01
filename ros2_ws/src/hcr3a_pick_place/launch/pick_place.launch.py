"""Run the pick-and-place node against a running move_group.

    ros2 launch hcr3a_pick_place pick_place.launch.py              # Gazebo (sim time)
    ros2 launch hcr3a_pick_place pick_place.launch.py use_sim_time:=false   # mock demo
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory("hcr3a_moveit_config"), "launch"))
from hcr3a_moveit import build_moveit_config  # noqa: E402


def launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    moveit_config = build_moveit_config("gazebo" if use_sim_time else "mock")
    return [
        Node(
            package="hcr3a_pick_place",
            executable="pick_place_node",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {"use_sim_time": use_sim_time},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
