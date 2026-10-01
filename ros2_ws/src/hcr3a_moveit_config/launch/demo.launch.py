"""MoveIt + RViz with mock (no-physics) ros2_control hardware.

    ros2 launch hcr3a_moveit_config demo.launch.py
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from hcr3a_moveit import build_moveit_config  # noqa: E402


def generate_launch_description():
    pkg = get_package_share_directory("hcr3a_moveit_config")
    moveit_config = build_moveit_config("mock")
    controllers_file = os.path.join(pkg, "config", "ros2_controllers.yaml")

    spawners = [
        Node(package="controller_manager", executable="spawner", arguments=[name])
        for name in ("joint_state_broadcaster", "arm_controller", "gripper_controller")
    ]
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit_config.robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, controllers_file],
            output="screen",
        ),
        *spawners,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "move_group.launch.py")),
            launch_arguments={"sim_mode": "mock", "rviz": LaunchConfiguration("rviz")}.items(),
        ),
    ])
