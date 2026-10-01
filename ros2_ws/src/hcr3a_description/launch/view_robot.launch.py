"""Show the HCR-3A URDF in RViz with joint sliders."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg = get_package_share_directory("hcr3a_description")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(pkg, "urdf", "hcr3a.urdf.xacro")]), value_type=str
    )
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(pkg, "rviz", "view_robot.rviz")]),
    ])
