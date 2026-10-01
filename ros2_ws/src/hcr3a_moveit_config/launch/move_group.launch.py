"""move_group + optional RViz. Used by the Gazebo bringup (and usable with real hardware)."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from hcr3a_moveit import build_moveit_config  # noqa: E402


def launch_setup(context):
    sim_mode = LaunchConfiguration("sim_mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    controllers_file = os.path.join(
        get_package_share_directory("hcr3a_moveit_config"), "config", "ros2_controllers.yaml"
    )
    moveit_config = build_moveit_config(sim_mode, controllers_file)
    rviz_config = os.path.join(
        get_package_share_directory("hcr3a_moveit_config"), "config", "moveit.rviz"
    )
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim_mode", default_value="mock"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
