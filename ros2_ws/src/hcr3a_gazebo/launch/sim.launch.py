"""Gazebo Fortress + ros2_control + MoveIt + RViz for the HCR-3A pick-and-place scene.

    ros2 launch hcr3a_gazebo sim.launch.py              # Gazebo GUI + RViz
    ros2 launch hcr3a_gazebo sim.launch.py headless:=true rviz:=false
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory("hcr3a_moveit_config"), "launch"))
from hcr3a_moveit import build_moveit_config  # noqa: E402


def launch_setup(context):
    moveit_pkg = get_package_share_directory("hcr3a_moveit_config")
    world = os.path.join(get_package_share_directory("hcr3a_gazebo"), "worlds", "pick_place.sdf")
    controllers_file = os.path.join(moveit_pkg, "config", "ros2_controllers.yaml")
    moveit_config = build_moveit_config("gazebo", controllers_file)

    headless = LaunchConfiguration("headless").perform(context) == "true"
    gz_args = f"-r {'-s ' if headless else ''}{world}"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "hcr3a", "-topic", "robot_description"],
        output="screen",
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            parameters=[{"use_sim_time": True}],
        )

    jsb = spawner("joint_state_broadcaster")
    arm = spawner("arm_controller")
    gripper = spawner("gripper_controller")

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_pkg, "launch", "move_group.launch.py")),
        launch_arguments={
            "sim_mode": "gazebo",
            "use_sim_time": "true",
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    # Point the Gazebo GUI camera at the work cell (default view is far away).
    gui_camera = TimerAction(period=8.0, actions=[ExecuteProcess(cmd=[
        "ign", "service", "-s", "/gui/move_to/pose",
        "--reqtype", "ignition.msgs.GUICamera", "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000", "--req",
        "pose: {position: {x: 1.25, y: -0.95, z: 0.75}, "
        "orientation: {w: 0.3269, x: -0.1965, y: 0.0697, z: 0.9218}}",
    ])])

    actions = [
        gazebo,
        rsp,
        clock_bridge,
        spawn,
        # controllers come up once the robot (and its ros2_control plugin) exists in Gazebo
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, gripper])),
        move_group,
    ]
    if not headless:
        actions.append(gui_camera)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="run gz server only"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
