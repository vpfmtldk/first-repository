"""Shared MoveIt configuration for the HCR-3A launch files."""
import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config(sim_mode="mock", controllers_file=""):
    xacro_file = os.path.join(
        get_package_share_directory("hcr3a_description"), "urdf", "hcr3a.urdf.xacro"
    )
    return (
        MoveItConfigsBuilder("hcr3a", package_name="hcr3a_moveit_config")
        .robot_description(
            file_path=xacro_file,
            mappings={"sim_mode": sim_mode, "controllers_file": controllers_file},
        )
        .robot_description_semantic(file_path="config/hcr3a.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
