#!/usr/bin/env python3

import os
import yaml
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("bumblebee_MobileManipulator_ver30_description"),
            "urdf",
            "bumblebee_MobileManipulator_ver4.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Semantic config
    robot_description_semantic_path = os.path.join(
        get_package_share_directory("bumblebee_world_moveit_config"),
        "config",
        "bumblebee_MobileManipulator_ver4.srdf",
    )
    with open(robot_description_semantic_path, "r") as file:
        robot_description_semantic_config = file.read()

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Kinematics config
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("bumblebee_world_moveit_config"),
        "config",
        "kinematics.yaml",
    )
    with open(kinematics_yaml_path, "r") as file:
        kinematics_yaml = yaml.safe_load(file)


    moveit_pose_node = Node(
        package="bumblebee_manipulator",
        executable="moveit_pose_subscriber_node",
        name="moveit_pose_subscriber_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml
        ]
    )

    ld = LaunchDescription()
    ld.add_action(moveit_pose_node)

    return ld
