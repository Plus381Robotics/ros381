import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    mini_mbp_node = Node(
        package="ros381_base",
        executable="miniMBP",
        name="mini_mbp",
        namespace="ros381",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )

    tactics_node = Node(
        package="ros381_tactics",
        executable="global",
        name="tactic_global",
        namespace="ros381",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )

    uc_node = Node(
        package="ros381_hardware",
        executable="uc",
        name="uc",
        namespace="ros381",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )

    return LaunchDescription(
        [
            mini_mbp_node,
            tactics_node,
            uc_node,
        ]
    )
