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

    # ros2 run usb_cam usb_cam_node_exe --ros-args   -p image_width:=1280   -p image_height:=720   -p framerate:=30.0   -p pixel_format:="mjpeg2rgb"
    cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera",
        namespace="ros381",
        parameters=[
            {"image_width": 1280},
            {"image_height": 720},
            {"framerate": 30.0},
            {"pixel_format": "mjpeg2rgb"}
        ]
    )

    return LaunchDescription(
        [
            mini_mbp_node,
            tactics_node,
            uc_node,
            cam_node,
        ]
    )
