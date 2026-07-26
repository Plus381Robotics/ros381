import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    config_dir = "/home/hostuser/ros381/src/ros381_bringup/config"
    calibration_file = os.path.join(config_dir, "ov9281_3mm.yaml")

    mini_mbp_node = Node(
        package="ros381_base",
        executable="miniMBP",
        name="mini_mbp",
        namespace="ros381",
        output="screen",
        parameters=[os.path.join(config_dir, "hardware.params.yaml")],
    )

    tactics_node = Node(
        package="ros381_tactics",
        executable="global",
        name="tactic_global",
        namespace="ros381",
        output="screen",
        parameters=[os.path.join(config_dir, "hardware.params.yaml")],
    )

    uc_node = Node(
        package="ros381_hardware",
        executable="uc",
        name="uc",
        namespace="ros381",
        parameters=[os.path.join(config_dir, "hardware.params.yaml")],
    )

    cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera",
        namespace="ros381",
        parameters=[
            {"camera_name": "ov9281_3mm"},
            {"image_width": 1280},
            {"image_height": 720},
            {"framerate": 120.0},
            {"pixel_format": "mjpeg2rgb"},
            {"brightness": 0},
            {"contrast": 32},
            {"saturation": 0}, # irrelevant for monochrome OV9281 
            {"sharpness": 0},
            {"gain": 64}, # low as possible
            {"auto_white_balance": False},
            {"white_balance": 4000}, # ignored on mono sensor
            # mora rucno da se menja svaki put, ne prihvata parametre
            # v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1
            # v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=15
            # v4l2-ctl -d /dev/video0 --set-ctrl=gamma=200
            # {"auto_exposure": 1},
            # {"exposure": 15}, # tune manually 
            # {"exposure_time": 15},
            {"gamma": 200},
            # {"exposure_time_absolute": 15},
            {"autofocus": False},
            {"focus": -1},
            {"camera_info_url": f"file://{calibration_file}"},
        ],
    )

    return LaunchDescription(
        [
            mini_mbp_node,
            tactics_node,
            uc_node,
            cam_node,
        ]
    )
