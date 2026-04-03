import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    control_loop_node = Node(
        package="ros381_base",
        executable="control_loop",
        name="control_loop",
        namespace="ros381",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )
    
    obstacle_node = Node(
        package="ros381_base",
        executable="obstacle",
        name="obstacle",
        namespace="ros381",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )
    
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

    odometry_node = Node(
        package="ros381_base",
        executable="odometry",
        name="odometry",
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

    ax12a_setup = Node(
        package="dynamixel_sdk_examples",
        executable="ax12a_setup",
        name="ax12a_setup",
        namespace="ros381",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )

    ax12a_single = Node(
        package="dynamixel_sdk_examples",
        executable="ax12a_single",
        name="ax12a_single",
        namespace="ros381",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )

    ax12a_bulk = Node(
        package="dynamixel_sdk_examples",
        executable="ax12a_bulk",
        name="ax12a_bulk",
        namespace="ros381",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )

    ax12a_hybrid = Node(
        package="dynamixel_sdk_examples",
        executable="ax12a_single_hybrid",
        name="ax12a_hybrid",
        namespace="ros381",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )

    csi_camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="csi_camera",
        namespace="ros381",
        parameters=[
            {
                "camera_name": "front_camera",
                "video_device": "/dev/video10",
                "image_width": 640,
                "image_height": 480,
                "frame_rate": 30,
            }
        ],
        remappings=[
            ("csi_camera/image_raw", "image_raw_front"),
        ],
    )

    usb_camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="usb_camera",
        namespace="ros381",
        parameters=[
            {
                "camera_name": "back_camera",
                "video_device": "/dev/video0",
                "image_width": 640,
                "image_height": 480,
                "frame_rate": 30,
            }
        ],
        remappings=[
            ("usb_camera/image_raw", "image_raw_back"),
        ],
    ) 

    aruco_detection_front = Node(
        package="ros381_vision",
        executable="aruco_detection",
        name="aruco_detection_front",
        namespace="ros381",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )

    aruco_detection_back = Node(
        package="ros381_vision",
        executable="aruco_detection",
        name="aruco_detection_back",
        namespace="ros381",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rplidar_ros"),
                "launch",
                "rplidar_s2_launch.py"
            )
        ),
        launch_arguments={
            "serial_port": "/dev/rplidar",
        }.items(),
    )

    return LaunchDescription(
        [
            # control_loop_node,
            # odometry_node,
            obstacle_node,
            mini_mbp_node,
            tactics_node,
            uc_node,
            ax12a_setup,
            ax12a_single,
            ax12a_bulk,
            ax12a_hybrid,
            # csi_camera_node,
            # usb_camera_node,
            aruco_detection_front,
            aruco_detection_back,
            rplidar_launch,
        ]
    )