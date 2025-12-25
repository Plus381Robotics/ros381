import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    hardware_pkg = get_package_share_directory("ros381_hardware")
    base_pkg = get_package_share_directory("ros381_base")
    tactics_pkg = get_package_share_directory("ros381_tactics")

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
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="csi_camera",
        namespace="ros381",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/hardware.params.yaml"
        ],
    )

    return LaunchDescription(
        [
            control_loop_node,
            odometry_node,
            tactics_node,
            uc_node,
            ax12a_setup,
            ax12a_single,
            ax12a_bulk,
            ax12a_hybrid,
            csi_camera_node,
            # launch.actions.RegisterEventHandler(
            #     event_handler=launch.event_handlers.OnProcessExit(
            #         target_action=hardware,
            #         on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            #     )
            # ),
        ]
    )
