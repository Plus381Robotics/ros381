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

    return LaunchDescription(
        [
            control_loop_node,
            odometry_node,
            tactics_node,
            uc_node,
            # launch.actions.RegisterEventHandler(
            #     event_handler=launch.event_handlers.OnProcessExit(
            #         target_action=hardware,
            #         on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            #     )
            # ),
        ]
    )
