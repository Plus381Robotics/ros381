import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node


def generate_launch_description():
    webots_pkg = get_package_share_directory("ros381_webots")
    base_pkg = get_package_share_directory("ros381_base")
    tactics_pkg = get_package_share_directory("ros381_tactics")
    visualization_pkg = get_package_share_directory("ros381_visualization")
    description_pkg = get_package_share_directory("ros381_description")
    robot_description_path = os.path.join(description_pkg, "urdf", "ros381.urdf")

    webots = WebotsLauncher(
        world=os.path.join(description_pkg, "worlds", "table.wbt"), ros2_supervisor=True
    )

    webots_node = WebotsController(
        robot_name="ros381",
        parameters=[
            {"robot_description": robot_description_path},
            {"use_sim_time": True},
        ],
    )

    control_loop_node = Node(
        package="ros381_base",
        executable="control_loop",
        name="control_loop",
        namespace="ros381",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
        ],
    )

    odometry_node = Node(
        package="ros381_base",
        executable="odometry",
        name="odometry",
        namespace="ros381",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
        ],
    )

    tactics_node = Node(
        package="ros381_tactics",
        executable="global",
        name="tactic_global",
        namespace="ros381",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
        ],
    )

    uc_node = Node(
        package="ros381_hardware",
        executable="uc",
        name="uc",
        namespace="ros381",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
		],
	)
    
    visualization_node = Node(
        package="ros381_visualization",
        executable="lidar_plot",
        name="lidar_plot",
        parameters=[
            ('robot_name', 'ros381')
		],
	)

    return LaunchDescription(
        [
            webots,
            webots._supervisor,
            webots_node,
            control_loop_node,
            odometry_node,
            tactics_node,
            uc_node,
            visualization_node,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
