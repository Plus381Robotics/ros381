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
    description_pkg = get_package_share_directory("ros381_description")
    blue_description_path = os.path.join(description_pkg, "urdf", "blue.urdf")
    yellow_description_path = os.path.join(description_pkg, "urdf", "yellow.urdf")

    webots = WebotsLauncher(
        world=os.path.join(description_pkg, "worlds", "table2.wbt"),
        ros2_supervisor=True,
    )

    webots_node_1 = WebotsController(
        robot_name="blue",
        namespace="blue",
        parameters=[
            {"robot_description": blue_description_path},
            {"use_sim_time": True},
        ],
    )

    webots_node_2 = WebotsController(
        robot_name="yellow",
        namespace="yellow",
        parameters=[
            {"robot_description": yellow_description_path},
            {"use_sim_time": True},
        ],
    )

    control_loop_node_1 = Node(
        package="ros381_base",
        executable="control_loop",
        name="control_loop",
        namespace="blue",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
        ],
    )

    odometry_node_1 = Node(
        package="ros381_base",
        executable="odometry",
        name="odometry",
        namespace="blue",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
        ],
    )

    control_loop_node_2 = Node(
        package="ros381_base",
        executable="control_loop",
        name="control_loop",
        namespace="yellow",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
        ],
    )

    odometry_node_2 = Node(
        package="ros381_base",
        executable="odometry",
        name="odometry",
        namespace="yellow",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
        ],
    )

    tactics_node_1 = Node(
        package="ros381_tactics",
        executable="global",
        name="tactic_global",
        namespace="blue",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
        ],
    )

    tactics_node_2 = Node(
        package="ros381_tactics",
        executable="global",
        name="tactic_global",
        namespace="yellow",
        output="screen",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
        ],
    )

    chinch_trigger_node = Node(
        package="ros381_webots",
        executable="global_chinch",
        name="global_chinch",
        output="screen",
    )

    uc_node_1 = Node(
        package="ros381_hardware",
        executable="uc",
        name="uc",
        namespace="blue",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
		],
	)

    uc_node_2 = Node(
        package="ros381_hardware",
        executable="uc",
        name="uc",
        namespace="yellow",
        parameters=[
            "/home/hostuser/ros381/src/ros381_bringup/config/webots.params.yaml"
		],
	)
    
    visualization_node = Node(
        package="ros381_visualization",
        executable="lidar_plot",
        name="lidar_plot",
        parameters=[
            {"robot_name": "yellow"}
		],
	)
    
    gridmap_node_1 = Node(
        package="ros381_base",
        executable="gridmap",
        name="gridmap",
        namespace="blue",
    )

    gridmap_node_2 = Node(
        package="ros381_base",
        executable="gridmap",
        name="gridmap",
        namespace="yellow",
    )

    return LaunchDescription(
        [
            webots,
            webots._supervisor,
            webots_node_1,
            webots_node_2,
            control_loop_node_1,
            odometry_node_1,
            control_loop_node_2,
            odometry_node_2,
            tactics_node_1,
            tactics_node_2,
            chinch_trigger_node,
            uc_node_1,
            uc_node_2,
            visualization_node,
            # gridmap_node_1,
            gridmap_node_2,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
