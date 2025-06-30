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
    description_pkg = get_package_share_directory("ros381_description")
    robot_description_path = os.path.join(description_pkg, "urdf", "ros381.urdf")

    webots = WebotsLauncher(world=os.path.join(description_pkg, "worlds", "table.wbt"))

    webots_node = WebotsController(
        robot_name="ros381",
        parameters=[
            {"robot_description": robot_description_path},
            {"use_sim_time": True}
        ],
    )

    control_loop_node = Node(
        package="ros381_base",
        executable="control_loop",
        name="control_loop",
        output="screen",
        # Add parameters if needed:
        # parameters=[os.path.join(control_pkg, 'config', 'control_params.yaml')]
    )

    return LaunchDescription(
        [
            webots,
            webots_node,
            control_loop_node,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
