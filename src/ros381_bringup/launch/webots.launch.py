import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node


def generate_launch_description():
    webots_pkg = get_package_share_directory("ros381_webots")
    control_pkg = get_package_share_directory("ros381_control")
    description_pkg = get_package_share_directory("ros381_description")

    robot_description_path = os.path.join(description_pkg, "urdf", "ros381.urdf")
    webots = WebotsLauncher(world=os.path.join(webots_pkg, "worlds", "my_world.wbt"))

    webots_node = WebotsController(
        robot_name="omni-1",
        parameters=[
            {"robot_description": robot_description_path},
        ],
    )

    control_loop_node = Node(
        package="ros381_control",
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
