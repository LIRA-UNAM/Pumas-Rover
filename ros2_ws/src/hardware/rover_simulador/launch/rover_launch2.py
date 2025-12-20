import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('rover')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_rover.urdf')

    # Webots launcher
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', '4_wheeled_robot.wbt')
    )
    # MyRoverDriver node
    my_rover_driver = WebotsController(
        robot_name='robot(1)',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    # Camera Publisher node
    camera_publisher_node = Node(
        package='rover',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen'
    )

    return LaunchDescription([
        webots,
        my_rover_driver,
        camera_publisher_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])