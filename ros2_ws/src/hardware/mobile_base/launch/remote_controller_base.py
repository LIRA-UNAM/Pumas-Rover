from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  

    remote_control_node = Node (
        name='remote_control',
        package='mobile_base',
        executable='remote_control',
        arguments=['--ros-args', '--log-level', 'error']
    )
    
    mobile_base_node = Node(
        name='mobile_base',
        package='mobile_base',
        executable='mobile_base',
        arguments=['--ros-args', '--log-level', 'error']
    )
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name='joy',
        #output='screen',
    )

    serial_write_node = Node(
        name='serial_writer',
        package='Rover_Emisor',
        executable='rover_serial_writer',
        #arguments=['--ros-args', '--log-level', 'error']
        
    )
    
        
    return LaunchDescription([
        serial_write_node,
        mobile_base_node,
        remote_control_node,
        joy_node
    
    ])
