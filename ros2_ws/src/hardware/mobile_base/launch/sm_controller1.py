from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
  


    state_machine_node1 = Node (
        name='state_machine1',
        package='mobile_base',
        executable='state_machine1',
        
        
    )

    objective_movement_node = Node(
        name='objective_movement',
        package='mobile_base',
        executable='objective_movement',
        arguments=['--ros-args', '--log-level', 'error']
    )

    rock_follower_node = Node(
        name='rock_follower',
        package='mobile_base',
        executable='rock_follower',
        arguments=['--ros-args', '--log-level', 'error']
    )
    goal_follower_node = Node(
        name='goal_follower',
        package='mobile_base',
        executable='goal_follower',
        arguments=['--ros-args', '--log-level', 'error']
    )

    realsense_launch = IncludeLaunchDescription(
    PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py'
    ]),
    launch_arguments={
        'rgb_camera.color_profile': '640x480x15',
        'depth_module.depth_profile': '640x480x15',
        'initial_reset': 'true',
        'enable_gyro': 'false',
        'enable_accel': 'false',
        'align_depth.enable': 'true'
    }.items()
)
    

    
    mobile_base_node = Node(
        name='mobile_base',
        package='mobile_base',
        executable='mobile_base',
        arguments=['--ros-args', '--log-level', 'error']
    )

    serial_write_node = Node(
        name='serial_writer',
        package='Rover_Emisor',
        executable='rover_serial_writer',
        arguments=['--ros-args', '--log-level', 'error']
        
    )
    
        
    return LaunchDescription([

        goal_follower_node,
        serial_write_node,
        state_machine_node1,
        objective_movement_node,
        rock_follower_node,
        realsense_launch,
        mobile_base_node
        

        
    
    ])
