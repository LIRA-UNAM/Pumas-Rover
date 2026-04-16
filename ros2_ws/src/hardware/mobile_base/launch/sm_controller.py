from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
  
    # Argumento centralizado
    robot_ip = LaunchConfiguration('robot_ip')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.27.1.212',
        description='IP del brazo xArm'
    )


    state_machine_node = Node (
        name='state_machine',
        package='mobile_base',
        executable='state_machine',
        
        
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

    xarm_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('xarm_api'),
            'launch',
            'xarm6_driver.launch.py'
        ]),
        launch_arguments={
            'robot_ip': robot_ip
        }.items()
    )


    xarm_rviz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('xarm_controller'),
            'launch',
            'xarm6_control_rviz_display.launch.py'
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'add_realsense_d435i': 'true'
        }.items()
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
    
    search_node = Node(
    name='search',
    package='xarm_depth_yolo',
    executable='search',
)

    
    mobile_base_node = Node(
        name='mobile_base',
        package='mobile_base',
        executable='mobile_base',
        arguments=['--ros-args', '--log-level', 'error']
    )

    
        
    return LaunchDescription([
        robot_ip_arg,

        #state_machine_node,
        objective_movement_node,
        rock_follower_node,
        xarm_driver_launch,
        xarm_rviz_launch,
        realsense_launch,
        #search_node,
        state_machine_node,
        #delayed_sm,
        mobile_base_node,
        

        
    
    ])
