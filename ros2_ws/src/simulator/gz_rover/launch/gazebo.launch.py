from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_gz_rover = FindPackageShare(package='gz_rover')
    urdf_file = PathJoinSubstitution([pkg_gz_rover, 'urdf', 'rover.urdf'])
    world_path = PathJoinSubstitution([pkg_gz_rover, 'worlds', 'empty.world'])
    
    # Load URDF content
    robot_description = {'robot_description': Command(['cat', urdf_file])}
    
    # Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ]
    )
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
    ])