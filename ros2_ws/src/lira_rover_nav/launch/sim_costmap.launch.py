import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('lira_rover_nav')
    params_file = os.path.join(pkg_dir, 'config', 'lira_costmap_params.yaml')

    # 1. Transformacion estatica: map -> odom
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'odom']
    )

    # 2. Transformacion estatica: base_link -> laser_frame
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['--x', '0.28', '--y', '0.0', '--z', '0.22',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']
    )

    # 3. Nodo Simulador del Rover (Odometria + TF odom->base_link + LiDAR /scan)
    sim_rover_node = Node(
        package='lira_rover_nav',
        executable='sim_rover_node',
        name='lira_sim_rover_node',
        output='screen'
    )

    # 4. Servidor del Mapa de Costos Local
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        namespace='local_costmap',
        output='screen',
        parameters=[params_file]
    )

    # 5. Servidor del Mapa de Costos Global
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        namespace='global_costmap',
        output='screen',
        parameters=[params_file]
    )

    # 6. Gestor de Ciclo de Vida (bond_timeout: 0.0 evita bloqueos por namespace)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmaps',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                '/local_costmap/local_costmap',
                '/global_costmap/global_costmap'
            ]
        }]
    )

    return LaunchDescription([
        tf_map_odom,
        tf_base_laser,
        sim_rover_node,
        local_costmap_node,
        global_costmap_node,
        lifecycle_manager_node
    ])
