import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  
    mobile_base_node = Node(
        name='mobile_base',
        package='mobile_base',
        executable='mobile_base',
    )
    
    objective_movement_node = Node(
        package="mobile_base",
        executable="objective_movement",
        name='objective_movement',
        #output='screen',
    )

    searcher_node = Node(
        package="search_goal",
        executable="searcher",
        name='searcher',
        #output='screen',
    )

    xarm6_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('xarm_api'),
            'launch',
            'xarm6_driver.launch.py',
        ]),
        launch_arguments={'robot_ip': '172.27.1.212'}
    )

    xarm_controller_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('xarm_controller'),
            'launch',
            'xarm6_control_rviz_display.launch.py',
        ]),
        launch_arguments={
            'robot_ip': '172.27.1.212',
            'add_realsense_d435i': 'true'
        }
    )

    realsense_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('realsense2_camera'),
            'launch',
            'rs_launch.py',
        ]),
        launch_arguments={
            'align_depth': 'true',
            'rgb_camera.color_profile': '640x480x15',
            'depth_module.depth_profile': '640x480x15',
            'initial_reset': 'true',
            'enable_gyro': 'false',
            'enable_accel': 'false'
        }
    )

    follower_node = Node(
        package='xarm_depth_yolo',
        executable='follower',
        name='follower_vel',
        #output='screen',
    )

    yolo_process = ExecuteProcess(
        cmd=['bash', '-c', 'source bin/activate && python yolo_rs_test.py'],
        cwd=os.path.join(os.path.dirname(__file__), '../../../hardware/yolo_realsense/yolo_realsense')
    )
    '''''
    TERMINAL 1 (API)
    $ping 172.27.1.212
    $ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=172.27.1.212

    TERMINAL 2 (BRAZO + RS(URDF)):
    $ros2 launch xarm_controller xarm6_control_rviz_display.launch.py \robot_ip:=172.27.1.212 add_realsense_d435i:=true

    TERMINAL 3 (ACTIVAR RS):
    $ros2 launch realsense2_camera rs_launch.py align_depth:=true

    TERMINAL 4 (YOLO):
    $ros2 launch realsense2_camera rs_launch.py \
    rgb_camera.color_profile:=640x480x15 \
    depth_module.depth_profile:=640x480x15 \
    initial_reset:=true \
    enable_gyro:=false \
    enable_accel:=false

    TERMINAL 5 (YOLO):
    cd Pumas-Rover/ros2_ws/src/hardware/yolo_realsense/yolo_realsense
    source bin/activate
    $python yolo_rs_test.py-> (venv) 

    //para desactivar
    $deactivate 

    Terminal 5 (FOLLOWER)
    $ros2 run xarm_depth_yolo follower_vel
    '''
    
        
    return LaunchDescription([
        mobile_base_node,
        objective_movement_node,
        searcher_node,
        xarm6_driver_launch,
        xarm_controller_launch,
        realsense_launch,
        follower_node,
        yolo_process
    ])
