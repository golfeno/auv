#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_auv_bringup')
    pkg_desc    = get_package_share_directory('my_auv_description')
    pkg_gz_sim  = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_bringup, 'worlds', 'static_world.sdf')
    model_path = os.path.join(pkg_desc,    'models', 'submarine.sdf')

    # Gazebo (пробел гарантирован через f-string)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Спавн
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'submarine', '-file', model_path, '-x', '0', '-y', '0', '-z', '-3'],
        output='screen'
    )

    # Мост: ROS Float64 <-> Gazebo Double (строго 64-битные числа!)
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/model/submarine/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/submarine/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/submarine/joint/left_propeller_joint/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/right_propeller_joint/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/vertical_rudder/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_left/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_right/cmd_position@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/submarine/link/body/wrench@geometry_msgs/msg/WrenchStamped@gz.msgs.Wrench',
        ],
        parameters=[
            {'qos_overrides./model/submarine.odometry.publisher.reliability': 'reliable'},
            {'qos_overrides./model/submarine.imu.publisher.reliability': 'reliable'},
        ],
        output='screen'
    )

    return LaunchDescription([gz_sim, spawn, bridge])
