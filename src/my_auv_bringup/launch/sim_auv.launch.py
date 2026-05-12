from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    urdf_file = PathJoinSubstitution([
        FindPackageShare('my_auv_description'),
        'urdf',
        'my_auv.xacro'
    ])

    spawn_auv = Node(
   	 package='ros_gz_sim',
    	executable='create',
    	arguments=[
        	'-world', 'empty',                     # Название мира Gazebo
        	'-file', urdf_file,                    # Путь к файлу вашей модели (URDF или SDF)
        	'-name', 'my_auv',                     # Имя модели в симуляции
        	'-x', '0.0', '-y', '0.0', '-z', '0.0', # Начальные координаты X, Y, Z
        	'-R', '0.0', '-P', '0.0', '-Y', '0.0'  # Начальная ориентация в радианах (Roll, Pitch, Yaw)
    	],
    	output='screen'
    )

    bridge_config = PathJoinSubstitution([
        FindPackageShare('my_auv_bringup'),
        'config',
        'bridge_config.yaml'
    ])
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--config-file', bridge_config],
        output='screen'
    )

    controller = Node(
        package='my_auv_control',
        executable='auv_controller',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([gazebo, spawn_auv, bridge, controller])
