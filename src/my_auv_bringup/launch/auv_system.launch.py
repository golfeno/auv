from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    world_file = PathJoinSubstitution([
        FindPackageShare('my_auv_bringup'),
        'worlds',
        'buoyancy_only_world.sdf'
    ])

    gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
             f'gz_args:="-r {world_file}"'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/submarine/joint/left_propeller_joint/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/submarine/joint/right_propeller_joint/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/submarine/joint/vertical_rudder/cmd_position@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_left/cmd_position@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/submarine/joint/horizontal_rudder_right/cmd_position@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        output='screen'
    )

    spawn_model = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                     '-world', 'buoyancy_only_world',
                     '-file', '/home/golfe/auv_ws/src/my_auv_description/models/submarine/model.sdf',
                     '-name', 'submarine',
                     '-x', '0', '-y', '0', '-z', '0.5'],
                output='screen'
            )
        ]
    )

    control_node = Node(
        package='my_auv_control',
        executable='dual_turn_control',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        gazebo,
        bridge,
        spawn_model,
        control_node
    ])
