from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Мост для одометрии (Gazebo -> ROS)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/submarine/odometry@nav_msgs/msg/Odometry]gz.msgs.Odometry'],
            output='screen'
        ),
        # Ваш контроллер (если хотите запускать всё вместе)
        Node(
            package='my_auv_control',
            executable='dual_turn_control',
            output='screen'
        ),
        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/golfe/auv_ws/src/my_auv_bringup/config/auv.rviz']
        ),
    ])
