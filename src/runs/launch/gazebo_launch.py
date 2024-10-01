from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        Node(
            package='runs',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', '$(find runs)/urdf/robot.urdf'],
            output='screen')
    ])

