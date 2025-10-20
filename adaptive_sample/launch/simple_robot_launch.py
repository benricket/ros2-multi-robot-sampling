from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adaptive_sample',
            executable='orchestrator',
            name='orchestrator'
        ),
        Node(
            package='adaptive_sample',
            executable='simple_robot',
            parameters=[{'id':1}],
            name='robot1'
        ),
        Node(
            package='adaptive_sample',
            executable='simple_robot',
            parameters=[{'id':2}],
            name='robot2'
        ),
    ])