from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adaptive_sample',
            executable='orchestrator',
            name='orchestrator',
            parameters=[
                {"num_robots":1}
            ]
        ),
        Node(
            package='adaptive_sample',
            executable='simple_robot',
            name='robot0',
            namespace='robot0',
        ),
        Node(
            package='adaptive_sample',
            executable='visualize.py',
            name='vis',
            parameters=[
                {"num_robots":1}
            ]
        )
    ])