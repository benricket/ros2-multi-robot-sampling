from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adaptive_sample',
            executable='orchestrator',
            name='orchestrator',
            remappings=[
                ('/waypt_in','/robot999/waypt_in')
            ]
        ),
        Node(
            package='adaptive_sample',
            executable='gp_test_pub',
            name='waypoint_publisher',
            remappings=[
                ('/waypt_in','/robot999/waypt_in')
            ]
        ),
        Node(
            package='adaptive_sample',
            executable='simple_robot',
            name='robot999',
            namespace='robot999',
            remappings=[
                ('/waypt_in','/robot999/waypt_in')
            ]
        ),
        Node(
            package='adaptive_sample',
            executable='visualize.py',
            name='vis',
        )
    ])