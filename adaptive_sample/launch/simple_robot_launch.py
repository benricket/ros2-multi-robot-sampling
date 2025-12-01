from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adaptive_sample',
            executable='gp_test',
            name='model'
        ),
        Node(
            package='adaptive_sample',
            executable='visualize.py',
            name='visualizer'
        ),
        Node(
            package='adaptive_sample',
            executable='simple_robot',
            parameters=[{'id':999}],
            name='robot999',
            namespace='robot999'
        ),
    ])