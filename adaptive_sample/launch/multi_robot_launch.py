from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    nodes_to_launch = [
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
            executable='visualize.py',
            name='vis',
        )
    ]

    num_robots_str = LaunchConfiguration("num_robots").perform(context)
    num_robots = int(num_robots_str)

    for i in range(num_robots):
        namespace = f'robot{i}'
        nodes_to_launch.append(
            Node(
                package='adaptive_sample',
                namespace=namespace,
                executable='simple_robot',
                name='sim'
            )
        )
    
    return nodes_to_launch

def generate_launch_description():
    num_robots_arg = DeclareLaunchArgument(
        "num_robots",
        default_value='2',
        description="Count of robots to simulate"
    )
    return LaunchDescription([num_robots_arg, OpaqueFunction(function=launch_setup)])