from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_aggregator',
            namespace='can_aggregator1',
            executable='aggregator',
            name='aggregate'
        ),
        Node(
            package='can_aggregator',
            namespace='can_aggregator2',
            executable='commander',
            name='command'
        ),
        Node(
            package='ros2socketcan',
            namespace='ros2socketcan1',
            executable='ros2socketcan',
            name='ros2can'
        )
    ])
    
    
# Setup use Node object