from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_converter',
            executable='point_cloud_converter_node',
            name='point_cloud_converter',
            remappings=[
               
                ('input_topic', '/vins_estimator/point_cloud'),
            ],
            output='screen'  # Print output to the screen
        )
    ])