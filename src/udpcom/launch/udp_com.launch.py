import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('udpcom'),
        'config',
        'udp.yaml'
    )

    return LaunchDescription([
        Node(
            package='udpcom',
            executable='udpcom_receiver_node',
            name='udpcom_receiver_node',
            output='screen',
            emulate_tty=True,
            parameters=[config_dir]
        ),
        Node(
            package='udpcom',
            executable='udpcom_sender_node',
            name='udpcom_sender_node',
            output='screen',
            emulate_tty=True,
            parameters=[config_dir]
        )
    ])
