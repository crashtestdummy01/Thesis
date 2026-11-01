from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bota_payload_utils",
            executable="bota_payload_estimator_node",
            output="screen",
            parameters=[{
                'node_name': 'bota_payload_estimator_node',
                'config_file': '/home/ergodic_group/Matej/pixi_franka_ros2/config/payload_estimator.json',
                'input_node_name': 'bota_ft',
            }],
        ),
    ])
