from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='transport_system', executable='camera_node', output='screen'),
        Node(package='transport_system', executable='arm_node', output='screen'),
        Node(package='transport_system', executable='supervisor_gui', output='screen'),
        Node(package='transport_system_cpp', executable='robot_node', output='screen'),
    ])
