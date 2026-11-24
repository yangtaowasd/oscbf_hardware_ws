from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="oscbf_control", executable=script, namespace="nero_sim", output="screen")
        for script in ("nero_sim_node.py", "nero_control_node.py")
    ])
