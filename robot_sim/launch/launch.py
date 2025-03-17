from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('robot_sim'),
        'config',
        'topics_params.yaml'
        )

    node= Node(
        package = 'robot_sim',
        name = 'robot_sim_node',
        executable = 'robot_sim_node',
        parameters = [config]
    )
    ld.add_action(node)
    return ld