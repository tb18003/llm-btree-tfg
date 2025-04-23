from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('llm_bridge'),
        'config',
        'params.yaml'
        )

    llm_node = Node(
        package = 'llm_bridge',
        name = 'llm_bridge_node',
        executable = 'llm_bridge_node',
        parameters = [config]
    )

    llm_service = Node(
        package = 'llm_bridge',
        name = 'llm_bridge_service',
        executable = 'llm_bridge_service',
        parameters = [config]
    )
    ld.add_action(llm_node)
    ld.add_action(llm_service)
    return ld