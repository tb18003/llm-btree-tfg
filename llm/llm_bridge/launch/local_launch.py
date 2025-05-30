from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('llm_bridge'),
        'config',
        'local_params.yaml'
        )

    llm_node = Node(
        package = 'llm_bridge',
        name = 'llm_bridge_node',
        executable = 'llm_bridge_node',
        parameters = [config]
    )

    llm_service = Node(
        package = 'llm_bridge',
        name = 'llm_service_node',
        executable = 'llm_service_node',
        parameters = [config]
    )

    ld.add_action(llm_node)
    ld.add_action(llm_service)
    return ld