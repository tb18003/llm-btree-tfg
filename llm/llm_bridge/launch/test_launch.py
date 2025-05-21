from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()
    test_config = os.path.join(
        get_package_share_directory('llm_bridge'),
        'config',
        'test_params.yaml'
        )
    
    config = os.path.join(
        get_package_share_directory('llm_bridge'),
        'config',
        'params.yaml'
        )

    llm_test_node = Node(
        package = 'llm_bridge',
        name = 'llm_test_node',
        executable = 'llm_bridge_test',
        parameters = [test_config]
    )

    llm_service = Node(
        package = 'llm_bridge',
        name = 'llm_test_service_node',
        executable = 'llm_bridge_service',
        parameters=[config],
        output="screen",
        prefix="xterm -e"
    )

    ld.add_action(llm_test_node)
    ld.add_action(llm_service)
    return ld