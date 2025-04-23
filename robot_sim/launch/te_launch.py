from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('robot_sim'),
        'config',
        'params.yaml'
        )

    rs_node = Node(
        package = 'robot_sim',
        name = 'robot_sim_node',
        executable = 'robot_sim_node',
        parameters = [config]
    )

    tm_node = Node(
        package = 'task_manager',
        name = 'task_executor_node',
        executable = 'task_executor_node',
        parameters = [config]
    )
    ld.add_action(rs_node)
    ld.add_action(tm_node)
    return ld