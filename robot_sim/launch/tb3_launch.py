import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    rs_dir = get_package_share_directory('robot_sim')
    launch_dir = os.path.join(bringup_dir, 'launch')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'tb3_simulation_launch.py')),
        launch_arguments={
            "world": os.path.join(rs_dir, 'assets', 'mapir_lab_world.model'),
            "x_pose": "2.0",
            "y_pose": "-2.0",
            "map": os.path.join(rs_dir, 'assets', 'map.yaml')
        }.items())
    
    tts_sim = Node(
        package='robot_sim',
        executable='tts_sim_node',
        name='tts_sim_node',
        output='screen',
        prefix=['xterm -e '],
    )

    whisper_sim = Node(
        package='robot_sim',
        executable='whisper_sim_node',
        name='whisper_sim_node',
        output='screen',
        prefix=['xterm -e '],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(bringup_cmd)
    ld.add_action(tts_sim)
    ld.add_action(whisper_sim)

    return ld