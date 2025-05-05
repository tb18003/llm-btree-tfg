import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'tb3_simulation_launch.py')),
        launch_arguments={
            "world": "/home/tonib/Documentos/tfg/gazebo/world_models/mapir_lab_v1/mapir_lab_world.model",
            "x_pose": "2.0",
            "y_pose": "-2.0",
            "map": "/home/tonib/map/map.yaml"
        }.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(bringup_cmd)

    return ld