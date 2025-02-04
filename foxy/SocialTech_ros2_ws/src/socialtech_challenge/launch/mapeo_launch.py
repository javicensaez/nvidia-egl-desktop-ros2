import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Ruta al archivo de parámetros
    slam_params_file = os.path.join(
        get_package_share_directory('socialtech_challenge'),
        'config',
        'slam_params.yaml'
    )

    # Nodo de SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    return LaunchDescription([slam_node])

