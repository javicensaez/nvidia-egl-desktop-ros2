import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Ruta al archivo de configuración de RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('socialtech_challenge'),
        'config',
        'rviz_config.rviz'
    )

    # Nodo para RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([rviz_node])

