from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mi_solucion_dock",
                executable="dock_lidar",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
