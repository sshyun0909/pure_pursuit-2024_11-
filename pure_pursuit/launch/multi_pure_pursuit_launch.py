from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'config_multi.yaml'
    )

    pure_pursuit = Node(
        package='pure_pursuit',
        executable='pure_pursuit_multi',
        name='pure_pursuit',
        parameters=[config]
    )

    ld.add_action(pure_pursuit)

    return ld
