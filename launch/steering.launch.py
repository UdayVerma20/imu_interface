from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('imu_interface')
    config = os.path.join(pkg_share, 'config', 'steering.yaml')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for steering Arduino'
    )

    return LaunchDescription([
        port_arg,
        Node(
            package='imu_interface',
            executable='steering_publisher.py',
            name='steering_publisher',
            output='screen',
            parameters=[config, {'port': LaunchConfiguration('port')}]
        )
    ])

