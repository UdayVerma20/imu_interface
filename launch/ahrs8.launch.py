from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port for the AHRS-8 sensor"
    )

    baud_arg = DeclareLaunchArgument(
        "baud",
        default_value="115200",
        description="Baud rate for the AHRS-8 sensor"
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="fcu",
        description="Frame ID for published IMU data"
    )

    # Define node
    ahrs8_node = Node(
        package="sparton_ahrs8_driver",
        executable="ahrs8_nmea.py",
        name="ahrs8_driver",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baud": LaunchConfiguration("baud"),
            "frame_id": LaunchConfiguration("frame_id")
        }]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_id_arg,
        ahrs8_node
    ])
