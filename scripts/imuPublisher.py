#!/usr/bin/env python3
"""
ROS2 port of the AHRS-8 NMEA driver (originally rospy).
Place in scripts/ and make executable.
"""

import math
import serial
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

# try to import standard tf transformations for quaternion/euler conversions
try:
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
except Exception:
    # fallback: many systems provide tf.transformations under different names
    from tf.transformations import euler_from_quaternion, quaternion_from_euler


def verify_checksum(response: str) -> bool:
    message_and_checksum = response.strip("$").split("*")
    if len(message_and_checksum) < 2:
        return False
    message = message_and_checksum[0]
    checksum = message_and_checksum[1].strip()

    calculated_checksum = 0
    for character in message:
        calculated_checksum ^= ord(character)
    hexstring = format(calculated_checksum, "02X")
    return hexstring == checksum


def millidegrees_to_radians(value: float) -> float:
    return (value / 1000.0) * (math.pi / 180.0)


def millig_to_meter(value: float) -> float:
    return (value / 1000.0) * 9.81


def populate_G(string: str, message: Imu) -> None:
    split_string = string.split(",")
    Gx_string = split_string[1]
    Gy_string = split_string[2]
    Gz_string = split_string[3].split("*")[0]

    Gx_float = millidegrees_to_radians(float(Gx_string.split("=")[1]))
    Gy_float = millidegrees_to_radians(float(Gy_string.split("=")[1]))
    Gz_float = millidegrees_to_radians(float(Gz_string.split("=")[1]))

    # X FORWARD Y LEFT Z UP with sign adjustments from original driver
    message.angular_velocity.x = Gx_float
    message.angular_velocity.y = -Gy_float
    message.angular_velocity.z = -Gz_float


def populate_QUAT(string: str, message: Imu) -> None:
    split_string = string.split(",")
    w_string = split_string[1]
    x_string = split_string[2]
    y_string = split_string[3]
    z_string = split_string[4].split("*")[0]

    w_float = float(w_string.split("=")[1])
    x_float = float(x_string.split("=")[1])
    y_float = float(y_string.split("=")[1])
    z_float = float(z_string.split("=")[1])

    # Build euler (r,p,y) from quaternion using ENU convention (matching original)
    euler = euler_from_quaternion([y_float, x_float, -z_float, w_float])

    # Adjust yaw reference (original added +pi/2)
    quat = quaternion_from_euler(euler[0], euler[1], euler[2] + math.pi / 2)

    # quaternion_from_euler returns (x, y, z, w)
    message.orientation.w = quat[3]
    message.orientation.x = quat[0]
    message.orientation.y = quat[1]
    message.orientation.z = quat[2]


def populate_A(string: str, message: Imu) -> None:
    split_string = string.split(",")
    Ax_string = split_string[1]
    Ay_string = split_string[2]
    Az_string = split_string[3].split("*")[0]

    Ax_float = millig_to_meter(float(Ax_string.split("=")[1]))
    Ay_float = millig_to_meter(float(Ay_string.split("=")[1]))
    Az_float = millig_to_meter(float(Az_string.split("=")[1]))

    # X FORWARD Y LEFT Z UP with sign adjustments from original driver
    message.linear_acceleration.x = -Ax_float
    message.linear_acceleration.y = Ay_float
    message.linear_acceleration.z = Az_float


def set_all_covariance(imu_msg: Imu, covariance_matrix: list) -> None:
    for i in range(9):
        imu_msg.orientation_covariance[i] = covariance_matrix[i]
        imu_msg.angular_velocity_covariance[i] = covariance_matrix[i]
        imu_msg.linear_acceleration_covariance[i] = covariance_matrix[i]


class AHRS8Node(Node):
    def __init__(self):
        super().__init__("imu_node")

        # declare parameters with defaults
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "fcu")
        self.declare_parameter("topic", "imu/data")

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.topic = self.get_parameter("topic").value

        self.get_logger().info(
            f"AHRS-8: Initializing on port {self.port} at {self.baud} baud, frame_id={self.frame_id}"
        )

        # publisher
        self.imu_pub = self.create_publisher(Imu, self.topic, 10)
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.frame_id

        default_covariance_matrix = [
            1e-6,
            0,
            0,
            0,
            1e-6,
            0,
            0,
            0,
            1e-6,
        ]
        set_all_covariance(self.imu_msg, default_covariance_matrix)

        # open serial
        try:
            self.compass_serial = serial.Serial(self.port, self.baud, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"AHRS-8: Serial open failed: {e}")
            raise

        # initial serial housekeeping (match original behavior)
        try:
            # writes must be bytes in py3
            self.compass_serial.write(b"\x13")
            self.compass_serial.write(b"$xxHDM\r\n")
            self.compass_serial.write(b"printmask 0 set drop\r\n")
            self.compass_serial.write(b"printmodulus 0 set drop\r\n")
            self.compass_serial.write(b"printtrigger 0 set drop\r\n")
            rclpy.spin_once(self, timeout_sec=0.1)
            self.compass_serial.write(b"\x11")
            rclpy.spin_once(self, timeout_sec=0.1)
            self.compass_serial.reset_input_buffer()
            self.compass_serial.reset_output_buffer()
        except serial.SerialException:
            self.get_logger().error("AHRS-8: Serial communications not opened properly!")

        self.get_logger().info("AHRS-8: Output reset, beginning to retrieve data.")

    def run(self):
        # main loop replicating original while not rospy.is_shutdown()
        try:
            while rclpy.ok():
                # Request angular velocity
                try:
                    self.compass_serial.write(b"$PSPA,G\r\n")
                    response_bytes = self.compass_serial.readline()
                    response = response_bytes.decode("utf-8", errors="ignore").strip()
                except serial.SerialException as e:
                    self.get_logger().error(f"AHRS-8: Serial read/write error: {e}")
                    continue

                if verify_checksum(response):
                    populate_G(response, self.imu_msg)
                else:
                    self.get_logger().error("AHRS-8: Bad checksum, skipping dataset.")
                    continue

                # Request quaternion orientation
                try:
                    self.compass_serial.write(b"$PSPA,QUAT\r\n")
                    response_bytes = self.compass_serial.readline()
                    response = response_bytes.decode("utf-8", errors="ignore").strip()
                except serial.SerialException as e:
                    self.get_logger().error(f"AHRS-8: Serial read/write error: {e}")
                    continue

                if verify_checksum(response):
                    populate_QUAT(response, self.imu_msg)
                else:
                    self.get_logger().error("AHRS-8: Bad checksum, skipping dataset.")
                    continue

                # Request linear acceleration
                try:
                    self.compass_serial.write(b"$PSPA,A\r\n")
                    response_bytes = self.compass_serial.readline()
                    response = response_bytes.decode("utf-8", errors="ignore").strip()
                except serial.SerialException as e:
                    self.get_logger().error(f"AHRS-8: Serial read/write error: {e}")
                    continue

                if verify_checksum(response):
                    populate_A(response, self.imu_msg)
                else:
                    self.get_logger().error("AHRS-8: Bad checksum, skipping dataset.")
                    continue

                # stamp and publish
                now = self.get_clock().now().to_msg()
                self.imu_msg.header.stamp = now
                self.imu_pub.publish(self.imu_msg)

                # allow rclpy to handle timers/callbacks
                rclpy.spin_once(self, timeout_sec=0.001)

        except KeyboardInterrupt:
            pass
        finally:
            try:
                if self.compass_serial and self.compass_serial.is_open:
                    self.compass_serial.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = AHRS8Node()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"AHRS-8: Exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
