#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial


class SteeringInterface(Node):
    def __init__(self):
        super().__init__('steering_interface')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('steering_min', 0.0)
        self.declare_parameter('steering_max', 1023.0)
        self.declare_parameter('angle_min', -0.7854)  # -45 deg
        self.declare_parameter('angle_max', 0.7854)   # +45 deg

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.steering_min = self.get_parameter('steering_min').value
        self.steering_max = self.get_parameter('steering_max').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value

        # Conversion slopes
        self.angle_to_steer_slope = (self.steering_max - self.steering_min) / (self.angle_max - self.angle_min)
        self.steer_to_angle_slope = (self.angle_max - self.angle_min) / (self.steering_max - self.steering_min)

        # ROS interfaces
        self.pub = self.create_publisher(Float32, 'steering_angle', 10)
        self.sub = self.create_subscription(Float32, 'steering_cmd', self.cmd_callback, 10)

        # Connect to serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to {port} at {baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect: {e}")
            self.ser = None

        # Timer for polling sensor
        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz

    def read_serial(self):
        if not self.ser:
            return
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return
            raw_val = float(line)
            # Map to radians
            angle = (raw_val - self.steering_min) * self.steer_to_angle_slope + self.angle_min
            msg = Float32()
            msg.data = angle
            self.pub.publish(msg)
        except ValueError:
            pass

    def cmd_callback(self, msg: Float32):
        """Receive angle (radians), convert to raw steering, and send to Arduino."""
        if not self.ser:
            return
        angle = msg.data
        steering_val = (angle - self.angle_min) * self.angle_to_steer_slope + self.steering_min
        self.ser.write(f"{steering_val:.2f}\n".encode())
        self.get_logger().info(f"Sent steering command: {steering_val:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SteeringInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
