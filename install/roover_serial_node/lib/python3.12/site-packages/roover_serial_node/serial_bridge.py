import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import struct

class RooverSerialBridge(Node):
    def __init__(self):
        super().__init__('roover_serial_bridge')
        self.serial = serial.Serial('/dev/ttyAMA0', 115200, timeout=1) # dev/serial0
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.publisher = self.create_publisher(String, '/gd32/status', 10)
        self.create_timer(0.05, self.read_serial)

    def cmd_callback(self, msg):
        steer = int(msg.angular.z * 1000)
        speed = int(msg.linear.x * 1000)
        start = 0xABCD
        checksum = (start + steer + speed) & 0xFFFF
        packed = struct.pack('<HhhH', start, steer, speed, checksum)
        self.serial.write(packed)

    def read_serial(self):
        if self.serial.in_waiting:
            data = self.serial.read(32)
            msg = String()
            msg.data = data.hex()
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RooverSerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
