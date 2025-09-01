import rclpy
from rclpy.node import Node
import serial
import struct

class SerialFeedbackNode(Node):
    def __init__(self):
        super().__init__('serial_feedback_node')
        self.serial_port = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)

        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        # Zoek naar startbyte (0xAAAA)
        while True:
            sync = self.serial_port.read(2)
            if len(sync) < 2:
                return
            if sync == b'\xAA\xAA':
                break

        packet = self.serial_port.read(20)
        if len(packet) != 20:
            return

        data = struct.unpack('<2h6h2H', packet)
        fields = [
            'cmd1', 'cmd2',
            'speedR_meas', 'speedL_meas',
            'wheelR_cnt', 'wheelL_cnt',
            'batVoltage', 'boardTemp',
            'cmdLed', 'checksum'
        ]
        feedback = dict(zip(fields, data))
        self.get_logger().info(str(feedback))


def main(args=None):
    print("Feedback node gestart...")
    rclpy.init(args=args)
    node = SerialFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
