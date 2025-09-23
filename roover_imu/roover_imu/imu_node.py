#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import board, busio
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lis3mdl import LIS3MDL
import math

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # I2C bus initialiseren
        i2c = busio.I2C(board.SCL, board.SDA)

        # IMU sensoren
        self.lsm6 = LSM6DSOX(i2c, address=0x6a)
        self.lis3 = LIS3MDL(i2c, address=0x1c)

        # Publishers
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', 10)

        # Timer → 50 Hz
        self.timer = self.create_timer(0.02, self.publish_data)

        self.get_logger().info("IMU node gestart (LSM6DSOX + LIS3MDL)")

    def publish_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Versnelling (m/s²)
        ax, ay, az = self.lsm6.acceleration
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        # Gyro (rad/s → library geeft in deg/s, dus omrekenen)
        gx, gy, gz = self.lsm6.gyro
        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)

        # Oriëntatie: nog niet, dus leeg (0 quaternion)
        imu_msg.orientation_covariance[0] = -1.0

        self.pub_imu.publish(imu_msg)

        # Magnetometer
        mag_msg = MagneticField()
        mag_msg.header = imu_msg.header
        mx, my, mz = self.lis3.magnetic
        mag_msg.magnetic_field.x = mx * 1e-6  # µT → Tesla
        mag_msg.magnetic_field.y = my * 1e-6
        mag_msg.magnetic_field.z = mz * 1e-6
        self.pub_mag.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
