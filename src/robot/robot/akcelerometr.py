#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import board
import busio
import adafruit_adxl34x
from geometry_msgs.msg import Quaternion

class AccelerometerPublisher(Node):
    def __init__(self):
        super().__init__('accelerometer_publisher')
        
        # Inicjalizacja I2C
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.accelerometer = adafruit_adxl34x.ADXL345(self.i2c, address=0x1D)
            self.accelerometer.range = adafruit_adxl34x.Range.RANGE_2_G
            self.get_logger().info('ADXL345 initialized successfully.')
        except ValueError as e:
            self.get_logger().error(f'Failed to initialize ADXL345: {e}')
            self.destroy_node()
            return
        
        # Nowe offsety na podstawie danych w spoczynku
        self.X_OFFSET = 0.06
        self.Y_OFFSET = 0.10
        self.Z_OFFSET = -10.19 

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50 Hz

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link' # Zdefiniuj ramkę sensora
        
        # Odczyt danych i korekcja offsetów
        x, y, z = self.accelerometer.acceleration
        imu_msg.linear_acceleration.x = -y - self.Y_OFFSET
        imu_msg.linear_acceleration.y = x - self.X_OFFSET
        imu_msg.linear_acceleration.z = z - self.Z_OFFSET
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance[0] = -1.0 
        imu_msg.angular_velocity_covariance[0] = -1.0 
        imu_msg.linear_acceleration_covariance[0] = 0.04 
        imu_msg.linear_acceleration_covariance[4] = 0.04
        imu_msg.linear_acceleration_covariance[8] = 0.04

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AccelerometerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()