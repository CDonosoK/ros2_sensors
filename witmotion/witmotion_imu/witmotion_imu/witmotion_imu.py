import rclpy
import serial
import struct
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation

class WitmotionImu(Node):
    def __init__(self):
        super().__init__('witmotion_imu')
        
        self.imu_pub = self.create_publisher(Imu, '/witmotion/sensor/imu', 10)
        
        try:
            self.serial_device = serial.Serial('/dev/ttyUSB1', 115200)
            self.get_logger().info('Witmotion IMU node started')
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial device: {e}")
            rclpy.shutdown()

        self.rate = 10  # Hz
        self.timer = self.create_timer(1.0 / self.rate, self.read_data)

    def euler_to_quaternion(self, roll, pitch, yaw):
        rot = Rotation.from_euler('xyz', [yaw, pitch, roll], degrees=True)
        q = rot.as_quat()
        q = np.round(q, 3)
        return q[::-1]

    def read_data(self):
        try:
            data = self.serial_device.read_until(b'U')
            if data[0] == 0x61 and len(data) == 20:
                # Acceleration data
                acc = np.array(struct.unpack('<hhh', data[1:7])) / 32768.0 * 16.0 * 9.8
                acc = np.round(acc, 3)

                # Angular velocity data
                gyro = np.array(struct.unpack('<hhh', data[7:13])) / 32768.0 * 2000.0
                gyro = np.round(gyro, 3)
                
                # Angle data
                angle = np.array(struct.unpack('<hhh', data[13:19])) / 32768.0 * 180.0
                angle = np.round(angle, 3)
                q = self.euler_to_quaternion(angle[0], angle[1], angle[2])

                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'witmotion_imu'
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]
                imu_msg.angular_velocity.x = gyro[0]
                imu_msg.angular_velocity.y = gyro[1]
                imu_msg.angular_velocity.z = gyro[2]
                imu_msg.linear_acceleration.x = acc[0]
                imu_msg.linear_acceleration.y = acc[1]
                imu_msg.linear_acceleration.z = acc[2]
                
                self.imu_pub.publish(imu_msg)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    witmotion_imu = WitmotionImu()
    rclpy.spin(witmotion_imu)
    witmotion_imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
