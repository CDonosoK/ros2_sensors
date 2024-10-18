import rclpy
import serial
import struct
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class WitmotionGps(Node):
    def __init__(self):
        super().__init__('witmotion_gps')

        self.imu_pub = self.create_publisher(NavSatFix, '/witmotion/sensor/imu', 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('port_', '/dev/ttyUSB0'),
                ('baudrate_', 115200),
                ('timeout_', 0.5),
                ('rate_', 10)
            ]
        )
        self.port_ = self.get_parameter('port_').get_parameter_value().string_value
        self.baudrate_ = self.get_parameter('baudrate_').get_parameter_value().integer_value
        self.timeout_ = self.get_parameter('timeout_').get_parameter_value().double_value
        self.rate = self.get_parameter('rate_').get_parameter_value().double_value

        try:
            self.serial_device = serial.Serial(self.port_, self.baudrate_)
            self.get_logger().info('Witmotion GPS node started')
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial device: {e}")
            rclpy.shutdown()

        self.timer = self.create_timer(1.0 / self.rate, self.read_data)

    def read_data(self):
        data = self.serial_device.readline().decode('utf-8').split(',')
        try:
            if data[0] == '$GNGGA':
                satelital = int(data[7])
                self.alt = float(data[9])
                if data[3] == 'S':
                    datos = str(data[2])
                    deg = int(datos[0:2])
                    min = float(datos[2:])
                    self.lat = float("-"+str(deg + min/60))
                else:
                    datos = str(data[2])
                    deg = int(datos[0:2])
                    min = float(datos[2:])
                    self.lat = float(deg + min/60)
                if data[5] == 'W':
                    datos = str(data[4])
                    deg = int(datos[0:3])
                    min = float(datos[3:])
                    self.lon = float("-"+str(deg + min/60))
                else:
                    datos = str(data[4])
                    deg = int(datos[0:3])
                    min = float(datos[3:])
                    self.lon = float(deg + min/60)

                gps_data = NavSatFix()
                gps_data.latitude = self.lat
                gps_data.longitude = self.lon
                gps_data.altitude = self.alt
                gps_data.status.status = satelital
                gps_data.header.stamp = self.get_clock().now().to_msg()
                gps_data.header.frame_id = "witmotion_gps"

        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    witmotion_gps = WitmotionGps()
    rclpy.spin(witmotion_gps)
    witmotion_gps.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()