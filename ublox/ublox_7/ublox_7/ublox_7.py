#!/usr/bin/env python3
import sys, os, json, threading, http.server, socketserver
import rclpy
from rclpy.node import Node
from serial import Serial, SerialException
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32

class Ublox7(Node):
    def __init__(self):
        super().__init__('ublox_7')
        self.gps_pub = self.create_publisher(NavSatFix, '/ublox_7/sensor/gps', 10)

        self.sat_pub = self.create_publisher(Int32, '/ublox_7/sensor/satellites', 10)

        self.declare_parameters('', [
            ('port_',      '/dev/ttyACM0'),
            ('baudrate_',  115200),
            ('timeout_',   0.5),
            ('rate_',      10.0),
        ])
        self.port_     = self.get_parameter('port_').get_parameter_value().string_value
        self.baudrate_ = self.get_parameter('baudrate_').get_parameter_value().integer_value
        self.rate      = self.get_parameter('rate_').get_parameter_value().double_value

        try:
            self.ser = Serial(self.port_, self.baudrate_, timeout=self.get_parameter('timeout_').value)
        except SerialException as e:
            self.get_logger().error(f"Cannot open serial port: {e}")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(1.0/self.rate, self.read_data)

    def read_data(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        parts = line.split(',')
        if parts[0] != '$GPGGA' or len(parts) < 10:
            return

        try:
            sats = int(parts[7])
            alt  = float(parts[9])

            lat_raw = parts[2]
            lat = (int(lat_raw[:2]) + float(lat_raw[2:])/60.0)
            if parts[3]=='S': lat = -lat

            lon_raw = parts[4]
            lon = (int(lon_raw[:3]) + float(lon_raw[3:])/60.0)
            if parts[5]=='W': lon = -lon

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'ublox_7'
            msg.latitude  = lat
            msg.longitude = lon
            msg.altitude  = alt

            self.gps_pub.publish(msg)

            sat_msg = Int32()
            sat_msg.data = sats
            self.sat_pub.publish(sat_msg)

        except Exception as e:
            self.get_logger().error(f"Parse GPGGA failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Ublox7()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
