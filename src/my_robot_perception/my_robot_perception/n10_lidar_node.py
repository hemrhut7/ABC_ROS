#!/usr/bin/env python3
"""
n10_lidar_node — ROS 2 Node for N10 LiDAR Sensor over USB Serial.

This node is completely self-contained and decoupled from Connector.py / Lidar.py.
It reads raw 360-degree point data directly from the USB serial interface and publishes
standard sensor_msgs/msg/LaserScan messages to the ROS 2 network (default topic: /scan).

Dependencies:
    - rclpy, sensor_msgs, std_msgs
    - numpy, pyserial

Typical execution::

    ros2 run my_robot_perception n10_lidar_node
    ros2 run my_robot_perception n10_lidar_node --ros-args -p port:=/dev/ttyACM0 -p baud_rate:=230400
"""

import sys
import time
import numpy as np
import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class N10LidarDriver:
    """
    Self-contained hardware driver for N10 USB LiDAR sensor.
    """
    HEADER = b'\xa5\x5a'
    PACKET_SIZE = 58

    def __init__(self, port: str = None, baud_rate: int = 230400):
        self.port_name = port
        self.baud_rate = baud_rate
        self.ser = None
        self.last_angle = 0.0

        self.scan_x = []
        self.scan_y = []
        self.scan_dist = []
        self.scan_angle = []
        self.scan_intensity = []

    def find_port(self) -> str:
        if self.port_name:
            return self.port_name

        targets = ["usb", "ttyusb", "ttyacm", "uart"]
        for p in serial.tools.list_ports.comports():
            info_str = f"{p.device} {p.description} {p.hwid} {p.manufacturer or ''}".lower()
            if any(t in info_str for t in targets):
                self.port_name = p.device
                return p.device
        return None

    def connect(self, timeout: float = 1.0) -> bool:
        if not self.port_name:
            self.find_port()
        if not self.port_name:
            return False

        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            return True
        except Exception as e:
            print(f"[ERROR] N10LidarDriver connect exception: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    @staticmethod
    def cal_crc8(data: bytes) -> int:
        return sum(data) & 0xFF

    def read_packet(self) -> bytes:
        if not self.ser or not self.ser.is_open:
            return None

        header_buf = bytearray()
        attempts = 0
        while attempts < 1000:
            b = self.ser.read(1)
            if not b:
                return None
            header_buf += b
            if len(header_buf) > 2:
                header_buf = header_buf[1:]
            if header_buf == self.HEADER:
                payload = self.ser.read(56)
                if len(payload) == 56:
                    return bytes(self.HEADER + payload)
                return None
            attempts += 1
        return None

    def get_scan(self) -> dict:
        msg = self.read_packet()
        if not msg or len(msg) < self.PACKET_SIZE:
            return None

        # Check CRC-8
        if msg[57] != self.cal_crc8(msg[:57]):
            return None

        start_angle = (msg[5] * 256 + msg[6]) / 100.0
        end_angle = (msg[55] * 256 + msg[56]) / 100.0

        if end_angle < start_angle:
            angle_diff = (end_angle + 360.0) - start_angle
        else:
            angle_diff = end_angle - start_angle

        points_in_packet = 16
        curr_x, curr_y, curr_dist, curr_angle, curr_intensity = [], [], [], [], []

        for i in range(points_in_packet):
            idx = 7 + i * 3
            dist_raw = msg[idx] * 256 + msg[idx + 1]
            intensity = msg[idx + 2]

            if dist_raw == 0xFFFF:
                continue

            distance = dist_raw / 1000.0  # meters
            angle = start_angle + (angle_diff / (points_in_packet - 1)) * i
            if angle >= 360.0:
                angle -= 360.0

            angle_rad = -np.deg2rad(angle)
            x = distance * np.cos(angle_rad)
            y = distance * np.sin(angle_rad)

            curr_x.append(x)
            curr_y.append(y)
            curr_dist.append(distance)
            curr_angle.append(angle)
            curr_intensity.append(intensity)

        revolution_completed = False
        if start_angle < self.last_angle - 180.0:
            revolution_completed = True
        self.last_angle = start_angle

        result = None
        if revolution_completed and len(self.scan_angle) > 0:
            result = {
                'time': time.time(),
                'x': np.array(self.scan_x),
                'y': np.array(self.scan_y),
                'dist': np.array(self.scan_dist),
                'angle': np.array(self.scan_angle),
                'intensity': np.array(self.scan_intensity)
            }
            self.scan_x = []
            self.scan_y = []
            self.scan_dist = []
            self.scan_angle = []
            self.scan_intensity = []

        self.scan_x.extend(curr_x)
        self.scan_y.extend(curr_y)
        self.scan_dist.extend(curr_dist)
        self.scan_angle.extend(curr_angle)
        self.scan_intensity.extend(curr_intensity)

        return result


class N10LidarNode(Node):
    """
    ROS 2 Node for N10 LiDAR serial driver.
    """
    def __init__(self):
        super().__init__('n10_lidar_node')

        # Declare ROS 2 parameters
        self.declare_parameter('port', '')                # Empty string = auto-detect USB port
        self.declare_parameter('baud_rate', 230400)       # N10 LiDAR default baud rate
        self.declare_parameter('frame_id', 'laser_frame')  # Laser frame ID
        self.declare_parameter('topic_name', '/scan')     # Output LaserScan topic name
        self.declare_parameter('range_min', 0.05)         # Minimum valid range in meters
        self.declare_parameter('range_max', 12.0)         # Maximum valid range in meters

        # Read parameters
        port_param = self.get_parameter('port').value
        self.port = port_param if port_param else None
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic_name = str(self.get_parameter('topic_name').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.get_logger().info(
            f"Initializing N10 LiDAR Node (Standalone):\n"
            f"  Target Port: {self.port if self.port else 'Auto-detecting USB port'}\n"
            f"  Baud Rate: {self.baud_rate}\n"
            f"  Frame ID: {self.frame_id}\n"
            f"  Output Topic: {self.topic_name}\n"
            f"  Range Min/Max: {self.range_min}m / {self.range_max}m"
        )

        # Create ROS 2 LaserScan publisher
        self.scan_pub = self.create_publisher(LaserScan, self.topic_name, 10)

        # Initialize decoupled driver instance
        self.driver = N10LidarDriver(port=self.port, baud_rate=self.baud_rate)

        # Connect to hardware
        if not self.driver.connect(timeout=1.0):
            self.get_logger().error(
                "Failed to connect to N10 LiDAR hardware. "
                "Please verify USB physical connection, permissions (dialout/tty), or device power."
            )
            raise RuntimeError("LiDAR connection failed")

        self.get_logger().info(f"Connected successfully to LiDAR at port: {self.driver.port_name}")

        # Set up polling timer for processing incoming serial packets (200 Hz = 5ms interval)
        self.timer = self.create_timer(0.005, self.poll_lidar)

        self.last_scan_time = time.time()
        self.scan_count = 0

    def poll_lidar(self):
        """
        Periodically poll and drain all available serial data packets from N10 LiDAR buffer.
        """
        try:
            while self.driver.ser and self.driver.ser.is_open:
                if self.driver.ser.in_waiting < self.driver.PACKET_SIZE:
                    break

                data = self.driver.get_scan()
                if data and 'dist' in data:
                    self.publish_scan(data)
        except Exception as e:
            self.get_logger().error(f"Error during LiDAR polling loop: {e}")

    def publish_scan(self, data: dict):
        try:
            dist = data['dist']
            angle_deg = data['angle']
            intensity = data.get('intensity', np.array([]))

            if len(dist) == 0:
                return

            current_time = time.time()
            scan_time = float(current_time - self.last_scan_time)
            self.last_scan_time = current_time
            self.scan_count += 1

            # Convert angles from degrees to radians
            angle_rad = np.deg2rad(angle_deg)

            # Sort points by angle
            sort_indices = np.argsort(angle_rad)
            sorted_angles = angle_rad[sort_indices]
            sorted_dist = dist[sort_indices]
            sorted_intensity = intensity[sort_indices] if len(intensity) == len(dist) else None

            # Construct ROS 2 LaserScan message
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = self.frame_id

            scan_msg.angle_min = float(sorted_angles[0])
            scan_msg.angle_max = float(sorted_angles[-1])

            if len(sorted_angles) > 1:
                scan_msg.angle_increment = float((sorted_angles[-1] - sorted_angles[0]) / (len(sorted_angles) - 1))
                scan_msg.time_increment = float(scan_time / len(sorted_angles))
            else:
                scan_msg.angle_increment = 0.0
                scan_msg.time_increment = 0.0

            scan_msg.scan_time = scan_time
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max

            ranges_list = []
            for r in sorted_dist:
                r_val = float(r)
                if self.range_min <= r_val <= self.range_max:
                    ranges_list.append(r_val)
                else:
                    ranges_list.append(float('inf'))

            scan_msg.ranges = ranges_list

            if sorted_intensity is not None:
                scan_msg.intensities = [float(i) for i in sorted_intensity]
            else:
                scan_msg.intensities = []

            # Publish LaserScan message
            self.scan_pub.publish(scan_msg)

            if self.scan_count % 50 == 0:
                self.get_logger().info(
                    f"Published scan #{self.scan_count}: {len(ranges_list)} points, "
                    f"Scan Rate: {1.0 / scan_time:.1f} Hz"
                )

        except Exception as e:
            self.get_logger().error(f"Error during publish_scan: {e}")

    def destroy_node(self):
        self.get_logger().info("Disconnecting LiDAR device and shutting down node...")
        try:
            if hasattr(self, 'driver') and self.driver:
                self.driver.disconnect()
        except Exception as e:
            self.get_logger().warn(f"Error during disconnect: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = N10LidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[FATAL] N10 LiDAR Node terminated unexpectedly: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
