"""
read_lidar.py — Standalone script for reading N10 LiDAR sensor data over USB serial.

This script is completely decoupled from Connector.py / Lidar.py and does not import them.
It reads raw 360-degree point data directly via pyserial and reports statistics.

Dependencies:
    - numpy
    - pyserial

Typical usage::

    python tools/read_lidar.py --port /dev/ttyACM0 --baud 230400
    python tools/read_lidar.py --max-scans 10
"""

import sys
import time
import argparse
import numpy as np
import serial
import serial.tools.list_ports


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

            distance = dist_raw / 1000.0
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


def parse_args():
    parser = argparse.ArgumentParser(description="Read N10 LiDAR sensor data over USB serial connection (Standalone).")
    parser.add_argument("--port", "-p", type=str, default=None, help="Serial port path (e.g. /dev/ttyACM0, /dev/ttyUSB0). Default: Auto-detect")
    parser.add_argument("--baud", "-b", type=int, default=230400, help="Baud rate (default: 230400)")
    parser.add_argument("--max-scans", "-n", type=int, default=0, help="Maximum number of 360° scans to read (0 = run continuously)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Print detailed point cloud summary for each scan")
    return parser.parse_args()


def main():
    args = parse_args()

    print("==================================================")
    print("      N10 LiDAR Reader (Standalone USB Serial)   ")
    print("==================================================")
    print(f"Target Port : {args.port if args.port else 'Auto-detecting USB port...'}")
    print(f"Baud Rate   : {args.baud}")
    print("==================================================")

    driver = N10LidarDriver(port=args.port, baud_rate=args.baud)

    if not driver.connect(timeout=1.0):
        print("[ERROR] Failed to connect to LiDAR device. Please check USB cable, port permission, or device power.")
        sys.exit(1)

    print(f"[INFO] Successfully connected to {driver.port_name} at {driver.baud_rate} baud.")
    print("[INFO] Reading LiDAR scans... Press Ctrl+C to stop.\n")

    scan_count = 0
    start_time = time.time()

    try:
        while True:
            data = driver.get_scan()

            if data and 'dist' in data:
                scan_count += 1
                dist = data['dist']
                angle = data['angle']
                x = data['x']
                y = data['y']
                timestamp = data['time']

                num_points = len(dist)
                min_dist = np.min(dist) if num_points > 0 else 0.0
                max_dist = np.max(dist) if num_points > 0 else 0.0
                avg_dist = np.mean(dist) if num_points > 0 else 0.0

                elapsed = time.time() - start_time
                fps = scan_count / elapsed if elapsed > 0 else 0.0

                print(f"[Scan #{scan_count:04d}] Time: {timestamp:.3f}s | Points: {num_points:3d} | "
                      f"Dist (Min/Avg/Max): {min_dist:.2f}m / {avg_dist:.2f}m / {max_dist:.2f}m | Rate: {fps:.1f} Hz")

                if args.verbose and num_points > 0:
                    print(f"  └─ First 3 points: X={x[:3].round(3)}, Y={y[:3].round(3)}, Dist={dist[:3].round(3)}m, Angle={angle[:3].round(1)}°")

                if args.max_scans > 0 and scan_count >= args.max_scans:
                    print(f"\n[INFO] Reached requested max scans limit ({args.max_scans}). Exiting loop.")
                    break

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt received. Stopping LiDAR reading...")
    finally:
        driver.disconnect()
        print("[INFO] LiDAR device disconnected cleanly.")


if __name__ == "__main__":
    main()
