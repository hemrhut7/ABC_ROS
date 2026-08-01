#!/usr/bin/env python3
import math
import os
import signal
import subprocess
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Message type imports
from sensor_msgs.msg import Imu, JointState, MagneticField, FluidPressure, BatteryState, LaserScan, Temperature
from std_msgs.msg import Int32, Float32MultiArray
from geometry_msgs.msg import Twist, TwistStamped

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw) in degrees.
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# System Modes mapping (matching hal_type_define.h)
SYSTEM_MODES = {
    0: "MODE_STOP",
    1: "MODE_FREE",
    2: "MODE_PWM",
    3: "MODE_MOTOR",
    4: "MODE_ANGLE",
    5: "MODE_VELOCITY",
    6: "MODE_REMOTE"
}

class ESP32SerialNode(Node):
    def __init__(self):
        super().__init__('esp32_serial_node')

        # Node parameters
        self.declare_parameter('display_rate', 1.0) # Hz for terminal print
        self.display_rate = self.get_parameter('display_rate').value

        # Auto Record Parameters
        self.declare_parameter('auto_record', True)
        self.declare_parameter('record_output_dir', 'ros2_bag')
        self.declare_parameter('record_topics', [
            '/imu/data_raw',
            '/imu/mag',
            '/baro/pressure',
            '/baro/temperature',
            '/joint_states',
            '/battery_state',
            '/system_mode',
            '/pid_target',
            '/gnss/fix',
            '/tf_static'
        ])

        self.auto_record = self.get_parameter('auto_record').value
        raw_dir = self.get_parameter('record_output_dir').value

        # Automatic path mapping for Docker container vs Host
        if os.path.exists('/workspace') and os.path.isdir('/workspace'):
            # Running inside Docker container: force saving to /workspace/ros2_bag so it maps to Host's ~/ROS_ABC/ros2_bag
            if raw_dir.startswith('/home/hank/ROS_ABC'):
                target_dir = raw_dir.replace('/home/hank/ROS_ABC', '/workspace', 1)
            elif os.path.isabs(raw_dir) and not raw_dir.startswith('/workspace'):
                target_dir = os.path.join('/workspace', 'ros2_bag')
            else:
                target_dir = os.path.join('/workspace', raw_dir.lstrip('/'))
        else:
            # Running directly on Host
            expanded = os.path.expanduser(raw_dir)
            if os.path.isabs(expanded):
                target_dir = expanded
            else:
                target_dir = os.path.join('/home/hank/ROS_ABC', expanded)

        self.record_output_dir = os.path.abspath(target_dir)
        self.record_topics = self.get_parameter('record_topics').value
        self.record_process = None

        self.get_logger().info(f"Starting ESP32 Serial/Telemetry Node with display rate: {self.display_rate} Hz")

        if self.auto_record:
            self.start_rosbag_recording()

        # Local state database for telemetry
        self.state = {
            'imu': None,
            'mag': None,
            'pressure': None,
            'temperature': None,
            'joint_states': None,
            'battery': None,
            'scan': None,
            'system_mode': None,
            'delay_count': None,
            'pid_target': None
        }

        # Counters for stats
        self.msg_counts = {k: 0 for k in self.state.keys()}

        # Initialize Subscribers (subscribing to topics published by ESP32 micro-ROS agent)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(MagneticField, '/imu/mag', self.mag_callback, qos_profile_sensor_data)
        self.create_subscription(FluidPressure, '/baro/pressure', self.pressure_callback, qos_profile_sensor_data)
        self.create_subscription(Temperature, '/baro/temperature', self.temp_callback, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Int32, '/system_mode', self.system_mode_callback, qos_profile_sensor_data)
        self.create_subscription(Int32, '/delay_count', self.delay_count_callback, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/pid_target', self.pid_target_callback, qos_profile_sensor_data)

        # Initialize Publishers (sending command packets back to ESP32 micro-ROS) - DISABLED to prevent accidental command transmission
        # self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.cmd_mode_pub = self.create_publisher(Int32, '/cmd_mode', 10)

        # Setup periodic display timer
        self.timer = self.create_timer(1.0 / self.display_rate, self.display_telemetry)

    # Callback implementations
    def imu_callback(self, msg):
        self.state['imu'] = msg
        self.msg_counts['imu'] += 1

    def mag_callback(self, msg):
        self.state['mag'] = msg
        self.msg_counts['mag'] += 1

    def pressure_callback(self, msg):
        self.state['pressure'] = msg
        self.msg_counts['pressure'] += 1

    def temp_callback(self, msg):
        self.state['temperature'] = msg
        self.msg_counts['temperature'] += 1

    def joint_states_callback(self, msg):
        self.state['joint_states'] = msg
        self.msg_counts['joint_states'] += 1

    def battery_callback(self, msg):
        self.state['battery'] = msg
        self.msg_counts['battery'] += 1

    def scan_callback(self, msg):
        self.state['scan'] = msg
        self.msg_counts['scan'] += 1

    def system_mode_callback(self, msg):
        self.state['system_mode'] = msg.data
        self.msg_counts['system_mode'] += 1

    def delay_count_callback(self, msg):
        self.state['delay_count'] = msg.data
        self.msg_counts['delay_count'] += 1

    def pid_target_callback(self, msg):
        self.state['pid_target'] = msg.data
        self.msg_counts['pid_target'] += 1

    # Commands publisher helper API (DISABLED to prevent accidental command transmission)
    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        # msg = Twist()
        # msg.linear.x = linear_x
        # msg.angular.z = angular_z
        # self.cmd_vel_pub.publish(msg)
        self.get_logger().warn("publish_cmd_vel is currently DISABLED to prevent accidental command transmission.")

    def publish_cmd_mode(self, mode: int):
        # msg = Int32()
        # msg.data = mode
        # self.cmd_mode_pub.publish(msg)
        self.get_logger().warn("publish_cmd_mode is currently DISABLED to prevent accidental command transmission.")

    # Telemetry dashboard printing
    def display_telemetry(self):
        lines = []
        lines.append("==================================================")
        lines.append("           ESP32 TELEMETRY MONITOR (SERIAL)       ")
        lines.append("==================================================")

        # 1. System Mode and Delay Count
        mode = self.state['system_mode']
        delay = self.state['delay_count']
        mode_name = SYSTEM_MODES.get(mode, "UNKNOWN") if mode is not None else "N/A"
        mode_str = f"{mode_name} ({mode})" if mode is not None else "N/A"
        delay_str = f"{delay}" if delay is not None else "N/A"
        lines.append(f"[System]  Mode: {mode_str:<20} | Delay Count: {delay_str}")

        # 2. Battery
        bat = self.state['battery']
        bat_str = f"{bat.voltage:.2f} V" if bat is not None else "N/A"
        lines.append(f"[Battery] Voltage: {bat_str}")

        # 3. Barometer and Temperature
        temp = self.state['temperature']
        pres = self.state['pressure']
        temp_str = f"{temp.temperature:.2f} °C" if temp is not None else "N/A"
        pres_str = f"{pres.fluid_pressure / 100.0:.2f} hPa" if pres is not None else "N/A"
        lines.append(f"[Env]     Temp: {temp_str:<12} | Pressure: {pres_str}")

        # 4. IMU
        imu = self.state['imu']
        if imu is not None:
            r, p, y = euler_from_quaternion(
                imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w
            )
            lines.append(f"[IMU]     Roll: {r:>6.1f}° | Pitch: {p:>6.1f}° | Yaw: {y:>6.1f}°")
            lines.append(f"          Accel: [x: {imu.linear_acceleration.x:>6.2f}, y: {imu.linear_acceleration.y:>6.2f}, z: {imu.linear_acceleration.z:>6.2f}] m/s²")
            lines.append(f"          Gyro:  [x: {imu.angular_velocity.x:>6.2f}, y: {imu.angular_velocity.y:>6.2f}, z: {imu.angular_velocity.z:>6.2f}] rad/s")
        else:
            lines.append("[IMU]     No Data")

        # 5. Magnetometer
        mag = self.state['mag']
        if mag is not None:
            mx, my, mz = mag.magnetic_field.x * 1e6, mag.magnetic_field.y * 1e6, mag.magnetic_field.z * 1e6
            lines.append(f"[Mag]     Field: [x: {mx:>6.1f}, y: {my:>6.1f}, z: {mz:>6.1f}] uT")
        else:
            lines.append("[Mag]     No Data")

        # 6. Joint States
        js = self.state['joint_states']
        if js is not None and len(js.name) >= 2:
            indices = {name: i for i, name in enumerate(js.name)}
            l_idx = indices.get("left_wheel", 0)
            r_idx = indices.get("right_wheel", 1)

            l_pos = js.position[l_idx] if len(js.position) > l_idx else 0.0
            r_pos = js.position[r_idx] if len(js.position) > r_idx else 0.0
            l_vel = js.velocity[l_idx] if len(js.velocity) > l_idx else 0.0
            r_vel = js.velocity[r_idx] if len(js.velocity) > r_idx else 0.0
            l_eff = js.effort[l_idx] if len(js.effort) > l_idx else 0.0
            r_eff = js.effort[r_idx] if len(js.effort) > r_idx else 0.0

            lines.append(f"[Joints]  left_wheel  | pos: {l_pos:>6.2f} rad | vel: {l_vel:>6.2f} rad/s | pwm: {l_eff:>5.1f}")
            lines.append(f"          right_wheel | pos: {r_pos:>6.2f} rad | vel: {r_vel:>6.2f} rad/s | pwm: {r_eff:>5.1f}")
        else:
            lines.append("[Joints]  No Data")

        # 7. LaserScan
        scan = self.state['scan']
        if scan is not None and len(scan.ranges) > 0:
            valid_ranges = [r for r in scan.ranges if scan.range_min <= r <= scan.range_max]
            pt_count = len(scan.ranges)
            avg_range = sum(valid_ranges) / len(valid_ranges) if len(valid_ranges) > 0 else 0.0
            lines.append(f"[Lidar]   Points: {pt_count:<6} | Avg Range: {avg_range:.2f} m (range: {scan.range_min:.2f}m - {scan.range_max:.2f}m)")
        else:
            lines.append("[Lidar]   No Data")

        # 8. PID Target Array
        pt = self.state['pid_target']
        if pt is not None and len(pt) >= 6:
            lines.append(f"[PID Tar] Target Val: {pt[0]:>6.2f} | Motor RPM L: {pt[1]:>6.1f} | R: {pt[2]:>6.1f}")
            lines.append(f"          Pitch Tar:  {pt[3]:>6.2f} | Vel Tar:     {pt[4]:>6.2f} | Steer RPM: {pt[5]:>6.1f}")
        else:
            lines.append("[PID Tar] No Data")

        lines.append("--------------------------------------------------")
        lines.append("Message rates overview (received counts):")
        stats_str = ", ".join([f"{k}:{v}" for k, v in self.msg_counts.items()])
        lines.append(f"  {stats_str}")
        lines.append("==================================================")

        print("\033[H\033[J", end="")
        print("\n".join(lines))

    def start_rosbag_recording(self):
        try:
            os.makedirs(self.record_output_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            bag_name = f"rosbag2_{timestamp}"
            bag_path = os.path.join(self.record_output_dir, bag_name)

            import shutil
            ros2_exec = shutil.which('ros2')
            if not ros2_exec:
                for candidate in ['/opt/ros/humble/install/bin/ros2', '/opt/ros/humble/bin/ros2']:
                    if os.path.exists(candidate):
                        ros2_exec = candidate
                        break
            if not ros2_exec:
                ros2_exec = 'ros2'

            cmd = [ros2_exec, 'bag', 'record', '-o', bag_path] + list(self.record_topics)
            self.get_logger().info(f"[Auto Record] Starting rosbag recording -> {bag_path}")
            self.get_logger().info(f"[Auto Record] Exec: {ros2_exec} | Target topics: {self.record_topics}")

            self.record_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )
        except Exception as e:
            self.get_logger().error(f"[Auto Record] Failed to start rosbag recording: {e}")

    def stop_rosbag_recording(self):
        if self.record_process and self.record_process.poll() is None:
            self.get_logger().info("[Auto Record] Stopping rosbag recording process cleanly...")
            try:
                os.killpg(os.getpgid(self.record_process.pid), signal.SIGINT)
                self.record_process.wait(timeout=5)
                self.get_logger().info("[Auto Record] Rosbag recording saved successfully.")
            except Exception as e:
                self.get_logger().warn(f"[Auto Record] Error stopping rosbag record process: {e}")
            self.record_process = None

    def destroy_node(self):
        self.stop_rosbag_recording()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
