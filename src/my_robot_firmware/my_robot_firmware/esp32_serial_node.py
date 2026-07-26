#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Message type imports
from sensor_msgs.msg import Imu, JointState, MagneticField, FluidPressure, BatteryState, LaserScan, Temperature
from std_msgs.msg import Int32, Float32MultiArray
from geometry_msgs.msg import Twist

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

        self.get_logger().info(f"Starting ESP32 Serial/Telemetry Node with display rate: {self.display_rate} Hz")

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

        # Initialize Publishers (sending command packets back to ESP32 micro-ROS)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_mode_pub = self.create_publisher(Int32, '/cmd_mode', 10)

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

    # Commands publisher helper API
    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f"Published cmd_vel: linear={linear_x}, angular={angular_z}")

    def publish_cmd_mode(self, mode: int):
        msg = Int32()
        msg.data = mode
        self.cmd_mode_pub.publish(msg)
        self.get_logger().info(f"Published cmd_mode: mode={mode}")

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

def main(args=None):
    rclpy.init(args=args)
    node = ESP32SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: Shutting down esp32_serial_node.")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
