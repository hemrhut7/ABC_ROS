import os
import sys

# Dynamically search upwards for INS_python/.venv site-packages.
# If .venv exists, load site-packages from .venv; otherwise fall back to system Python environment.
def _setup_environment():
    curr_dir = os.path.abspath(os.path.dirname(__file__))
    py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    while True:
        cand1 = os.path.join(curr_dir, 'src', 'INS_python', '.venv', 'lib', py_ver, 'site-packages')
        cand2 = os.path.join(curr_dir, 'INS_python', '.venv', 'lib', py_ver, 'site-packages')
        submod1 = os.path.join(curr_dir, 'src', 'INS_python')
        submod2 = os.path.join(curr_dir, 'INS_python')
        
        found = False
        if os.path.exists(cand1):
            if cand1 not in sys.path:
                sys.path.insert(0, cand1)
            if submod1 not in sys.path:
                sys.path.insert(0, submod1)
            found = True
        elif os.path.exists(cand2):
            if cand2 not in sys.path:
                sys.path.insert(0, cand2)
            if submod2 not in sys.path:
                sys.path.insert(0, submod2)
            found = True

        if found:
            break
            
        parent = os.path.dirname(curr_dir)
        if parent == curr_dir:
            break
        curr_dir = parent

_setup_environment()

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, MagneticField, FluidPressure, JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data

from nav_ekf.filters.ins_gnss_kf import INS_GNSS
from nav_ekf.sensors.params import ICM20948_params
from nav_ekf.sensors.tools import accLeveling, magCompassing
from nav_ekf.filters.constants import EKF_MODE
from nav_ekf.core.coordinate_transformation import llh2ENU

class InsEkfNode(Node):
    def __init__(self):
        super().__init__('ins_ekf_node')

        # --- Parameters ---
        self.declare_parameter('ekf_mode', EKF_MODE.STATE_16)
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('gnss_topic', '/gnss/fix')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('baro_topic', '/baro/pressure')
        self.declare_parameter('vel_topic', '/velocity') # Body velocity / Encoder
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('wheel_radius', 0.0325)     # Default 3.25cm
        self.declare_parameter('wheel_separation', 0.20) # Default 0.20 m
        self.declare_parameter('left_wheel_name', 'left_wheel')
        self.declare_parameter('right_wheel_name', 'right_wheel')
        self.declare_parameter('use_joint_states', True)
        
        self.declare_parameter('enable_gnss_pos', False)
        self.declare_parameter('enable_gnss_vel', False)
        self.declare_parameter('enable_mag', True)
        self.declare_parameter('enable_baro', True)
        self.declare_parameter('enable_agv', True)
        self.declare_parameter('block_agv_h', False)
        self.declare_parameter('std_agv', [0.1, 0.1, 0.1])
        self.declare_parameter('lever_arm_agv', [-0.012, 0.015, -0.0805])
        self.declare_parameter('enable_nhc', False)
        self.declare_parameter('enable_zupt_hor', False)
        self.declare_parameter('press_params', 10.0)
        self.declare_parameter('std_mag', 50.0)
        self.declare_parameter('std_mag_yaw', 5.0) # deg

        self.declare_parameter('init_cov_p', 5.0)
        self.declare_parameter('init_cov_v', 0.5)
        self.declare_parameter('init_cov_att', 3.0)  # Initial pitch/roll covariance (deg)
        self.declare_parameter('init_cov_yaw', 30.0) # Initial yaw covariance (deg)
        self.declare_parameter('init_wait_time', 5.0) # Seconds to collect IMU/mag before EKF start
        
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')

        self.declare_parameter('path_min_dist', 0.05)  # meters
        self.declare_parameter('path_max_size', 1000)  # max pose points
        self.declare_parameter('path_pub_rate', 0.2)   # max publish interval in seconds (5Hz)

        # Pre-allocate static conversion matrices
        # Transformation from ROS FLU to EKF RFU
        self.R_flu2rfu = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ])

        # --- EKF Initialization ---
        mode = self.get_parameter('ekf_mode').value
        self.kf = INS_GNSS(dim=mode)
        self.kf.setIMUParams(ICM20948_params)
        
        # Default params (can be updated via ROS params if needed)
        self.kf.setGNSSParams(np.array([1.0, 1.0, 2.0, 0.1, 0.1, 0.2]))
        
        std_agv = tuple(self.get_parameter('std_agv').value)
        block_agv_h = self.get_parameter('block_agv_h').value
        lever_arm_agv_flu = np.array(self.get_parameter('lever_arm_agv').value, dtype=float)
        # Convert ROS FLU (x forward, y left, z up) to EKF RFU (x right, y forward, z up)
        lever_arm_agv_rfu = self.R_flu2rfu @ lever_arm_agv_flu
        self.kf.setAGVParams(std_vel=std_agv, is_block_hei=block_agv_h, lever_arm=lever_arm_agv_rfu)
        
        press_params = float(self.get_parameter('press_params').value)
        self.kf.setPressParams(press_params)
        
        std_mag = float(self.get_parameter('std_mag').value)
        std_mag_yaw = float(self.get_parameter('std_mag_yaw').value)
        self.kf.setMagParams(std_mag=std_mag, std_yaw=std_mag_yaw)
        
        self.initialized = False
        self.last_imu_time = None
        self.init_start_time = None
        self.init_acc_buf = []
        self.init_mag_buf = []
        
        # Buffer for latest sensor data
        self.latest_gnss = None
        self.latest_mag = None
        self.latest_baro = None
        self.latest_vel = None
        self.latest_w_flu = np.zeros(3)
        
        # Flags to trigger update
        self.new_gnss = False
        self.new_mag = False
        self.new_baro = False
        self.new_vel = False
        self.latest_vel_time = 0.0

        # --- Subscriptions ---
        self.imu_sub = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.imu_callback, qos_profile_sensor_data)
        self.gnss_sub = self.create_subscription(
            NavSatFix, self.get_parameter('gnss_topic').value, self.gnss_callback, qos_profile_sensor_data)
        
        if self.get_parameter('enable_mag').value:
            self.mag_sub = self.create_subscription(
                MagneticField, self.get_parameter('mag_topic').value, self.mag_callback, qos_profile_sensor_data)
        
        if self.get_parameter('enable_baro').value:
            self.baro_sub = self.create_subscription(
                FluidPressure, self.get_parameter('baro_topic').value, self.baro_callback, qos_profile_sensor_data)
        
        if self.get_parameter('enable_agv').value:
            if self.get_parameter('use_joint_states').value:
                self.joint_sub = self.create_subscription(
                    JointState, self.get_parameter('joint_states_topic').value, self.joint_states_callback, qos_profile_sensor_data)
            self.vel_sub = self.create_subscription(
                TwistStamped, self.get_parameter('vel_topic').value, self.vel_callback, qos_profile_sensor_data)

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, 'ins/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'ins/pose', 10)
        self.path_pub = self.create_publisher(Path, 'ins/path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Path accumulator & optimization
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.get_parameter('map_frame').value
        self.last_path_pos = None
        self.last_path_pub_time = 0.0

        self.get_logger().info(f"INS EKF Node started in mode {mode}")

    def gnss_callback(self, msg):
        if msg.status.status < 0 or np.isnan(msg.latitude) or np.isnan(msg.longitude): # No fix
            return
        self.latest_gnss = msg
        self.new_gnss = True

    def mag_callback(self, msg):
        # Convert ROS standard FLU (x forward, y left, z up) to EKF standard RFU (x right, y forward, z up)
        # and convert Tesla (T) to milliGauss (mGauss/mG, 1 T = 10^7 mGauss)
        self.latest_mag = np.array([
            -msg.magnetic_field.y * 1e7,
            msg.magnetic_field.x * 1e7,
            msg.magnetic_field.z * 1e7
        ])
        self.new_mag = True

    def baro_callback(self, msg):
        self.latest_baro = msg.fluid_pressure
        self.new_baro = True

    def vel_callback(self, msg):
        # Body velocity (e.g. from encoder TwistStamped)
        self.latest_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.latest_vel_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.new_vel = True

    def joint_states_callback(self, msg):
        # Directly compute body velocity from wheel encoder JointState
        if len(msg.name) >= 2 and len(msg.velocity) >= 2:
            indices = {name: i for i, name in enumerate(msg.name)}
            l_name = self.get_parameter('left_wheel_name').value
            r_name = self.get_parameter('right_wheel_name').value
            l_idx = indices.get(l_name, 0)
            r_idx = indices.get(r_name, 1)

            l_vel = msg.velocity[l_idx] if len(msg.velocity) > l_idx else 0.0
            r_vel = msg.velocity[r_idx] if len(msg.velocity) > r_idx else 0.0

            r = self.get_parameter('wheel_radius').value
            v_x = r * (l_vel + r_vel) / 2.0

            self.latest_vel = np.array([v_x, 0.0, 0.0])
            self.latest_vel_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.new_vel = True

    def imu_callback(self, msg):
        curr_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if not self.initialized:
            wait_time = float(self.get_parameter('init_wait_time').value)
            if self.init_start_time is None:
                self.init_start_time = curr_t
                self.init_acc_buf = []
                self.init_mag_buf = []
                self.get_logger().info(f"EKF initial alignment: waiting {wait_time:.1f}s after first IMU before starting estimation")

            # Convert ROS standard FLU to EKF standard RFU for leveling and mag yaw
            f = np.array([-msg.linear_acceleration.y, msg.linear_acceleration.x, msg.linear_acceleration.z])
            self.init_acc_buf.append(f)
            if self.latest_mag is not None:
                self.init_mag_buf.append(self.latest_mag)

            if curr_t - self.init_start_time < wait_time:
                return

            # Use the 5 seconds of collected IMU/mag data for attitude init
            init_acc = np.nanmean(np.vstack(self.init_acc_buf), axis=0)
            pitch, roll = accLeveling(init_acc)
            if len(self.init_mag_buf) > 0:
                init_mag = np.nanmean(np.vstack(self.init_mag_buf), axis=0)
                yaw = magCompassing(init_mag, pitch, roll)
            else:
                yaw = 0.0

            if self.get_parameter('enable_gnss_pos').value and self.latest_gnss is not None:
                pos0 = np.array([np.deg2rad(self.latest_gnss.latitude),
                                 np.deg2rad(self.latest_gnss.longitude),
                                 self.latest_gnss.altitude])
            else:
                pos0 = np.array([0.0, 0.0, 0.0])

            std_p = float(self.get_parameter('init_cov_p').value)
            std_v = float(self.get_parameter('init_cov_v').value)
            std_att = np.deg2rad(float(self.get_parameter('init_cov_att').value))
            std_yaw = np.deg2rad(float(self.get_parameter('init_cov_yaw').value))

            self.kf.setInitStatus(
                pos0, np.zeros(3), np.array([pitch, roll, yaw]),
                std_pos=std_p, std_vel=std_v, std_ver_ori=std_att, std_yaw=std_yaw
            )
            self.initialized = True
            self.ref_lla_rad = pos0
            self.get_logger().info(f"EKF Initialized after {wait_time:.1f}s at LLA: {np.rad2deg(pos0[0])}, {np.rad2deg(pos0[1])}, {pos0[2]}")

        # Save raw angular velocity in FLU frame for Odometry msg
        self.latest_w_flu = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # Prepare inputs for kf.run
        # Convert ROS standard FLU (x forward, y left, z up) to EKF standard RFU (x right, y forward, z up)
        # x_rfu = -y_flu, y_rfu = x_flu, z_rfu = z_flu
        w = np.array([-msg.angular_velocity.y, msg.angular_velocity.x, msg.angular_velocity.z])
        f = np.array([-msg.linear_acceleration.y, msg.linear_acceleration.x, msg.linear_acceleration.z])
        
        pos_gnss = np.zeros(3)
        vel_gnss = np.zeros(3)
        pos_qlt = np.array([1.0, 1.0, 2.0])
        vel_qlt = np.array([10.0, 10.0, 10.0])
        num_gnss = 0
        
        if self.new_gnss:
            pos_gnss = np.array([np.deg2rad(self.latest_gnss.latitude), 
                                 np.deg2rad(self.latest_gnss.longitude), 
                                 self.latest_gnss.altitude])
            if self.latest_gnss.position_covariance_type > 0:
                cov = self.latest_gnss.position_covariance
                if cov[0] > 0 and cov[4] > 0 and cov[8] > 0:
                    pos_qlt = np.sqrt(np.array([cov[0], cov[4], cov[8]]))
            num_gnss = 10 
        
        mag = self.latest_mag if self.latest_mag is not None else np.zeros(3)
        static_press = self.latest_baro if self.latest_baro is not None else 0
        ref_vel = self.latest_vel if self.latest_vel is not None else np.zeros(3)

        # Handle ZUPT Hor logic (if enabled)
        in_zupt_hor = False
        if self.get_parameter('enable_zupt_hor').value and self.latest_vel is not None:
            # Prevent stale velocity values from triggering false ZUPT locks
            if abs(curr_t - self.latest_vel_time) < 0.5:
                encoder_speed = abs(self.latest_vel[0])
                yaw_rate = abs(w[2])
                in_zupt_hor = (encoder_speed <= 0.05 and yaw_rate <= np.deg2rad(1.0))

        # Run EKF Update
        self.kf.run(
            curr_t, w, f,
            pos_gnss, vel_gnss, yaw_gnss=np.nan, num_gnss=num_gnss, 
            pos_qlt=pos_qlt, vel_qlt=vel_qlt,
            mag=mag, static_press=static_press, ref_vel=ref_vel,
            in_gnss=self.new_gnss and self.get_parameter('enable_gnss_pos').value,
            in_mag=self.new_mag,
            in_bar=self.new_baro,
            in_b_vel=self.new_vel,
            in_nhc=self.get_parameter('enable_nhc').value,
            in_zupt_hor=in_zupt_hor
        )

        # Reset flags
        self.new_gnss = False
        self.new_mag = False
        self.new_baro = False
        self.new_vel = False

        # Publish Results
        self.publish_results(msg.header.stamp)

    def publish_results(self, stamp):
        # 1. Get current state from EKF (LLA, ENU Vel, Euler)
        pos_lla_rad = self.kf.me.pos # [lat, lon, alt]
        vel_enu = self.kf.me.vel     # EKF navigation frame is already ENU
        
        # 2. Convert to ROS messages
        # Position relative to starting point
        pos_enu = llh2ENU(pos_lla_rad, self.ref_lla_rad)
        
        # Orientation: EKF provides R_rfu2enu. ROS expects R_flu2enu.
        # R_flu2enu = R_rfu2enu @ R_flu2rfu
        R_rfu2enu = self.kf.me.getDCM()
        R_b2w = R_rfu2enu @ self.R_flu2rfu
        
        # Convert rotation matrix to quaternion safely
        q = self.rotation_matrix_to_quaternion(R_b2w)
        
        # ROS nav_msgs/Odometry convention:
        # odom.pose is in header.frame_id (map frame, ENU)
        # odom.twist is in child_frame_id (base_link frame, FLU)
        vel_flu = R_b2w.T @ vel_enu

        # 3. Create messages
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.get_parameter('map_frame').value
        odom.child_frame_id = self.get_parameter('base_link_frame').value
        
        odom.pose.pose.position.x = float(pos_enu[0])
        odom.pose.pose.position.y = float(pos_enu[1])
        odom.pose.pose.position.z = float(pos_enu[2])
        odom.pose.pose.orientation = q
        
        odom.twist.twist.linear.x = float(vel_flu[0])
        odom.twist.twist.linear.y = float(vel_flu[1])
        odom.twist.twist.linear.z = float(vel_flu[2])

        odom.twist.twist.angular.x = float(self.latest_w_flu[0])
        odom.twist.twist.angular.y = float(self.latest_w_flu[1])
        odom.twist.twist.angular.z = float(self.latest_w_flu[2])
        
        self.odom_pub.publish(odom)
        
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        # Publish path with distance filtering, max size cap, and rate throttling
        min_dist = float(self.get_parameter('path_min_dist').value)
        max_size = int(self.get_parameter('path_max_size').value)
        pub_rate = float(self.get_parameter('path_pub_rate').value)

        curr_t = stamp.sec + stamp.nanosec * 1e-9
        should_append = False

        if self.last_path_pos is None:
            should_append = True
        else:
            dx = pos_enu[0] - self.last_path_pos[0]
            dy = pos_enu[1] - self.last_path_pos[1]
            dz = pos_enu[2] - self.last_path_pos[2]
            if (dx * dx + dy * dy + dz * dz) >= (min_dist * min_dist):
                should_append = True

        if should_append:
            self.last_path_pos = pos_enu.copy()
            self.path_msg.poses.append(pose)
            if max_size > 0 and len(self.path_msg.poses) > max_size:
                self.path_msg.poses.pop(0)

        if curr_t - self.last_path_pub_time >= pub_rate:
            if len(self.path_msg.poses) > 0:
                self.path_msg.header.stamp = stamp
                self.path_pub.publish(self.path_msg)
                self.last_path_pub_time = curr_t
        
        if self.get_parameter('publish_tf').value:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = odom.child_frame_id
            t.transform.translation.x = float(pos_enu[0])
            t.transform.translation.y = float(pos_enu[1])
            t.transform.translation.z = float(pos_enu[2])
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    def rotation_matrix_to_quaternion(self, R):
        quat = Rotation.from_matrix(R).as_quat()  # [x, y, z, w]
        return Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))

def main(args=None):
    rclpy.init(args=args)
    node = InsEkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if rclpy.ok():
            node.get_logger().error(f"Unexpected exception: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

