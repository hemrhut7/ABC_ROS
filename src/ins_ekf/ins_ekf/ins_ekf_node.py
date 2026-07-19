import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, MagneticField, FluidPressure
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.time import Time

# Add project root or submodule to path to import nav_ekf
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
submodule_dir = os.path.abspath(os.path.join(parent_dir, '..', 'INS_python'))
if os.path.exists(submodule_dir):
    sys.path.insert(0, submodule_dir)
sys.path.insert(0, parent_dir)

from nav_ekf.filters.ins_gnss_kf import INS_GNSS
from nav_ekf.sensors.params import MTI7_params
from nav_ekf.filters.kalman_filter import EKF_MODE
from nav_ekf.core.coordinate_transformation import llh2ENU

class InsEkfNode(Node):
    def __init__(self):
        super().__init__('ins_ekf_node')

        # --- Parameters ---
        self.declare_parameter('ekf_mode', EKF_MODE.STATE_16)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gnss_topic', '/gnss/fix')
        self.declare_parameter('mag_topic', '/mag/data')
        self.declare_parameter('baro_topic', '/pressure')
        self.declare_parameter('vel_topic', '/velocity') # Body velocity / Encoder
        
        self.declare_parameter('enable_gnss_pos', True)
        self.declare_parameter('enable_gnss_vel', True)
        self.declare_parameter('enable_mag', True)
        self.declare_parameter('enable_baro', True)
        self.declare_parameter('enable_agv', True)
        self.declare_parameter('enable_nhc', False)
        self.declare_parameter('enable_zupt_hor', False)
        
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_link_frame', 'base_link')

        # --- EKF Initialization ---
        mode = self.get_parameter('ekf_mode').value
        self.kf = INS_GNSS(dim=mode)
        self.kf.setIMUParams(MTI7_params)
        
        # Default params (can be updated via ROS params if needed)
        self.kf.setGNSSParams(np.array([1.0, 1.0, 2.0, 0.1, 0.1, 0.2]))
        self.kf.setAGVParams(lever_arm=(-0.04, 0.23, 0))
        self.kf.setPressParams(15.0)
        
        self.initialized = False
        self.last_imu_time = None
        
        # Buffer for latest sensor data
        self.latest_gnss = None
        self.latest_mag = None
        self.latest_baro = None
        self.latest_vel = None
        
        # Flags to trigger update
        self.new_gnss = False
        self.new_mag = False
        self.new_baro = False
        self.new_vel = False
        self.latest_vel_time = 0.0

        # Pre-allocate static conversion matrices
        # Transformation from ROS FLU to EKF RFU
        self.R_flu2rfu = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ])

        # --- Subscriptions ---
        self.imu_sub = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.imu_callback, 10)
        self.gnss_sub = self.create_subscription(
            NavSatFix, self.get_parameter('gnss_topic').value, self.gnss_callback, 10)
        
        if self.get_parameter('enable_mag').value:
            self.mag_sub = self.create_subscription(
                MagneticField, self.get_parameter('mag_topic').value, self.mag_callback, 10)
        
        if self.get_parameter('enable_baro').value:
            self.baro_sub = self.create_subscription(
                FluidPressure, self.get_parameter('baro_topic').value, self.baro_callback, 10)
        
        if self.get_parameter('enable_agv').value:
            self.vel_sub = self.create_subscription(
                TwistStamped, self.get_parameter('vel_topic').value, self.vel_callback, 10)

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, 'ins/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'ins/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"INS EKF Node started in mode {mode}")

    def gnss_callback(self, msg):
        if msg.status.status < 0: # No fix
            return
        self.latest_gnss = msg
        self.new_gnss = True
        
        if not self.initialized:
            # Initialize position from first valid GNSS
            pos0 = np.array([np.deg2rad(msg.latitude), np.deg2rad(msg.longitude), msg.altitude])
            vel0 = np.zeros(3)
            att0 = np.array([0.0, 0.0, 0.0]) # Will be improved by static alignment or mag
            
            self.kf.setInitStatus(
                pos0, vel0, att0,
                std_pos=5.0, std_vel=0.5, std_ver_ori=np.deg2rad(1.0), std_yaw=np.deg2rad(30.0)
            )
            self.initialized = True
            self.get_logger().info(f"EKF Initialized at LLA: {msg.latitude}, {msg.longitude}, {msg.altitude}")
            # Set reference for ENU conversion
            self.ref_lla_rad = pos0

    def mag_callback(self, msg):
        self.latest_mag = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
        self.new_mag = True

    def baro_callback(self, msg):
        self.latest_baro = msg.fluid_pressure
        self.new_baro = True

    def vel_callback(self, msg):
        # Body velocity (e.g. from encoder)
        self.latest_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        self.latest_vel_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.new_vel = True

    def imu_callback(self, msg):
        if not self.initialized:
            return

        curr_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
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
                pos_qlt = np.sqrt(np.array([self.latest_gnss.position_covariance[0], 
                                            self.latest_gnss.position_covariance[4], 
                                            self.latest_gnss.position_covariance[8]]))
            num_gnss = 10 
        
        mag = self.latest_mag if self.latest_mag is not None else np.zeros(3)
        static_press = self.latest_baro if self.latest_baro is not None else 0
        ref_vel = self.latest_vel if self.latest_vel is not None else np.zeros(3)

        # Handle ZUPT Hor logic (if enabled)
        in_zupt_hor = False
        if self.get_parameter('enable_zupt_hor').value and self.latest_vel is not None:
            # Prevent stale velocity values from triggering false ZUPT locks
            if (curr_t - self.latest_vel_time) < 0.5:
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
        
        # Convert rotation matrix to quaternion
        q = self.rotation_matrix_to_quaternion(R_b2w)
        
        # 3. Create messages
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.get_parameter('map_frame').value
        odom.child_frame_id = self.get_parameter('base_link_frame').value
        
        odom.pose.pose.position.x = pos_enu[0]
        odom.pose.pose.position.y = pos_enu[1]
        odom.pose.pose.position.z = pos_enu[2]
        odom.pose.pose.orientation = q
        
        odom.twist.twist.linear.x = vel_enu[0]
        odom.twist.twist.linear.y = vel_enu[1]
        odom.twist.twist.linear.z = vel_enu[2]
        
        self.odom_pub.publish(odom)
        
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)
        
        if self.get_parameter('publish_tf').value:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = odom.child_frame_id
            t.transform.translation.x = pos_enu[0]
            t.transform.translation.y = pos_enu[1]
            t.transform.translation.z = pos_enu[2]
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    def rotation_matrix_to_quaternion(self, R):
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = InsEkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
