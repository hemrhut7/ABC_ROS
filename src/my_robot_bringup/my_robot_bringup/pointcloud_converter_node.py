import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation
import struct

class PointCloudConverterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_converter_node')
        
        self.latest_odom = None

        # Subscriptions
        self.sub_odom = self.create_subscription(
            Odometry, '/vins_estimator/odometry', self.odom_callback, 10)
        self.sub_pc = self.create_subscription(
            PointCloud, '/vins_estimator/point_cloud', self.pc_callback, 10)

        # Publishers
        self.pub_pc2 = self.create_publisher(
            PointCloud2, '/vins_estimator/point_cloud2', 10)

        self.get_logger().info("PointCloud -> PointCloud2 Converter Node (World to Base_link) started.")

    def odom_callback(self, msg):
        self.latest_odom = msg

    def convert_pc1_to_pc2(self, msg_pc1):
        msg_pc2 = PointCloud2()
        msg_pc2.header = msg_pc1.header
        msg_pc2.header.frame_id = 'base_link'

        # Get current transform from world to base_link if available
        if self.latest_odom is not None:
            pos = self.latest_odom.pose.pose.position
            ori = self.latest_odom.pose.pose.orientation
            t_w_b = np.array([pos.x, pos.y, pos.z])
            r_w_b = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w]).as_matrix()
            r_b_w = r_w_b.T
        else:
            t_w_b = np.zeros(3)
            r_b_w = np.eye(3)

        msg_pc2.height = 1
        msg_pc2.width = len(msg_pc1.points)
        msg_pc2.is_dense = False
        msg_pc2.is_bigendian = False

        msg_pc2.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg_pc2.point_step = 12
        msg_pc2.row_step = 12 * msg_pc2.width

        buffer = bytearray()
        for pt in msg_pc1.points:
            p_w = np.array([pt.x, pt.y, pt.z])
            p_b = r_b_w @ (p_w - t_w_b)
            buffer.extend(struct.pack('fff', float(p_b[0]), float(p_b[1]), float(p_b[2])))

        msg_pc2.data = bytes(buffer)
        return msg_pc2

    def pc_callback(self, msg):
        msg_pc2 = self.convert_pc1_to_pc2(msg)
        self.pub_pc2.publish(msg_pc2)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
