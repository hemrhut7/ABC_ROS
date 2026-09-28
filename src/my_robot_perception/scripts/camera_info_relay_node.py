#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoRelayNode(Node):
    """
    Relays camera_info messages by replacing unrectified R=I, P=K matrices
    with calibrated stereo rectification matrices R0, R1, P0, P1.
    """
    def __init__(self):
        super().__init__('camera_info_relay_node')

        # Calibrated parameters
        self.k_left = [
            642.3066735043554, 0.0, 307.39468172973955,
            0.0, 639.1076963871467, 232.5087613689875,
            0.0, 0.0, 1.0
        ]
        self.d_left = [0.17890018023870716, -0.20487403855793998, -0.001494892594896869, 0.00011300776298145689, 0.0]
        self.r_left = [
             0.99998325,  0.00354823,  0.00457229,
            -0.00353017,  0.99998595, -0.00395345,
            -0.00458626,  0.00393725,  0.99998173
        ]
        self.p_left = [
            693.21520523,   0.0,         314.91036224, 0.0,
              0.0,         693.21520523, 239.14138603, 0.0,
              0.0,           0.0,           1.0,        0.0
        ]

        self.k_right = [
            644.2906010514704, 0.0, 340.17904501005484,
            0.0, 640.7801151585047, 245.5372357813793,
            0.0, 0.0, 1.0
        ]
        self.d_right = [0.18277211689305964, -0.22449649871119265, 0.0019745518498334538, 0.0004427002638666012, 0.0]
        self.r_right = [
             0.99983589,  0.00322815,  0.01782635,
            -0.00329846,  0.99998689,  0.00391613,
            -0.01781348, -0.00397428,  0.99983343
        ]
        self.p_right = [
            693.21520523,   0.0,         314.91036224, -35.99214383,
              0.0,         693.21520523, 239.14138603,   0.0,
              0.0,           0.0,           1.0,          0.0
        ]

        self.pub_left = self.create_publisher(
            CameraInfo, '/camera/left/camera_info_rectified', 10
        )
        self.pub_right = self.create_publisher(
            CameraInfo, '/camera/right/camera_info_rectified', 10
        )

        self.sub_left = self.create_subscription(
            CameraInfo, '/camera/left/camera_info', self.left_cb, 10
        )
        self.sub_right = self.create_subscription(
            CameraInfo, '/camera/right/camera_info', self.right_cb, 10
        )

        self.get_logger().info('CameraInfoRelayNode started: replacing R=I with calibrated stereo rectification matrices.')

    def left_cb(self, msg: CameraInfo):
        out = CameraInfo()
        out.header = msg.header
        out.width = msg.width
        out.height = msg.height
        out.distortion_model = 'plumb_bob'
        out.d = self.d_left
        out.k = self.k_left
        out.r = self.r_left
        out.p = self.p_left
        self.pub_left.publish(out)

    def right_cb(self, msg: CameraInfo):
        out = CameraInfo()
        out.header = msg.header
        out.width = msg.width
        out.height = msg.height
        out.distortion_model = 'plumb_bob'
        out.d = self.d_right
        out.k = self.k_right
        out.r = self.r_right
        out.p = self.p_right
        self.pub_right.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
