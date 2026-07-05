#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Header

class ImageSplitterNode(Node):
    def __init__(self):
        super().__init__('image_splitter_node')
        
        # Declare parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('width', 2560)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_link')
        
        # Get parameters
        self.video_device = self.get_parameter('video_device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        
        self.get_logger().info(
            f"Initializing Image Splitter Node:\n"
            f"  Device: /dev/video{self.video_device}\n"
            f"  Target Resolution: {self.width}x{self.height}\n"
            f"  Target FPS: {self.fps}\n"
            f"  Frame ID: {self.frame_id}"
        )
        
        # Initialize publishers
        self.stereo_pub = self.create_publisher(Image, 'camera/stereo/image_raw', 10)
        self.left_pub = self.create_publisher(Image, 'camera/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, 'camera/right/image_raw', 10)
        
        self.stereo_compressed_pub = self.create_publisher(CompressedImage, 'camera/stereo/image_raw/compressed', 10)
        self.left_compressed_pub = self.create_publisher(CompressedImage, 'camera/left/image_raw/compressed', 10)
        self.right_compressed_pub = self.create_publisher(CompressedImage, 'camera/right/image_raw/compressed', 10)
        
        self.left_info_pub = self.create_publisher(CameraInfo, 'camera/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, 'camera/right/camera_info', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize camera capture using CAP_V4L2 backend
        self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video device /dev/video{self.video_device}")
            raise RuntimeError(f"Could not open device /dev/video{self.video_device}")
            
        # Configure MJPEG format for high FPS streaming
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Query actual settings from the camera
        self.actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(
            f"Camera opened successfully.\n"
            f"  Actual Resolution: {self.actual_w}x{self.actual_h}\n"
            f"  Actual FPS: {self.actual_fps}"
        )
        
        # Create timer for capture
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to grab frame from camera.")
            return
            
        timestamp = self.get_clock().now().to_msg()
        
        # 1. Full Stereo Frame
        try:
            stereo_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            stereo_msg.header.stamp = timestamp
            stereo_msg.header.frame_id = self.frame_id
            self.stereo_pub.publish(stereo_msg)
            
            # Stereo Compressed
            ret_c, comp_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret_c:
                comp_msg = CompressedImage()
                comp_msg.header = stereo_msg.header
                comp_msg.format = "jpeg"
                comp_msg.data = comp_data.tobytes()
                self.stereo_compressed_pub.publish(comp_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing stereo image: {e}")
            
        # Split left and right images
        h, w, _ = frame.shape
        half_w = w // 2
        
        left_frame = frame[:, :half_w]
        right_frame = frame[:, half_w:]
        
        # 2. Left Frame
        try:
            left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding="bgr8")
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = 'left_' + self.frame_id
            self.left_pub.publish(left_msg)
            
            # Left Compressed
            ret_c, comp_data = cv2.imencode('.jpg', left_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret_c:
                comp_msg = CompressedImage()
                comp_msg.header = left_msg.header
                comp_msg.format = "jpeg"
                comp_msg.data = comp_data.tobytes()
                self.left_compressed_pub.publish(comp_msg)
                
            # Publish Left CameraInfo
            self.publish_camera_info(self.left_info_pub, left_msg.header, half_w, h)
        except Exception as e:
            self.get_logger().error(f"Error publishing left image: {e}")
            
        # 3. Right Frame
        try:
            right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding="bgr8")
            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = 'right_' + self.frame_id
            self.right_pub.publish(right_msg)
            
            # Right Compressed
            ret_c, comp_data = cv2.imencode('.jpg', right_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret_c:
                comp_msg = CompressedImage()
                comp_msg.header = right_msg.header
                comp_msg.format = "jpeg"
                comp_msg.data = comp_data.tobytes()
                self.right_compressed_pub.publish(comp_msg)
                
            # Publish Right CameraInfo
            self.publish_camera_info(self.right_info_pub, right_msg.header, half_w, h)
        except Exception as e:
            self.get_logger().error(f"Error publishing right image: {e}")

    def publish_camera_info(self, publisher, header, width, height):
        info_msg = CameraInfo()
        info_msg.header = header
        info_msg.width = width
        info_msg.height = height
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.k = [1.0, 0.0, float(width / 2.0),
                      0.0, 1.0, float(height / 2.0),
                      0.0, 0.0, 1.0]
        info_msg.r = [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
        info_msg.p = [1.0, 0.0, float(width / 2.0), 0.0,
                      0.0, 1.0, float(height / 2.0), 0.0,
                      0.0, 0.0, 1.0, 0.0]
        publisher.publish(info_msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ImageSplitterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
