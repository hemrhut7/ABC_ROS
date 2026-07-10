#!/usr/bin/env python3
"""
image_splitter_node — ROS 2 Node for stereoscopic image splitting and compression.

This node captures a wide dual-lens stereo image from a camera (e.g. 2560x720),
optionally rotates it by 180 degrees, and splits it down the middle into separate
left and right images. It publishes the full raw/compressed stereo frame, as well
as the individual left and right raw/compressed frames with camera calibration info.

Optimizations:
- Multithreaded Capture: Utilizes a dedicated background thread for V4L2 camera reading to prevent blocking.
- Lazy Evaluation: Checks subscriber count on each topic before performing CPU-intensive operations (JPEG compression and ROS conversions).

Dependencies:
- rclpy, cv2, cv_bridge, sensor_msgs, std_msgs
"""

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Header
import threading
import time

class ImageSplitterNode(Node):
    def __init__(self):
        super().__init__('image_splitter_node')
        
        # Declare parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('width', 2560)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('rotation_angle', 0)
        self.declare_parameter('jpeg_quality', 80)
        
        # Get parameters
        self.video_device = self.get_parameter('video_device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rotation_angle = self.get_parameter('rotation_angle').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        self.get_logger().info(
            f"Initializing Image Splitter Node:\n"
            f"  Device: /dev/video{self.video_device}\n"
            f"  Target Resolution: {self.width}x{self.height}\n"
            f"  Target FPS: {self.fps}\n"
            f"  Frame ID: {self.frame_id}\n"
            f"  Rotation Angle: {self.rotation_angle}\n"
            f"  JPEG Quality: {self.jpeg_quality}"
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
            
        # Try setting parameters in different orders if needed, checking success
        self.get_logger().info("Configuring camera parameters...")
        
        # Method 1: Set FOURCC, then resolution, then FPS
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Verify
        actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
        
        if actual_fourcc_str != "MJPG":
            self.get_logger().warning(
                f"Failed to set MJPG format directly. Camera returned: '{actual_fourcc_str}'. "
                "Attempting alternative configuration order (Resolution -> FOURCC -> FPS)..."
            )
            # Method 2: Set resolution first, then FOURCC
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            actual_fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
            
        # Query final actual settings
        self.actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(
            f"Camera opened successfully.\n"
            f"  Actual Format: {actual_fourcc_str}\n"
            f"  Actual Resolution: {self.actual_w}x{self.actual_h}\n"
            f"  Actual FPS: {self.actual_fps}"
        )
        
        if actual_fourcc_str != "MJPG":
            self.get_logger().error(
                f"CRITICAL WARNING: Camera is streaming in '{actual_fourcc_str}' format instead of 'MJPG'. "
                "This will cause severe USB bandwidth limitation, dropping framerate to ~3-5 FPS. "
                "Please verify if the camera is connected to a USB 3.0 port and supports MJPG at 2560x720."
            )
            
        # Threading variables for asynchronous capture
        self.frame = None
        self.ret = False
        self.running = True
        self.lock = threading.Lock()
        
        # Start capture thread
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        # Create timer for processing/publishing at target FPS
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def capture_loop(self):
        while self.running and rclpy.ok():
            ret, frame = self.cap.read()
            with self.lock:
                self.ret = ret
                if ret:
                    self.frame = frame
            if not ret:
                time.sleep(0.01)

    def timer_callback(self):
        start_time = time.time()
        
        # Get the latest frame from the capture thread
        with self.lock:
            ret = self.ret
            frame = self.frame
            
        if not ret or frame is None:
            return

        # Check subscription counts to implement lazy evaluation (huge CPU saving)
        stereo_raw_subs = self.stereo_pub.get_subscription_count()
        stereo_comp_subs = self.stereo_compressed_pub.get_subscription_count()
        left_raw_subs = self.left_pub.get_subscription_count()
        left_comp_subs = self.left_compressed_pub.get_subscription_count()
        right_raw_subs = self.right_pub.get_subscription_count()
        right_comp_subs = self.right_compressed_pub.get_subscription_count()
        
        any_stereo = (stereo_raw_subs > 0) or (stereo_comp_subs > 0)
        any_left = (left_raw_subs > 0) or (left_comp_subs > 0)
        any_right = (right_raw_subs > 0) or (right_comp_subs > 0)
        
        if not (any_stereo or any_left or any_right):
            # No active subscribers, skip processing
            return
            
        # Rotate image if required
        if self.rotation_angle == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotation_angle != 0:
            self.get_logger().warning(f"Unsupported rotation angle: {self.rotation_angle}. No rotation applied.", once=True)
            
        timestamp = self.get_clock().now().to_msg()
        
        # Publish Stereo
        if any_stereo:
            try:
                stereo_msg = None
                if stereo_raw_subs > 0:
                    stereo_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    stereo_msg.header.stamp = timestamp
                    stereo_msg.header.frame_id = self.frame_id
                    self.stereo_pub.publish(stereo_msg)
                
                if stereo_comp_subs > 0:
                    ret_c, comp_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                    if ret_c:
                        comp_msg = CompressedImage()
                        comp_msg.header.stamp = timestamp
                        comp_msg.header.frame_id = self.frame_id
                        comp_msg.format = "jpeg"
                        comp_msg.data = comp_data.tobytes()
                        self.stereo_compressed_pub.publish(comp_msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing stereo image: {e}")
                
        # Split left and right images
        if any_left or any_right:
            h, w, _ = frame.shape
            half_w = w // 2
            
            left_frame = frame[:, :half_w] if any_left else None
            right_frame = frame[:, half_w:] if any_right else None
            
            # Left Frame
            if any_left and left_frame is not None:
                try:
                    left_header = Header()
                    left_header.stamp = timestamp
                    left_header.frame_id = 'left_' + self.frame_id
                    
                    if left_raw_subs > 0:
                        left_msg = self.bridge.cv2_to_imgmsg(left_frame, encoding="bgr8")
                        left_msg.header = left_header
                        self.left_pub.publish(left_msg)
                        
                    if left_comp_subs > 0:
                        ret_c, comp_data = cv2.imencode('.jpg', left_frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                        if ret_c:
                            comp_msg = CompressedImage()
                            comp_msg.header = left_header
                            comp_msg.format = "jpeg"
                            comp_msg.data = comp_data.tobytes()
                            self.left_compressed_pub.publish(comp_msg)
                            
                    self.publish_camera_info(self.left_info_pub, left_header, half_w, h)
                except Exception as e:
                    self.get_logger().error(f"Error publishing left image: {e}")
                    
            # Right Frame
            if any_right and right_frame is not None:
                try:
                    right_header = Header()
                    right_header.stamp = timestamp
                    right_header.frame_id = 'right_' + self.frame_id
                    
                    if right_raw_subs > 0:
                        right_msg = self.bridge.cv2_to_imgmsg(right_frame, encoding="bgr8")
                        right_msg.header = right_header
                        self.right_pub.publish(right_msg)
                        
                    if right_comp_subs > 0:
                        ret_c, comp_data = cv2.imencode('.jpg', right_frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                        if ret_c:
                            comp_msg = CompressedImage()
                            comp_msg.header = right_header
                            comp_msg.format = "jpeg"
                            comp_msg.data = comp_data.tobytes()
                            self.right_compressed_pub.publish(comp_msg)
                            
                    self.publish_camera_info(self.right_info_pub, right_header, half_w, h)
                except Exception as e:
                    self.get_logger().error(f"Error publishing right image: {e}")

        # Diagnostic log for execution time (log once every 150 frames to avoid spam)
        duration = time.time() - start_time
        if not hasattr(self, 'frame_count'):
            self.frame_count = 0
        self.frame_count += 1
        if self.frame_count % 150 == 0:
            self.get_logger().info(
                f"Processing loop execution time: {duration*1000:.2f} ms "
                f"(Subscribers - Stereo Raw/Comp: {stereo_raw_subs}/{stereo_comp_subs}, "
                f"Left: {left_raw_subs}/{left_comp_subs}, Right: {right_raw_subs}/{right_comp_subs})"
            )

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
        self.running = False
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
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
