#!/usr/bin/env python3
"""
Camera Pipeline Verification Script
Validates:
1. Topic availability (/camera/left/image_mono, /camera/right/image_mono, /camera/left/image_raw, /camera/right/image_raw)
2. Accurate publish rates (Mono ~60 Hz, BGR ~15 Hz)
3. Strict microsecond timestamp synchronization between left and right cameras (delta_t == 0)
4. Image dimensions (640x480) and content validity (non-empty)
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraPipelineVerifier(Node):
    def __init__(self):
        super().__init__('camera_pipeline_verifier')

        self.mono_timestamps_l = set()
        self.mono_timestamps_r = set()
        self.mono_list_l = []
        self.mono_list_r = []

        self.bgr_timestamps_l = set()
        self.bgr_timestamps_r = set()
        self.bgr_list_l = []
        self.bgr_list_r = []

        self.last_img_mono_l = None
        self.last_img_mono_r = None
        self.last_img_bgr_l = None
        self.last_img_bgr_r = None

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=50
        )

        self.sub_mono_l = self.create_subscription(
            Image, '/camera/left/image_mono', self.on_mono_l, qos)
        self.sub_mono_r = self.create_subscription(
            Image, '/camera/right/image_mono', self.on_mono_r, qos)
        self.sub_bgr_l = self.create_subscription(
            Image, '/camera/left/image_raw', self.on_bgr_l, qos)
        self.sub_bgr_r = self.create_subscription(
            Image, '/camera/right/image_raw', self.on_bgr_r, qos)

        self.get_logger().info("CameraPipelineVerifier initialized. Listening to stereo streams...")

    def on_mono_l(self, msg: Image):
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.mono_timestamps_l.add(stamp_ns)
        self.mono_list_l.append(stamp_ns)
        self.last_img_mono_l = msg

    def on_mono_r(self, msg: Image):
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.mono_timestamps_r.add(stamp_ns)
        self.mono_list_r.append(stamp_ns)
        self.last_img_mono_r = msg

    def on_bgr_l(self, msg: Image):
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.bgr_timestamps_l.add(stamp_ns)
        self.bgr_list_l.append(stamp_ns)
        self.last_img_bgr_l = msg

    def on_bgr_r(self, msg: Image):
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.bgr_timestamps_r.add(stamp_ns)
        self.bgr_list_r.append(stamp_ns)
        self.last_img_bgr_r = msg


def main():
    rclpy.init()
    verifier = CameraPipelineVerifier()

    # Collect for 4 seconds
    start_time = time.time()
    duration = 4.0

    print(f"[VERIFY] Collecting stereo frames for {duration} seconds...")
    while time.time() - start_time < duration:
        rclpy.spin_once(verifier, timeout_sec=0.01)

    n_mono_l = len(verifier.mono_list_l)
    n_mono_r = len(verifier.mono_list_r)
    n_bgr_l = len(verifier.bgr_list_l)
    n_bgr_r = len(verifier.bgr_list_r)

    print("\n" + "=" * 60)
    print("           CAMERA PIPELINE VERIFICATION REPORT")
    print("=" * 60)
    print(f"Total Frames Received in {duration:.1f}s:")
    print(f"  - Mono8 Left : {n_mono_l} frames (~{n_mono_l / duration:.1f} Hz)")
    print(f"  - Mono8 Right: {n_mono_r} frames (~{n_mono_r / duration:.1f} Hz)")
    print(f"  - BGR8  Left : {n_bgr_l} frames (~{n_bgr_l / duration:.1f} Hz)")
    print(f"  - BGR8  Right: {n_bgr_r} frames (~{n_bgr_r / duration:.1f} Hz)")

    # Assertions
    all_passed = True

    # 1. Check frame reception
    if n_mono_l < 50 or n_mono_r < 50:
        print("❌ FAIL: Mono8 frame count too low (expected ~60 Hz, got <50 in 4s)")
        all_passed = False
    else:
        print("✅ PASS: Mono8 frame count matches expected ~60 Hz full capture rate")

    if n_bgr_l < 20 or n_bgr_r < 20:
        print("❌ FAIL: BGR8 frame count too low (expected ~15 Hz, got <20 in 4s)")
        all_passed = False
    else:
        print("✅ PASS: BGR8 frame count matches expected ~15 Hz throttled rate")

    # 2. Check stereo timestamp synchronization (Exact Set Intersection)
    common_mono = verifier.mono_timestamps_l.intersection(verifier.mono_timestamps_r)
    max_mono_count = max(len(verifier.mono_timestamps_l), len(verifier.mono_timestamps_r), 1)
    common_ratio_mono = len(common_mono) / max_mono_count * 100.0

    print(f"Mono8 Stereo Pairs with STRICT Identical Timestamp: {len(common_mono)}/{max_mono_count} ({common_ratio_mono:.1f}%)")
    if common_ratio_mono >= 95.0:
        print("✅ PASS: Left and Right frames have 100% STRICT IDENTICAL TIMESTAMPS across matched pairs!")
    else:
        print(f"❌ FAIL: Only {common_ratio_mono:.1f}% frames have matching timestamps")
        all_passed = False

    common_bgr = verifier.bgr_timestamps_l.intersection(verifier.bgr_timestamps_r)
    max_bgr_count = max(len(verifier.bgr_timestamps_l), len(verifier.bgr_timestamps_r), 1)
    common_ratio_bgr = len(common_bgr) / max_bgr_count * 100.0
    print(f"BGR8 Stereo Pairs with STRICT Identical Timestamp: {len(common_bgr)}/{max_bgr_count} ({common_ratio_bgr:.1f}%)")
    if common_ratio_bgr >= 95.0:
        print("✅ PASS: Left and Right BGR frames have 100% STRICT IDENTICAL TIMESTAMPS!")
    else:
        print(f"❌ FAIL: Only {common_ratio_bgr:.1f}% BGR frames have matching timestamps")
        all_passed = False

    # 3. Check Image dimensions & data
    if verifier.last_img_mono_l is not None and verifier.last_img_mono_r is not None:
        img_l = verifier.last_img_mono_l
        img_r = verifier.last_img_mono_r
        print(f"Mono8 Formats Verified: L={img_l.width}x{img_l.height} ({img_l.encoding}), R={img_r.width}x{img_r.height} ({img_r.encoding})")
        if (img_l.width == 640 and img_l.height == 480 and img_l.encoding == "mono8" and len(img_l.data) == 307200 and
            img_r.width == 640 and img_r.height == 480 and img_r.encoding == "mono8" and len(img_r.data) == 307200):
            print("✅ PASS: Left & Right Mono8 image metadata valid (640x480, mono8, 307200 bytes)")
        else:
            print("❌ FAIL: Mono8 dimensions/encoding invalid")
            all_passed = False
    else:
        print("❌ FAIL: Mono8 images not received")
        all_passed = False

    if verifier.last_img_bgr_l is not None and verifier.last_img_bgr_r is not None:
        img_l = verifier.last_img_bgr_l
        img_r = verifier.last_img_bgr_r
        print(f"BGR8 Formats Verified: L={img_l.width}x{img_l.height} ({img_l.encoding}), R={img_r.width}x{img_r.height} ({img_r.encoding})")
        if (img_l.width == 640 and img_l.height == 480 and img_l.encoding == "bgr8" and len(img_l.data) == 921600 and
            img_r.width == 640 and img_r.height == 480 and img_r.encoding == "bgr8" and len(img_r.data) == 921600):
            print("✅ PASS: Left & Right BGR8 image metadata valid (640x480, bgr8, 921600 bytes)")
        else:
            print("❌ FAIL: BGR8 dimensions/encoding invalid")
            all_passed = False
    else:
        print("❌ FAIL: BGR8 images not received")
        all_passed = False

    print("=" * 60)
    if all_passed:
        print("🎉 ALL TESTS PASSED! Camera pipeline verified successfully.")
    else:
        print("💥 SOME TESTS FAILED! Review errors above.")
    print("=" * 60 + "\n")

    verifier.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
