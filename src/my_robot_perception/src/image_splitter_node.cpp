#include "my_robot_perception/image_splitter_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sstream>
#include <algorithm>

#include <filesystem>
#include <cstdlib>

namespace my_robot_perception
{

static void setup_clean_gstreamer_environment()
{
  namespace fs = std::filesystem;
  const std::string src_dir = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0";
  const std::string clean_dir = "/tmp/gst_clean_plugins";

  if (fs::exists(src_dir)) {
    std::error_code ec;
    fs::create_directories(clean_dir, ec);
    for (const auto & entry : fs::directory_iterator(src_dir, ec)) {
      if (entry.is_regular_file() || entry.is_symlink()) {
        std::string filename = entry.path().filename().string();
        if (filename.find("nvargus") == std::string::npos) {
          fs::path target = fs::path(clean_dir) / filename;
          if (!fs::exists(target)) {
            fs::create_symlink(entry.path(), target, ec);
          }
        }
      }
    }
    setenv("GST_PLUGIN_SYSTEM_PATH_1_0", "", 1);
    setenv("GST_PLUGIN_PATH_1_0", clean_dir.c_str(), 1);
    setenv("GST_REGISTRY_1_0", "/tmp/gst_clean_registry.bin", 1);
  }
  setenv("GST_PLUGIN_FEATURE_RANK", "nvarguscamerasrc:NONE,nvv4l2camerasrc:NONE", 1);
}

ImageSplitterNode::ImageSplitterNode(const rclcpp::NodeOptions & options)
: Node("image_splitter_node", options),
  running_(false)
{
  setup_clean_gstreamer_environment();

  // Declare parameters
  this->declare_parameter<bool>("use_hardware_decode", true);
  this->declare_parameter<std::string>("hw_decoder_type", "jpegdec");
  this->declare_parameter<int>("video_device", 0);
  this->declare_parameter<bool>("auto_detect_device", true);
  this->declare_parameter<int>("width", 1280);
  this->declare_parameter<int>("height", 480);
  this->declare_parameter<int>("fps", 60);
  this->declare_parameter<std::string>("fourcc", "MJPG");
  this->declare_parameter<std::string>("frame_id", "camera_link");
  this->declare_parameter<std::string>("left_frame_id", "left_camera_optical_frame");
  this->declare_parameter<std::string>("right_frame_id", "right_camera_optical_frame");
  this->declare_parameter<int>("rotation_angle", 180);
  this->declare_parameter<int>("jpeg_quality", 60);

  this->declare_parameter<bool>("publish_bgr", true);
  this->declare_parameter<bool>("publish_mono", true);
  this->declare_parameter<bool>("publish_compressed", true);

  this->declare_parameter<int>("bgr_publish_fps", 15);
  this->declare_parameter<int>("mono_publish_fps", 0);  // 0 = full capture speed (60 FPS)
  this->declare_parameter<int>("compressed_publish_fps", 15);

  this->declare_parameter<bool>("enable_mono_filter", true);
  this->declare_parameter<int>("mono_filter_ksize", 3);
  this->declare_parameter<double>("mono_filter_sigma", 0.5);

  // Get parameters
  use_hardware_decode_ = this->get_parameter("use_hardware_decode").as_bool();
  hw_decoder_type_ = this->get_parameter("hw_decoder_type").as_string();
  video_device_ = this->get_parameter("video_device").as_int();
  auto_detect_device_ = this->get_parameter("auto_detect_device").as_bool();
  width_ = this->get_parameter("width").as_int();
  height_ = this->get_parameter("height").as_int();
  fps_ = this->get_parameter("fps").as_int();
  fourcc_ = this->get_parameter("fourcc").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  left_frame_id_ = this->get_parameter("left_frame_id").as_string();
  right_frame_id_ = this->get_parameter("right_frame_id").as_string();
  rotation_angle_ = this->get_parameter("rotation_angle").as_int();
  jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();

  publish_bgr_ = this->get_parameter("publish_bgr").as_bool();
  publish_mono_ = this->get_parameter("publish_mono").as_bool();
  publish_compressed_ = this->get_parameter("publish_compressed").as_bool();

  bgr_publish_fps_ = this->get_parameter("bgr_publish_fps").as_int();
  mono_publish_fps_ = this->get_parameter("mono_publish_fps").as_int();
  compressed_publish_fps_ = this->get_parameter("compressed_publish_fps").as_int();

  enable_mono_filter_ = this->get_parameter("enable_mono_filter").as_bool();
  mono_filter_ksize_ = this->get_parameter("mono_filter_ksize").as_int();
  if (mono_filter_ksize_ % 2 == 0) {
    mono_filter_ksize_ += 1;
  }
  mono_filter_sigma_ = this->get_parameter("mono_filter_sigma").as_double();

  // Capitalize fourcc
  for (auto & c : fourcc_) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }

  // Calculate intervals in ms
  if (bgr_publish_fps_ > 0) {
    bgr_interval_ms_ = 1000.0 / bgr_publish_fps_;
  }
  if (mono_publish_fps_ > 0) {
    mono_interval_ms_ = 1000.0 / mono_publish_fps_;
  }
  if (compressed_publish_fps_ > 0) {
    compressed_interval_ms_ = 1000.0 / compressed_publish_fps_;
  }

  RCLCPP_INFO(this->get_logger(),
    "====================================================\n"
    "Initializing Jetson Stereo Camera Pipeline (rclcpp Composable Node):\n"
    "  HW Acceleration: %s (Decoder: %s)\n"
    "  Preferred Device: /dev/video%d (Auto Scan: %s)\n"
    "  Target Resolution: %dx%d @ %d FPS (FOURCC: %s)\n"
    "  Rotation Angle: %d deg\n"
    "  Streams Config:\n"
    "    - BGR8 Stream: %s (Rate: %s)\n"
    "    - Mono8 Stream: %s (Rate: %s, Gaussian Filter: %s [k=%d, s=%.2f])\n"
    "    - Compressed Stream: %s (Rate: %s, Quality: %d)\n"
    "====================================================",
    use_hardware_decode_ ? "Enabled (Jetson NVDEC + VIC)" : "Disabled (Software V4L2)",
    hw_decoder_type_.c_str(),
    video_device_, auto_detect_device_ ? "Enabled" : "Disabled",
    width_, height_, fps_, fourcc_.c_str(),
    rotation_angle_,
    publish_bgr_ ? "Enabled" : "Disabled",
    bgr_publish_fps_ > 0 ? (std::to_string(bgr_publish_fps_) + " Hz").c_str() : "Full Speed (60 Hz)",
    publish_mono_ ? "Enabled" : "Disabled",
    mono_publish_fps_ > 0 ? (std::to_string(mono_publish_fps_) + " Hz").c_str() : "Full Speed (60 Hz)",
    enable_mono_filter_ ? "ON" : "OFF", mono_filter_ksize_, mono_filter_sigma_,
    publish_compressed_ ? "Enabled" : "Disabled",
    compressed_publish_fps_ > 0 ? (std::to_string(compressed_publish_fps_) + " Hz").c_str() : "Full Speed (60 Hz)",
    jpeg_quality_);

  // Initialize QoS
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5))
    .reliable()
    .durability_volatile();

  // Create publishers
  if (publish_bgr_) {
    left_bgr_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/left/image_raw", sensor_qos);
    right_bgr_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/right/image_raw", sensor_qos);
  }

  if (publish_mono_) {
    left_mono_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/left/image_mono", sensor_qos);
    right_mono_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/right/image_mono", sensor_qos);
  }

  if (publish_compressed_) {
    left_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/left/image_raw/compressed", sensor_qos);
    right_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/right/image_raw/compressed", sensor_qos);
  }

  left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/left/camera_info", sensor_qos);
  right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/right/camera_info", sensor_qos);

  // Open camera
  int active_device = open_camera(video_device_, auto_detect_device_);
  if (active_device < 0 || !cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to open video capture device (Preferred /dev/video%d, Auto-scan: %s).",
      video_device_, auto_detect_device_ ? "true" : "false");
    throw std::runtime_error("Could not open video device");
  }
  video_device_ = active_device;

  // Start background capture thread
  running_ = true;
  capture_thread_ = std::thread(&ImageSplitterNode::capture_loop, this);
}

ImageSplitterNode::~ImageSplitterNode()
{
  running_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (cap_.isOpened()) {
    cap_.release();
  }
}

std::string ImageSplitterNode::build_gstreamer_pipeline(
  int dev_idx, int w, int h, int fps_val, int rot_angle, const std::string & decoder_type)
{
  int flip_method = 0;
  if (rot_angle == 180) {
    flip_method = 2;
  } else if (rot_angle == 90) {
    flip_method = 3;
  } else if (rot_angle == 270) {
    flip_method = 1;
  }

  std::ostringstream ss;
  if (decoder_type == "nvv4l2decoder") {
    ss << "v4l2src device=/dev/video" << dev_idx
       << " ! image/jpeg, width=" << w << ", height=" << h << ", framerate=" << fps_val << "/1"
       << " ! jpegparse ! nvv4l2decoder mjpeg=1 ! video/x-raw(memory:NVMM)"
       << " ! nvvidconv flip-method=" << flip_method
       << " ! video/x-raw, format=BGRx"
       << " ! videoconvert"
       << " ! video/x-raw, format=BGR"
       << " ! appsink drop=1 max-buffers=2";
  } else if (decoder_type == "nvjpegdec") {
    ss << "v4l2src device=/dev/video" << dev_idx
       << " ! image/jpeg, width=" << w << ", height=" << h << ", framerate=" << fps_val << "/1"
       << " ! jpegparse ! nvjpegdec ! video/x-raw(memory:NVMM)"
       << " ! nvvidconv flip-method=" << flip_method
       << " ! video/x-raw, format=BGRx"
       << " ! videoconvert"
       << " ! video/x-raw, format=BGR"
       << " ! appsink drop=1 max-buffers=2";
  } else {
    // Default & reliable pipeline for USB UVC MJPEG on Jetson Orin Nano:
    // SIMD multi-threaded JPEG decode + Jetson VIC (nvvidconv) for 180 flip & colorspace conversion
    ss << "v4l2src device=/dev/video" << dev_idx
       << " ! image/jpeg, width=" << w << ", height=" << h << ", framerate=" << fps_val << "/1"
       << " ! jpegdec"
       << " ! nvvidconv flip-method=" << flip_method
       << " ! video/x-raw, format=BGRx"
       << " ! videoconvert"
       << " ! video/x-raw, format=BGR"
       << " ! appsink drop=1 max-buffers=2";
  }

  return ss.str();
}

bool ImageSplitterNode::is_v4l2_capture_device(int index, std::string & card_name)
{
  std::string dev_path = "/dev/video" + std::to_string(index);
  int fd = open(dev_path.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd < 0) {
    return false;
  }
  struct v4l2_capability cap;
  if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
    close(fd);
    return false;
  }
  close(fd);

  uint32_t caps = cap.capabilities;
  if (caps & V4L2_CAP_DEVICE_CAPS) {
    caps = cap.device_caps;
  }
  if (!(caps & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))) {
    return false;
  }

  card_name = reinterpret_cast<const char *>(cap.card);
  return true;
}

int ImageSplitterNode::open_camera(int preferred_device, bool auto_scan)
{
  std::string card_name;

  // 1. Attempt hardware GStreamer pipeline if use_hardware_decode_ is true
  if (use_hardware_decode_) {
    if (preferred_device >= 0 && is_v4l2_capture_device(preferred_device, card_name)) {
      std::string gst_pipe = build_gstreamer_pipeline(
        preferred_device, width_, height_, fps_, rotation_angle_, hw_decoder_type_);
      RCLCPP_INFO(this->get_logger(),
        "Attempting Jetson HW decode on /dev/video%d (%s) with pipeline:\n  %s",
        preferred_device, card_name.c_str(), gst_pipe.c_str());

      if (cap_.open(gst_pipe, cv::CAP_GSTREAMER)) {
        hardware_decode_active_ = true;
        hardware_rotated_ = (rotation_angle_ != 0);
        // Warmup camera sensor to stabilize Auto-Exposure / Auto-White-Balance
        cv::Mat dummy_f;
        for (int i = 0; i < 5; ++i) {
          cap_.read(dummy_f);
        }
        RCLCPP_INFO(this->get_logger(), "Successfully opened hardware decode pipeline on /dev/video%d (%s)!",
          preferred_device, card_name.c_str());
        return preferred_device;
      }
      cap_.release();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      RCLCPP_WARN(this->get_logger(), "Failed to open HW pipeline on preferred /dev/video%d.", preferred_device);
    }

    if (auto_scan) {
      RCLCPP_INFO(this->get_logger(), "Scanning video devices for Jetson HW decode compatibility...");
      for (int dev_idx = 0; dev_idx < 64; ++dev_idx) {
        if (dev_idx == preferred_device || !is_v4l2_capture_device(dev_idx, card_name)) {
          continue;
        }
        std::string gst_pipe = build_gstreamer_pipeline(
          dev_idx, width_, height_, fps_, rotation_angle_, hw_decoder_type_);
        if (cap_.open(gst_pipe, cv::CAP_GSTREAMER)) {
          hardware_decode_active_ = true;
          hardware_rotated_ = (rotation_angle_ != 0);
          RCLCPP_INFO(this->get_logger(), "Auto-scan found and opened HW decode pipeline on /dev/video%d (%s)!",
            dev_idx, card_name.c_str());
          return dev_idx;
        }
        cap_.release();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    RCLCPP_WARN(this->get_logger(),
      "Jetson HW decode failed or not available on any scanned device. Falling back to Software V4L2 decode...");
  }

  // 2. Fallback / Software V4L2 capture
  hardware_decode_active_ = false;
  hardware_rotated_ = false;
  cap_.release();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto configure_v4l2 = [this](int dev) -> bool {
    cap_.release();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (cap_.open(dev, cv::CAP_V4L2)) {
      if (fourcc_.length() == 4) {
        int fourcc_code = cv::VideoWriter::fourcc(fourcc_[0], fourcc_[1], fourcc_[2], fourcc_[3]);
        cap_.set(cv::CAP_PROP_FOURCC, fourcc_code);
      }
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
      cap_.set(cv::CAP_PROP_FPS, fps_);
      return true;
    }
    return false;
  };

  if (preferred_device >= 0 && is_v4l2_capture_device(preferred_device, card_name)) {
    if (configure_v4l2(preferred_device)) {
      RCLCPP_INFO(this->get_logger(), "Opened camera via Software V4L2 at /dev/video%d (%s)",
        preferred_device, card_name.c_str());
      return preferred_device;
    }
  }

  if (auto_scan) {
    for (int dev_idx = 0; dev_idx < 64; ++dev_idx) {
      if (dev_idx == preferred_device || !is_v4l2_capture_device(dev_idx, card_name)) {
        continue;
      }
      if (configure_v4l2(dev_idx)) {
        RCLCPP_INFO(this->get_logger(), "Auto-scan opened camera via Software V4L2 at /dev/video%d (%s)",
          dev_idx, card_name.c_str());
        return dev_idx;
      }
    }
  }

  return -1;
}

void ImageSplitterNode::capture_loop()
{
  cv::Mat frame;
  while (running_ && rclcpp::ok()) {
    if (!cap_.read(frame) || frame.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    process_and_publish_frame(frame);
  }
}

void ImageSplitterNode::process_and_publish_frame(cv::Mat & frame)
{
  auto start_time = std::chrono::steady_clock::now();

  static bool first_frame_verified = false;
  if (!first_frame_verified && !frame.empty()) {
    cv::Scalar mean_val = cv::mean(frame);
    double avg_brightness = (mean_val[0] + mean_val[1] + mean_val[2]) / 3.0;
    RCLCPP_INFO(this->get_logger(),
      "Stream content verified: Size %dx%d, Brightness Mean: %.1f (B:%.1f, G:%.1f, R:%.1f)",
      frame.cols, frame.rows, avg_brightness, mean_val[0], mean_val[1], mean_val[2]);
    first_frame_verified = true;
  }

  size_t left_bgr_subs = left_bgr_pub_ ? left_bgr_pub_->get_subscription_count() : 0;
  size_t right_bgr_subs = right_bgr_pub_ ? right_bgr_pub_->get_subscription_count() : 0;

  size_t left_mono_subs = left_mono_pub_ ? left_mono_pub_->get_subscription_count() : 0;
  size_t right_mono_subs = right_mono_pub_ ? right_mono_pub_->get_subscription_count() : 0;

  size_t left_comp_subs = left_compressed_pub_ ? left_compressed_pub_->get_subscription_count() : 0;
  size_t right_comp_subs = right_compressed_pub_ ? right_compressed_pub_->get_subscription_count() : 0;

  size_t left_info_subs = left_info_pub_->get_subscription_count();
  size_t right_info_subs = right_info_pub_->get_subscription_count();

  // BGR publish check (15 Hz downsampled by default)
  bool should_publish_bgr = publish_bgr_ && (left_bgr_subs > 0 || right_bgr_subs > 0);
  if (should_publish_bgr && bgr_publish_fps_ > 0) {
    double elapsed_bgr_ms = std::chrono::duration<double, std::milli>(
      start_time - last_bgr_publish_time_).count();
    if (elapsed_bgr_ms < bgr_interval_ms_) {
      should_publish_bgr = false;
    } else {
      last_bgr_publish_time_ = start_time;
    }
  }

  // Mono publish check (Full 60 Hz by default)
  bool should_publish_mono = publish_mono_ && (left_mono_subs > 0 || right_mono_subs > 0);
  if (should_publish_mono && mono_publish_fps_ > 0) {
    double elapsed_mono_ms = std::chrono::duration<double, std::milli>(
      start_time - last_mono_publish_time_).count();
    if (elapsed_mono_ms < mono_interval_ms_) {
      should_publish_mono = false;
    } else {
      last_mono_publish_time_ = start_time;
    }
  }

  // Compressed publish check (15 Hz by default)
  bool should_publish_comp = publish_compressed_ && (left_comp_subs > 0 || right_comp_subs > 0);
  if (should_publish_comp && compressed_publish_fps_ > 0) {
    double elapsed_comp_ms = std::chrono::duration<double, std::milli>(
      start_time - last_compressed_publish_time_).count();
    if (elapsed_comp_ms < compressed_interval_ms_) {
      should_publish_comp = false;
    } else {
      last_compressed_publish_time_ = start_time;
    }
  }

  bool should_publish_info = (left_info_subs > 0 || right_info_subs > 0);

  // Skip frame processing entirely if no consumers
  if (!should_publish_bgr && !should_publish_mono && !should_publish_comp && !should_publish_info) {
    return;
  }

  // If software decode was used and rotation is needed, perform software rotation
  if (!hardware_rotated_) {
    if (rotation_angle_ == 180) {
      cv::rotate(frame, frame, cv::ROTATE_180);
    } else if (rotation_angle_ != 0) {
      RCLCPP_WARN_ONCE(this->get_logger(),
        "Unsupported software rotation angle: %d. No rotation applied.", rotation_angle_);
    }
  }

  // Unified precise timestamp for all outputs in this frame
  rclcpp::Time timestamp = this->get_clock()->now();

  int h = frame.rows;
  int w = frame.cols;
  int half_w = w / 2;

  // Split stereo image into left and right ROIs (clone to ensure continuous memory layout for cv_bridge)
  cv::Mat left_frame = frame(cv::Rect(0, 0, half_w, h)).clone();
  cv::Mat right_frame = frame(cv::Rect(half_w, 0, half_w, h)).clone();

  // Common Headers
  std_msgs::msg::Header left_header;
  left_header.stamp = timestamp;
  left_header.frame_id = left_frame_id_;

  std_msgs::msg::Header right_header;
  right_header.stamp = timestamp;
  right_header.frame_id = right_frame_id_;

  // 1. Publish BGR8 stream (Throttled, e.g. 15 Hz)
  if (should_publish_bgr) {
    if (left_bgr_subs > 0) {
      auto left_bgr_msg = cv_bridge::CvImage(left_header, "bgr8", left_frame).toImageMsg();
      left_bgr_pub_->publish(*left_bgr_msg);
    }
    if (right_bgr_subs > 0) {
      auto right_bgr_msg = cv_bridge::CvImage(right_header, "bgr8", right_frame).toImageMsg();
      right_bgr_pub_->publish(*right_bgr_msg);
    }
  }

  // 2. Publish Mono8 stream (Full speed 60 Hz + Gaussian de-noising)
  if (should_publish_mono) {
    if (left_mono_subs > 0) {
      cv::Mat left_gray;
      cv::cvtColor(left_frame, left_gray, cv::COLOR_BGR2GRAY);
      if (enable_mono_filter_) {
        cv::GaussianBlur(left_gray, left_gray,
          cv::Size(mono_filter_ksize_, mono_filter_ksize_),
          mono_filter_sigma_, mono_filter_sigma_);
      }
      auto left_mono_msg = cv_bridge::CvImage(left_header, "mono8", left_gray).toImageMsg();
      left_mono_pub_->publish(*left_mono_msg);
    }
    if (right_mono_subs > 0) {
      cv::Mat right_gray;
      cv::cvtColor(right_frame, right_gray, cv::COLOR_BGR2GRAY);
      if (enable_mono_filter_) {
        cv::GaussianBlur(right_gray, right_gray,
          cv::Size(mono_filter_ksize_, mono_filter_ksize_),
          mono_filter_sigma_, mono_filter_sigma_);
      }
      auto right_mono_msg = cv_bridge::CvImage(right_header, "mono8", right_gray).toImageMsg();
      right_mono_pub_->publish(*right_mono_msg);
    }
  }

  // 3. Publish Compressed stream (15 Hz)
  if (should_publish_comp) {
    std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (left_comp_subs > 0) {
      std::vector<uchar> buf;
      if (cv::imencode(".jpg", left_frame, buf, encode_params)) {
        sensor_msgs::msg::CompressedImage comp_msg;
        comp_msg.header = left_header;
        comp_msg.format = "jpeg";
        comp_msg.data = std::move(buf);
        left_compressed_pub_->publish(comp_msg);
      }
    }
    if (right_comp_subs > 0) {
      std::vector<uchar> buf;
      if (cv::imencode(".jpg", right_frame, buf, encode_params)) {
        sensor_msgs::msg::CompressedImage comp_msg;
        comp_msg.header = right_header;
        comp_msg.format = "jpeg";
        comp_msg.data = std::move(buf);
        right_compressed_pub_->publish(comp_msg);
      }
    }
  }

  // 4. Publish Camera Info (Synchronized with images)
  publish_camera_info(left_info_pub_, left_header, true, half_w, h);
  publish_camera_info(right_info_pub_, right_header, false, half_w, h);

  // Log subscriber changes
  if (left_bgr_subs != prev_left_bgr_subs_ || right_bgr_subs != prev_right_bgr_subs_ ||
      left_mono_subs != prev_left_mono_subs_ || right_mono_subs != prev_right_mono_subs_ ||
      left_comp_subs != prev_left_comp_subs_ || right_comp_subs != prev_right_comp_subs_)
  {
    RCLCPP_INFO(this->get_logger(),
      "Subscribers active -> BGR: [L:%zu, R:%zu] | Mono: [L:%zu, R:%zu] | Comp: [L:%zu, R:%zu]",
      left_bgr_subs, right_bgr_subs, left_mono_subs, right_mono_subs, left_comp_subs, right_comp_subs);
    prev_left_bgr_subs_ = left_bgr_subs;
    prev_right_bgr_subs_ = right_bgr_subs;
    prev_left_mono_subs_ = left_mono_subs;
    prev_right_mono_subs_ = right_mono_subs;
    prev_left_comp_subs_ = left_comp_subs;
    prev_right_comp_subs_ = right_comp_subs;
  }
}

void ImageSplitterNode::publish_camera_info(
  const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr & publisher,
  const std_msgs::msg::Header & header,
  bool is_left,
  int width,
  int height)
{
  sensor_msgs::msg::CameraInfo info_msg;
  info_msg.header = header;
  info_msg.width = width;
  info_msg.height = height;
  info_msg.distortion_model = "plumb_bob";

  if (is_left) {
    // Left Camera (cam0) calibration
    info_msg.d = {0.17890018023870716, -0.20487403855793998, -0.001494892594896869, 0.00011300776298145689, 0.0};
    info_msg.k = {
      642.3066735043554, 0.0, 307.39468172973955,
      0.0, 639.1076963871467, 232.5087613689875,
      0.0, 0.0, 1.0
    };
    info_msg.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    info_msg.p = {
      642.3066735043554, 0.0, 307.39468172973955, 0.0,
      0.0, 639.1076963871467, 232.5087613689875, 0.0,
      0.0, 0.0, 1.0, 0.0
    };
  } else {
    // Right Camera (cam1) calibration (Baseline from Kalibr: 51.912mm = 0.05191207112027276m)
    info_msg.d = {0.18277211689305964, -0.22449649871119265, 0.0019745518498334538, 0.0004427002638666012, 0.0};
    info_msg.k = {
      644.2906010514704, 0.0, 340.17904501005484,
      0.0, 640.7801151585047, 245.5372357813793,
      0.0, 0.0, 1.0
    };
    info_msg.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    info_msg.p = {
      644.2906010514704, 0.0, 340.17904501005484, -644.2906010514704 * 0.05191207112027276,
      0.0, 640.7801151585047, 245.5372357813793, 0.0,
      0.0, 0.0, 1.0, 0.0
    };
  }
  publisher->publish(info_msg);
}

}  // namespace my_robot_perception

RCLCPP_COMPONENTS_REGISTER_NODE(my_robot_perception::ImageSplitterNode)
