#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <cctype>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

class ImageSplitterNode : public rclcpp::Node
{
public:
  ImageSplitterNode()
  : Node("image_splitter_node"),
    running_(false)
  {
    // Declare parameters
    this->declare_parameter<int>("video_device", 0);
    this->declare_parameter<bool>("auto_detect_device", true);
    this->declare_parameter<int>("width", 1280);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<int>("fps", 60);
    this->declare_parameter<std::string>("fourcc", "MJPG");
    this->declare_parameter<std::string>("frame_id", "camera_link");
    this->declare_parameter<int>("rotation_angle", 180);
    this->declare_parameter<int>("jpeg_quality", 60);
    this->declare_parameter<std::string>("encoding", "mono8");
    this->declare_parameter<int>("publish_fps", 0);  // 0 = same as capture fps (no throttle) for raw image
    this->declare_parameter<int>("compressed_publish_fps", 15);  // Throttle compressed image for remote viewing

    // Get parameters
    video_device_ = this->get_parameter("video_device").as_int();
    auto_detect_device_ = this->get_parameter("auto_detect_device").as_bool();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    fps_ = this->get_parameter("fps").as_int();
    fourcc_ = this->get_parameter("fourcc").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    rotation_angle_ = this->get_parameter("rotation_angle").as_int();
    jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
    encoding_ = this->get_parameter("encoding").as_string();
    publish_fps_ = this->get_parameter("publish_fps").as_int();
    compressed_publish_fps_ = this->get_parameter("compressed_publish_fps").as_int();

    for (auto & c : encoding_) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (encoding_ != "mono8" && encoding_ != "rgb8" && encoding_ != "bgr8") {
      RCLCPP_WARN(this->get_logger(), "Unknown encoding '%s', defaulting to 'mono8'", encoding_.c_str());
      encoding_ = "mono8";
    }

    if (publish_fps_ > 0) {
      publish_interval_ms_ = 1000.0 / publish_fps_;
      RCLCPP_INFO(this->get_logger(), "Raw publish rate throttled to %d FPS (interval: %.1f ms)",
        publish_fps_, publish_interval_ms_);
    }

    if (compressed_publish_fps_ > 0) {
      compressed_publish_interval_ms_ = 1000.0 / compressed_publish_fps_;
      RCLCPP_INFO(this->get_logger(), "Compressed publish rate throttled to %d FPS (interval: %.1f ms)",
        compressed_publish_fps_, compressed_publish_interval_ms_);
    }

    // Capitalize fourcc
    for (auto & c : fourcc_) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }

    RCLCPP_INFO(this->get_logger(),
      "Initializing Image Splitter Node (rclcpp C++):\n"
      "  Preferred Device: /dev/video%d (Auto Scan: %s)\n"
      "  Target Resolution: %dx%d\n"
      "  Target FPS: %d\n"
      "  Target Format (FOURCC): %s\n"
      "  Frame ID: %s\n"
      "  Rotation Angle: %d\n"
      "  JPEG Quality: %d\n"
      "  Image Encoding: %s\n"
      "  Raw Publish FPS: %s\n"
      "  Compressed Publish FPS: %s",
      video_device_, auto_detect_device_ ? "Enabled" : "Disabled",
      width_, height_, fps_, fourcc_.c_str(),
      frame_id_.c_str(), rotation_angle_, jpeg_quality_, encoding_.c_str(),
      publish_fps_ > 0 ? std::to_string(publish_fps_).c_str() : "Full Speed",
      compressed_publish_fps_ > 0 ? std::to_string(compressed_publish_fps_).c_str() : "Full Speed");

    // Initialize publishers
    // RELIABLE: compatible with both RELIABLE and BEST_EFFORT subscribers (RVIZ2 uses RELIABLE)
    // VOLATILE + KeepLast(5): avoid buffering stale frames over network
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5))
      .reliable()
      .durability_volatile();

    left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/left/image_raw", sensor_qos);
    right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/right/image_raw", sensor_qos);

    left_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/left/image_raw/compressed", sensor_qos);
    right_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/right/image_raw/compressed", sensor_qos);

    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/left/camera_info", sensor_qos);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/right/camera_info", sensor_qos);

    // Initialize camera capture with auto-scan support
    int active_device = find_working_camera_device(video_device_, auto_detect_device_);
    if (active_device < 0 || !cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to open any video capture device (Preferred /dev/video%d, Auto-scan: %s).",
        video_device_, auto_detect_device_ ? "true" : "false");
      throw std::runtime_error("Could not open video device");
    }
    video_device_ = active_device;

    RCLCPP_INFO(this->get_logger(), "Configuring camera parameters on /dev/video%d...", video_device_);

    // Method 1: FOURCC -> Resolution -> FPS
    if (fourcc_.length() == 4) {
      int fourcc_code = cv::VideoWriter::fourcc(fourcc_[0], fourcc_[1], fourcc_[2], fourcc_[3]);
      cap_.set(cv::CAP_PROP_FOURCC, fourcc_code);
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    // Check configuration
    int actual_fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
    char actual_fourcc_str[5] = {
      static_cast<char>(actual_fourcc & 0xFF),
      static_cast<char>((actual_fourcc >> 8) & 0xFF),
      static_cast<char>((actual_fourcc >> 16) & 0xFF),
      static_cast<char>((actual_fourcc >> 24) & 0xFF),
      '\0'
    };

    if (std::string(actual_fourcc_str) != fourcc_) {
      RCLCPP_WARN(this->get_logger(),
        "Failed to set %s format directly. Camera returned: '%s'. "
        "Attempting alternative configuration order (Resolution -> FOURCC -> FPS)...",
        fourcc_.c_str(), actual_fourcc_str);

      cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
      if (fourcc_.length() == 4) {
        int fourcc_code = cv::VideoWriter::fourcc(fourcc_[0], fourcc_[1], fourcc_[2], fourcc_[3]);
        cap_.set(cv::CAP_PROP_FOURCC, fourcc_code);
      }
      cap_.set(cv::CAP_PROP_FPS, fps_);

      actual_fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
      actual_fourcc_str[0] = static_cast<char>(actual_fourcc & 0xFF);
      actual_fourcc_str[1] = static_cast<char>((actual_fourcc >> 8) & 0xFF);
      actual_fourcc_str[2] = static_cast<char>((actual_fourcc >> 16) & 0xFF);
      actual_fourcc_str[3] = static_cast<char>((actual_fourcc >> 24) & 0xFF);
      actual_fourcc_str[4] = '\0';
    }

    actual_w_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    actual_h_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    actual_fps_ = cap_.get(cv::CAP_PROP_FPS);

    RCLCPP_INFO(this->get_logger(),
      "Camera opened successfully.\n"
      "  Actual Format: %s\n"
      "  Actual Resolution: %dx%d\n"
      "  Actual FPS: %.1f",
      actual_fourcc_str, actual_w_, actual_h_, actual_fps_);

    if (std::string(actual_fourcc_str) != "MJPG") {
      RCLCPP_WARN(this->get_logger(),
        "WARNING: Camera is streaming in '%s' format instead of 'MJPG'. "
        "Notice: Non-MJPG formats (like YUYV) at high resolution will limit USB bandwidth, dropping framerate to ~3-5 FPS.",
        actual_fourcc_str);
    }

    // Start background capture thread
    running_ = true;
    capture_thread_ = std::thread(&ImageSplitterNode::capture_loop, this);
  }

  ~ImageSplitterNode() override
  {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void capture_loop()
  {
    cv::Mat frame;
    while (running_ && rclcpp::ok()) {
      if (!cap_.read(frame) || frame.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      process_and_publish_frame(frame);
    }
  }

  void process_and_publish_frame(cv::Mat & frame)
  {
    auto start_time = std::chrono::steady_clock::now();

    size_t left_raw_subs = left_pub_->get_subscription_count();
    size_t left_comp_subs = left_compressed_pub_->get_subscription_count();
    size_t right_raw_subs = right_pub_->get_subscription_count();
    size_t right_comp_subs = right_compressed_pub_->get_subscription_count();

    // Check if raw needs to be published (considering raw publish_fps throttling)
    bool should_publish_raw = (left_raw_subs > 0 || right_raw_subs > 0);
    if (should_publish_raw && publish_fps_ > 0) {
      double elapsed_raw_ms = std::chrono::duration<double, std::milli>(
        start_time - last_publish_time_).count();
      if (elapsed_raw_ms < publish_interval_ms_) {
        should_publish_raw = false;
      } else {
        last_publish_time_ = start_time;
      }
    }

    // Check if compressed needs to be published (considering compressed_publish_fps throttling)
    bool should_publish_comp = (left_comp_subs > 0 || right_comp_subs > 0);
    if (should_publish_comp && compressed_publish_fps_ > 0) {
      double elapsed_comp_ms = std::chrono::duration<double, std::milli>(
        start_time - last_compressed_publish_time_).count();
      if (elapsed_comp_ms < compressed_publish_interval_ms_) {
        should_publish_comp = false;
      } else {
        last_compressed_publish_time_ = start_time;
      }
    }

    if (!should_publish_raw && !should_publish_comp) {
      return;
    }

    // Rotate image if required
    if (rotation_angle_ == 180) {
      cv::rotate(frame, frame, cv::ROTATE_180);
    } else if (rotation_angle_ != 0) {
      RCLCPP_WARN_ONCE(this->get_logger(),
        "Unsupported rotation angle: %d. No rotation applied.", rotation_angle_);
    }

    rclcpp::Time timestamp = this->get_clock()->now();

    // Split left and right images
    int h = frame.rows;
    int w = frame.cols;
    int half_w = w / 2;

    bool handle_left = (should_publish_raw && left_raw_subs > 0) || (should_publish_comp && left_comp_subs > 0);
    bool handle_right = (should_publish_raw && right_raw_subs > 0) || (should_publish_comp && right_comp_subs > 0);

    cv::Mat left_frame = handle_left ? frame(cv::Rect(0, 0, half_w, h)) : cv::Mat();
    cv::Mat right_frame = handle_right ? frame(cv::Rect(half_w, 0, half_w, h)) : cv::Mat();

    // Convert encoding if necessary
    cv::Mat left_processed, right_processed;
    if (handle_left && !left_frame.empty()) {
      if (encoding_ == "mono8") {
        cv::cvtColor(left_frame, left_processed, cv::COLOR_BGR2GRAY);
      } else if (encoding_ == "rgb8") {
        cv::cvtColor(left_frame, left_processed, cv::COLOR_BGR2RGB);
      } else {
        left_processed = left_frame;
      }
    }

    if (handle_right && !right_frame.empty()) {
      if (encoding_ == "mono8") {
        cv::cvtColor(right_frame, right_processed, cv::COLOR_BGR2GRAY);
      } else if (encoding_ == "rgb8") {
        cv::cvtColor(right_frame, right_processed, cv::COLOR_BGR2RGB);
      } else {
        right_processed = right_frame;
      }
    }

    // Left Frame
    if (handle_left && !left_processed.empty()) {
      try {
        std_msgs::msg::Header left_header;
        left_header.stamp = timestamp;
        left_header.frame_id = "left_camera_optical_frame";

        if (should_publish_raw && left_raw_subs > 0) {
          sensor_msgs::msg::Image::SharedPtr left_msg =
            cv_bridge::CvImage(left_header, encoding_, left_processed).toImageMsg();
          left_pub_->publish(*left_msg);
        }

        if (should_publish_comp && left_comp_subs > 0) {
          std::vector<uchar> buf;
          std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
          if (cv::imencode(".jpg", left_processed, buf, params)) {
            sensor_msgs::msg::CompressedImage comp_msg;
            comp_msg.header = left_header;
            comp_msg.format = "jpeg";
            comp_msg.data = buf;
            left_compressed_pub_->publish(comp_msg);
          }
        }

        publish_camera_info(left_info_pub_, left_header, true, half_w, h);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing left image: %s", e.what());
      }
    }

    // Right Frame
    if (handle_right && !right_processed.empty()) {
      try {
        std_msgs::msg::Header right_header;
        right_header.stamp = timestamp;
        right_header.frame_id = "right_camera_optical_frame";

        if (should_publish_raw && right_raw_subs > 0) {
          sensor_msgs::msg::Image::SharedPtr right_msg =
            cv_bridge::CvImage(right_header, encoding_, right_processed).toImageMsg();
          right_pub_->publish(*right_msg);
        }

        if (should_publish_comp && right_comp_subs > 0) {
          std::vector<uchar> buf;
          std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
          if (cv::imencode(".jpg", right_processed, buf, params)) {
            sensor_msgs::msg::CompressedImage comp_msg;
            comp_msg.header = right_header;
            comp_msg.format = "jpeg";
            comp_msg.data = buf;
            right_compressed_pub_->publish(comp_msg);
          }
        }

        publish_camera_info(right_info_pub_, right_header, false, half_w, h);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing right image: %s", e.what());
      }
    }

    // Log only when subscriber state changes
    if (left_raw_subs != prev_left_raw_subs_ || left_comp_subs != prev_left_comp_subs_ ||
        right_raw_subs != prev_right_raw_subs_ || right_comp_subs != prev_right_comp_subs_) {
      RCLCPP_INFO(this->get_logger(),
        "Subscribers updated - Left raw: %zu, Left comp: %zu | Right raw: %zu, Right comp: %zu",
        left_raw_subs, left_comp_subs, right_raw_subs, right_comp_subs);
      prev_left_raw_subs_ = left_raw_subs;
      prev_left_comp_subs_ = left_comp_subs;
      prev_right_raw_subs_ = right_raw_subs;
      prev_right_comp_subs_ = right_comp_subs;
    }
  }

  void publish_camera_info(
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

  bool is_v4l2_capture_device(int index, std::string & card_name)
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

  int find_working_camera_device(int preferred_device, bool auto_scan)
  {
    std::string card_name;
    // 1. Attempt preferred device first if index is non-negative
    if (preferred_device >= 0) {
      if (is_v4l2_capture_device(preferred_device, card_name)) {
        if (cap_.open(preferred_device, cv::CAP_V4L2)) {
          RCLCPP_INFO(this->get_logger(),
            "Successfully opened preferred camera at /dev/video%d (%s)",
            preferred_device, card_name.c_str());
          return preferred_device;
        }
      }
      RCLCPP_WARN(this->get_logger(),
        "Failed to open preferred camera at /dev/video%d.", preferred_device);
      if (!auto_scan) {
        return -1;
      }
      RCLCPP_INFO(this->get_logger(), "Starting auto-scan for available video devices (/dev/video0 ~ /dev/video63)...");
    }

    // 2. Scan available capture devices (/dev/video0 to /dev/video63)
    for (int dev_idx = 0; dev_idx < 64; ++dev_idx) {
      if (dev_idx == preferred_device) {
        continue;
      }
      if (!is_v4l2_capture_device(dev_idx, card_name)) {
        continue;
      }

      RCLCPP_INFO(this->get_logger(),
        "Testing V4L2 capture device /dev/video%d (%s)...", dev_idx, card_name.c_str());

      if (cap_.open(dev_idx, cv::CAP_V4L2)) {
        RCLCPP_INFO(this->get_logger(),
          "Auto-scan successfully found and opened camera at /dev/video%d (%s)",
          dev_idx, card_name.c_str());
        return dev_idx;
      }
    }

    return -1;
  }

  // Parameters
  int video_device_;
  bool auto_detect_device_;
  int width_;
  int height_;
  int fps_;
  std::string fourcc_;
  std::string frame_id_;
  int rotation_angle_;
  int jpeg_quality_;
  std::string encoding_;
  int publish_fps_;
  double publish_interval_ms_{0.0};
  std::chrono::steady_clock::time_point last_publish_time_{};

  int compressed_publish_fps_;
  double compressed_publish_interval_ms_{0.0};
  std::chrono::steady_clock::time_point last_compressed_publish_time_{};

  int actual_w_{0};
  int actual_h_{0};
  double actual_fps_{0.0};

  // Subscriber state tracking (for change-based logging)
  size_t prev_left_raw_subs_{0};
  size_t prev_left_comp_subs_{0};
  size_t prev_right_raw_subs_{0};
  size_t prev_right_comp_subs_{0};

  // ROS 2 Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr left_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr right_compressed_pub_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

  // OpenCV & Threading
  cv::VideoCapture cap_;
  std::atomic<bool> running_;
  std::thread capture_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ImageSplitterNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("image_splitter_node"), "Node exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
