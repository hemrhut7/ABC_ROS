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

class ImageSplitterNode : public rclcpp::Node
{
public:
  ImageSplitterNode()
  : Node("image_splitter_node"),
    running_(false),
    frame_count_(0)
  {
    // Declare parameters
    this->declare_parameter<int>("video_device", 0);
    this->declare_parameter<int>("width", 1280);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<int>("fps", 60);
    this->declare_parameter<std::string>("fourcc", "MJPG");
    this->declare_parameter<std::string>("frame_id", "camera_link");
    this->declare_parameter<int>("rotation_angle", 180);
    this->declare_parameter<int>("jpeg_quality", 60);

    // Get parameters
    video_device_ = this->get_parameter("video_device").as_int();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    fps_ = this->get_parameter("fps").as_int();
    fourcc_ = this->get_parameter("fourcc").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    rotation_angle_ = this->get_parameter("rotation_angle").as_int();
    jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();

    // Capitalize fourcc
    for (auto & c : fourcc_) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }

    RCLCPP_INFO(this->get_logger(),
      "Initializing Image Splitter Node (rclcpp C++):\n"
      "  Device: /dev/video%d\n"
      "  Target Resolution: %dx%d\n"
      "  Target FPS: %d\n"
      "  Target Format (FOURCC): %s\n"
      "  Frame ID: %s\n"
      "  Rotation Angle: %d\n"
      "  JPEG Quality: %d",
      video_device_, width_, height_, fps_, fourcc_.c_str(),
      frame_id_.c_str(), rotation_angle_, jpeg_quality_);

    // Initialize publishers
    left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/left/image_raw", 10);
    right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/right/image_raw", 10);

    left_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/left/image_raw/compressed", 10);
    right_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/right/image_raw/compressed", 10);

    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/left/camera_info", 10);
    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/right/camera_info", 10);

    // Initialize camera capture
    cap_.open(video_device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video device /dev/video%d", video_device_);
      throw std::runtime_error("Could not open video device");
    }

    RCLCPP_INFO(this->get_logger(), "Configuring camera parameters...");

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

    bool any_left = (left_raw_subs > 0) || (left_comp_subs > 0);
    bool any_right = (right_raw_subs > 0) || (right_comp_subs > 0);

    if (!any_left && !any_right) {
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
    if (any_left || any_right) {
      int h = frame.rows;
      int w = frame.cols;
      int half_w = w / 2;

      cv::Mat left_frame = any_left ? frame(cv::Rect(0, 0, half_w, h)) : cv::Mat();
      cv::Mat right_frame = any_right ? frame(cv::Rect(half_w, 0, half_w, h)) : cv::Mat();

      // Left Frame
      if (any_left && !left_frame.empty()) {
        try {
          std_msgs::msg::Header left_header;
          left_header.stamp = timestamp;
          left_header.frame_id = "left_" + frame_id_;

          if (left_raw_subs > 0) {
            sensor_msgs::msg::Image::SharedPtr left_msg =
              cv_bridge::CvImage(left_header, "bgr8", left_frame).toImageMsg();
            left_pub_->publish(*left_msg);
          }

          if (left_comp_subs > 0) {
            std::vector<uchar> buf;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
            if (cv::imencode(".jpg", left_frame, buf, params)) {
              sensor_msgs::msg::CompressedImage comp_msg;
              comp_msg.header = left_header;
              comp_msg.format = "jpeg";
              comp_msg.data = buf;
              left_compressed_pub_->publish(comp_msg);
            }
          }

          publish_camera_info(left_info_pub_, left_header, half_w, h);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "Error publishing left image: %s", e.what());
        }
      }

      // Right Frame
      if (any_right && !right_frame.empty()) {
        try {
          std_msgs::msg::Header right_header;
          right_header.stamp = timestamp;
          right_header.frame_id = "right_" + frame_id_;

          if (right_raw_subs > 0) {
            sensor_msgs::msg::Image::SharedPtr right_msg =
              cv_bridge::CvImage(right_header, "bgr8", right_frame).toImageMsg();
            right_pub_->publish(*right_msg);
          }

          if (right_comp_subs > 0) {
            std::vector<uchar> buf;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
            if (cv::imencode(".jpg", right_frame, buf, params)) {
              sensor_msgs::msg::CompressedImage comp_msg;
              comp_msg.header = right_header;
              comp_msg.format = "jpeg";
              comp_msg.data = buf;
              right_compressed_pub_->publish(comp_msg);
            }
          }

          publish_camera_info(right_info_pub_, right_header, half_w, h);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "Error publishing right image: %s", e.what());
        }
      }
    }

    // Diagnostic logging every 150 frames
    frame_count_++;
    if (frame_count_ % 150 == 0) {
      auto end_time = std::chrono::steady_clock::now();
      double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
      RCLCPP_INFO(this->get_logger(),
        "Processing loop execution time: %.2f ms "
        "(Subscribers - Left: %zu/%zu, Right: %zu/%zu)",
        duration_ms, left_raw_subs, left_comp_subs, right_raw_subs, right_comp_subs);
    }
  }

  void publish_camera_info(
    const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr & publisher,
    const std_msgs::msg::Header & header,
    int width,
    int height)
  {
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header = header;
    info_msg.width = width;
    info_msg.height = height;
    info_msg.distortion_model = "plumb_bob";
    info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info_msg.k = {
      1.0, 0.0, static_cast<double>(width / 2.0),
      0.0, 1.0, static_cast<double>(height / 2.0),
      0.0, 0.0, 1.0
    };
    info_msg.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    info_msg.p = {
      1.0, 0.0, static_cast<double>(width / 2.0), 0.0,
      0.0, 1.0, static_cast<double>(height / 2.0), 0.0,
      0.0, 0.0, 1.0, 0.0
    };
    publisher->publish(info_msg);
  }

  // Parameters
  int video_device_;
  int width_;
  int height_;
  int fps_;
  std::string fourcc_;
  std::string frame_id_;
  int rotation_angle_;
  int jpeg_quality_;

  int actual_w_{0};
  int actual_h_{0};
  double actual_fps_{0.0};

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
  uint64_t frame_count_;
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
