#ifndef MY_ROBOT_PERCEPTION_IMAGE_SPLITTER_NODE_HPP_
#define MY_ROBOT_PERCEPTION_IMAGE_SPLITTER_NODE_HPP_

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

namespace my_robot_perception
{

class ImageSplitterNode : public rclcpp::Node
{
public:
  explicit ImageSplitterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImageSplitterNode() override;

private:
  void capture_loop();
  void process_and_publish_frame(cv::Mat & frame);
  void publish_camera_info(
    const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr & publisher,
    const std_msgs::msg::Header & header,
    bool is_left,
    int width,
    int height);

  bool is_v4l2_capture_device(int index, std::string & card_name);
  std::string build_gstreamer_pipeline(int dev_idx, int w, int h, int fps_val, int rot_angle, const std::string & decoder_type);
  int open_camera(int preferred_device, bool auto_scan);

  // Parameters
  bool use_hardware_decode_{true};
  std::string hw_decoder_type_{"nvjpegdec"};
  int video_device_{0};
  bool auto_detect_device_{true};
  int width_{1280};
  int height_{480};
  int fps_{60};
  std::string fourcc_{"MJPG"};
  std::string frame_id_{"camera_link"};
  std::string left_frame_id_{"left_camera_optical_frame"};
  std::string right_frame_id_{"right_camera_optical_frame"};
  int rotation_angle_{180};
  int jpeg_quality_{60};

  bool publish_bgr_{true};
  bool publish_mono_{true};
  bool publish_compressed_{true};

  int bgr_publish_fps_{15};
  double bgr_interval_ms_{0.0};
  std::chrono::steady_clock::time_point last_bgr_publish_time_{};

  int mono_publish_fps_{0};
  double mono_interval_ms_{0.0};
  std::chrono::steady_clock::time_point last_mono_publish_time_{};

  int compressed_publish_fps_{15};
  double compressed_interval_ms_{0.0};
  std::chrono::steady_clock::time_point last_compressed_publish_time_{};

  bool enable_mono_filter_{true};
  int mono_filter_ksize_{3};
  double mono_filter_sigma_{0.5};

  // State
  bool hardware_decode_active_{false};
  bool hardware_rotated_{false};
  int actual_w_{0};
  int actual_h_{0};
  double actual_fps_{0.0};

  // Subscriber state tracking for clean logging
  size_t prev_left_bgr_subs_{0};
  size_t prev_right_bgr_subs_{0};
  size_t prev_left_mono_subs_{0};
  size_t prev_right_mono_subs_{0};
  size_t prev_left_comp_subs_{0};
  size_t prev_right_comp_subs_{0};

  // ROS 2 Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_bgr_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_bgr_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_mono_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_mono_pub_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr left_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr right_compressed_pub_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

  // OpenCV & Threading
  cv::VideoCapture cap_;
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
};

}  // namespace my_robot_perception

#endif  // MY_ROBOT_PERCEPTION_IMAGE_SPLITTER_NODE_HPP_
