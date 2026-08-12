#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <memory>
#include <limits>

struct LidarPoint {
  float distance;
  float angle;      // in degrees
  float intensity;
};

struct ScanData {
  double timestamp;
  std::vector<LidarPoint> points;
};

class N10LidarDriver {
public:
  static constexpr size_t PACKET_SIZE = 58;
  static constexpr uint8_t HEADER_0 = 0xA5;
  static constexpr uint8_t HEADER_1 = 0x5A;

  N10LidarDriver(const std::string & port = "", int baud_rate = 230400)
  : port_name_(port), baud_rate_(baud_rate), fd_(-1), last_angle_(0.0f)
  {}

  ~N10LidarDriver() {
    disconnect();
  }

  std::string find_port() {
    if (!port_name_.empty()) {
      return port_name_;
    }

    // Common USB serial ports for LiDAR
    std::vector<std::string> prefixes = {"/dev/ttyUSB", "/dev/ttyACM"};
    for (const auto & prefix : prefixes) {
      for (int i = 0; i < 10; ++i) {
        std::string dev_path = prefix + std::to_string(i);
        struct stat buffer;
        if (stat(dev_path.c_str(), &buffer) == 0) {
          port_name_ = dev_path;
          return dev_path;
        }
      }
    }
    return "";
  }

  bool connect() {
    if (port_name_.empty()) {
      find_port();
    }
    if (port_name_.empty()) {
      return false;
    }

    fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
      return false;
    }

    // Configure serial port options
    struct termios options;
    tcgetattr(fd_, &options);

    speed_t speed = B230400;
    if (baud_rate_ == 115200) speed = B115200;
    else if (baud_rate_ == 460800) speed = B460800;

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag &= ~PARENB;   // No parity bit
    options.c_cflag &= ~CSTOPB;   // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;        // 8 data bits
    options.c_cflag &= ~CRTSCTS;   // No hardware flow control
    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem lines

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable software flow control
    options.c_oflag &= ~OPOST;                          // Raw output

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
      close(fd_);
      fd_ = -1;
      return false;
    }

    // Set non-blocking mode
    fcntl(fd_, F_SETFL, FNDELAY);
    return true;
  }

  void disconnect() {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  bool is_connected() const {
    return fd_ >= 0;
  }

  std::string get_port_name() const {
    return port_name_;
  }

  static uint8_t cal_crc8(const uint8_t * data, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
      sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
  }

  bool get_scan(ScanData & completed_scan) {
    if (fd_ < 0) return false;

    int bytes_available = 0;
    if (ioctl(fd_, FIONREAD, &bytes_available) < 0 || bytes_available <= 0) {
      return false;
    }

    // Read available bytes into internal stream buffer
    std::vector<uint8_t> read_buf(bytes_available);
    ssize_t bytes_read = read(fd_, read_buf.data(), bytes_available);
    if (bytes_read > 0) {
      stream_buf_.insert(stream_buf_.end(), read_buf.begin(), read_buf.begin() + bytes_read);
    }

    bool has_new_scan = false;

    // Process packets in buffer
    while (stream_buf_.size() >= PACKET_SIZE) {
      // Find header 0xA5 0x5A
      size_t header_idx = stream_buf_.size();
      for (size_t i = 0; i + 1 < stream_buf_.size(); ++i) {
        if (stream_buf_[i] == HEADER_0 && stream_buf_[i + 1] == HEADER_1) {
          header_idx = i;
          break;
        }
      }

      if (header_idx > 0) {
        // Discard bytes before header
        if (header_idx >= stream_buf_.size()) {
          stream_buf_.clear();
          break;
        }
        stream_buf_.erase(stream_buf_.begin(), stream_buf_.begin() + header_idx);
      }

      if (stream_buf_.size() < PACKET_SIZE) {
        break;
      }

      // Check CRC-8
      uint8_t packet_crc = stream_buf_[57];
      if (packet_crc != cal_crc8(stream_buf_.data(), 57)) {
        // Corrupted packet, drop header and search again
        stream_buf_.erase(stream_buf_.begin(), stream_buf_.begin() + 2);
        continue;
      }

      // Parse packet
      float start_angle = (stream_buf_[5] * 256 + stream_buf_[6]) / 100.0f;
      float end_angle = (stream_buf_[55] * 256 + stream_buf_[56]) / 100.0f;

      float angle_diff = (end_angle < start_angle) ? (end_angle + 360.0f - start_angle) : (end_angle - start_angle);
      constexpr int points_in_packet = 16;

      for (int i = 0; i < points_in_packet; ++i) {
        size_t idx = 7 + i * 3;
        uint16_t dist_raw = stream_buf_[idx] * 256 + stream_buf_[idx + 1];
        uint8_t intensity = stream_buf_[idx + 2];

        float distance = (dist_raw == 0xFFFF) ? std::numeric_limits<float>::infinity() : (dist_raw / 1000.0f);
        float angle = start_angle + (angle_diff / (points_in_packet - 1)) * i;
        if (angle >= 360.0f) {
          angle -= 360.0f;
        }

        current_points_.push_back({distance, angle, static_cast<float>(intensity)});
      }

      bool revolution_completed = (start_angle < last_angle_ - 180.0f);
      last_angle_ = start_angle;

      if (revolution_completed && !current_points_.empty()) {
        completed_scan.timestamp = std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        completed_scan.points = current_points_;
        current_points_.clear();
        has_new_scan = true;
      }

      // Remove processed packet
      stream_buf_.erase(stream_buf_.begin(), stream_buf_.begin() + PACKET_SIZE);

      if (has_new_scan) {
        return true;
      }
    }

    return false;
  }

private:
  std::string port_name_;
  int baud_rate_;
  int fd_;
  float last_angle_;
  std::vector<uint8_t> stream_buf_;
  std::vector<LidarPoint> current_points_;
};

class N10LidarNode : public rclcpp::Node {
public:
  N10LidarNode()
  : Node("n10_lidar_node"), scan_count_(0), is_connected_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("port", "");
    this->declare_parameter<int>("baud_rate", 230400);
    this->declare_parameter<std::string>("frame_id", "laser_link");
    this->declare_parameter<std::string>("topic_name", "/scan");
    this->declare_parameter<double>("range_min", 0.05);
    this->declare_parameter<double>("range_max", 12.0);

    // Read parameters
    port_param_ = this->get_parameter("port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();

    RCLCPP_INFO(this->get_logger(),
      "Initializing N10 LiDAR Node (rclcpp C++ Standalone):\n"
      "  Target Port: %s\n"
      "  Baud Rate: %d\n"
      "  Frame ID: %s\n"
      "  Output Topic: %s\n"
      "  Range Min/Max: %.2fm / %.2fm",
      port_param_.empty() ? "Auto-detecting USB port" : port_param_.c_str(),
      baud_rate_, frame_id_.c_str(), topic_name_.c_str(),
      range_min_, range_max_);

    // Publisher
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, 10);

    // Hardware driver
    driver_ = std::make_unique<N10LidarDriver>(port_param_, baud_rate_);
    if (driver_->connect()) {
      is_connected_ = true;
      last_scan_time_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(this->get_logger(), "LiDAR connection: ESTABLISHED on port %s", driver_->get_port_name().c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
        "Initial LiDAR connection failed on port %s. Will retry automatically.",
        port_param_.empty() ? "auto-detect" : port_param_.c_str());
    }

    // 200 Hz polling timer (5 ms)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&N10LidarNode::poll_lidar, this));
  }

  ~N10LidarNode() override {
    if (driver_) {
      driver_->disconnect();
    }
  }

private:
  void poll_lidar() {
    try {
      if (!driver_ || !driver_->is_connected()) {
        if (is_connected_) {
          is_connected_ = false;
          RCLCPP_WARN(this->get_logger(), "LiDAR connection: LOST (driver disconnected)");
        }

        // Try reconnection periodically
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_reconnect_attempt_).count() >= 2) {
          last_reconnect_attempt_ = now;
          if (driver_ && driver_->connect()) {
            is_connected_ = true;
            last_scan_time_ = now;
            RCLCPP_INFO(this->get_logger(), "LiDAR connection: RE-ESTABLISHED on port %s", driver_->get_port_name().c_str());
          }
        }
        return;
      }

      ScanData data;
      bool got_scan = false;
      while (driver_ && driver_->is_connected() && driver_->get_scan(data)) {
        if (!data.points.empty()) {
          publish_scan(data);
          got_scan = true;
        }
      }

      auto now = std::chrono::steady_clock::now();
      if (got_scan) {
        if (!is_connected_) {
          is_connected_ = true;
          RCLCPP_INFO(this->get_logger(), "LiDAR connection: ESTABLISHED on port %s", driver_->get_port_name().c_str());
        }
      } else if (is_connected_) {
        // Check timeout
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_scan_time_).count();
        if (elapsed >= 3) {
          is_connected_ = false;
          RCLCPP_WARN(this->get_logger(), "LiDAR connection: LOST (no scan received for >3s)");
        }
      }
    } catch (const std::exception & e) {
      if (is_connected_) {
        is_connected_ = false;
        RCLCPP_WARN(this->get_logger(), "LiDAR connection: LOST (Exception: %s)", e.what());
      }
    }
  }

  void publish_scan(const ScanData & data) {
    if (data.points.empty()) return;

    auto current_time = std::chrono::steady_clock::now();
    double scan_time = std::chrono::duration<double>(current_time - last_scan_time_).count();
    last_scan_time_ = current_time;
    scan_count_++;

    constexpr size_t num_bins = 450; // Fixed 450 angular bins for full 360-degree scan (~0.8 deg/bin)
    constexpr float angle_min = -static_cast<float>(M_PI);
    constexpr float angle_max = static_cast<float>(M_PI);
    constexpr float angle_increment = (angle_max - angle_min) / num_bins;

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = this->get_clock()->now();
    scan_msg.header.frame_id = frame_id_;

    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
    scan_msg.angle_increment = angle_increment;
    scan_msg.time_increment = static_cast<float>(scan_time / num_bins);
    scan_msg.scan_time = static_cast<float>(scan_time);
    scan_msg.range_min = static_cast<float>(range_min_);
    scan_msg.range_max = static_cast<float>(range_max_);

    scan_msg.ranges.assign(num_bins, std::numeric_limits<float>::infinity());
    scan_msg.intensities.assign(num_bins, 0.0f);

    for (const auto & pt : data.points) {
      if (std::isinf(pt.distance) || std::isnan(pt.distance) ||
          pt.distance < range_min_ || pt.distance > range_max_) {
        continue;
      }

      // N10 LiDAR hardware spins Clockwise (CW).
      // ROS standard (REP-103) uses right-handed coordinates: positive angles are Counter-Clockwise (CCW).
      // Convert CW angle in degrees (0..360) to ROS CCW angle in radians [-PI, PI).
      float ros_angle = -(pt.angle * static_cast<float>(M_PI) / 180.0f);
      while (ros_angle < -static_cast<float>(M_PI)) ros_angle += 2.0f * static_cast<float>(M_PI);
      while (ros_angle >= static_cast<float>(M_PI)) ros_angle -= 2.0f * static_cast<float>(M_PI);

      int bin = static_cast<int>(std::floor((ros_angle - angle_min) / angle_increment));
      if (bin >= 0 && static_cast<size_t>(bin) < num_bins) {
        if (std::isinf(scan_msg.ranges[bin]) || pt.distance < scan_msg.ranges[bin]) {
          scan_msg.ranges[bin] = pt.distance;
          scan_msg.intensities[bin] = pt.intensity;
        }
      }
    }

    scan_pub_->publish(scan_msg);
  }

  std::string port_param_;
  int baud_rate_;
  std::string frame_id_;
  std::string topic_name_;
  double range_min_;
  double range_max_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<N10LidarDriver> driver_;

  std::chrono::steady_clock::time_point last_scan_time_;
  std::chrono::steady_clock::time_point last_reconnect_attempt_;
  uint64_t scan_count_;
  bool is_connected_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<N10LidarNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("n10_lidar_node"), "N10 LiDAR Node exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
