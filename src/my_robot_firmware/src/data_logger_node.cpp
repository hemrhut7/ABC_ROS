#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <ctime>

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

class DataLoggerNode : public rclcpp::Node {
public:
  DataLoggerNode()
  : Node("data_logger_node"), record_pid_(-1)
  {
    // Parameters
    this->declare_parameter<std::string>("auto_log", "true");
    this->declare_parameter<std::string>("mode", "total");
    this->declare_parameter<std::string>("record_output_dir", "ros2_bag");
    this->declare_parameter<std::vector<std::string>>("record_topics", std::vector<std::string>{});

    // Handle auto_log as string or bool
    std::string auto_log_str = "true";
    try {
      auto param = this->get_parameter("auto_log");
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        auto_log_enabled_ = param.as_bool();
      } else {
        auto_log_str = param.as_string();
        std::transform(auto_log_str.begin(), auto_log_str.end(), auto_log_str.begin(), ::tolower);
        auto_log_enabled_ = (auto_log_str == "true" || auto_log_str == "enable" || auto_log_str == "enabled" || auto_log_str == "1" || auto_log_str == "on");
      }
    } catch (...) {
      auto_log_enabled_ = true;
    }

    mode_ = this->get_parameter("mode").as_string();
    std::transform(mode_.begin(), mode_.end(), mode_.begin(), ::tolower);

    std::string raw_dir = this->get_parameter("record_output_dir").as_string();
    record_topics_ = this->get_parameter("record_topics").as_string_array();

    // Automatic path mapping for Docker container vs Host
    struct stat st;
    bool is_docker = (stat("/workspace", &st) == 0 && S_ISDIR(st.st_mode));

    std::string target_dir;
    if (is_docker) {
      if (raw_dir.rfind("/home/hank/ROS_ABC", 0) == 0) {
        target_dir = "/workspace" + raw_dir.substr(std::string("/home/hank/ROS_ABC").length());
      } else if (!raw_dir.empty() && raw_dir[0] == '/' && raw_dir.rfind("/workspace", 0) != 0) {
        target_dir = "/workspace/ros2_bag";
      } else {
        std::string cleaned = raw_dir;
        while (!cleaned.empty() && cleaned[0] == '/') cleaned.erase(0, 1);
        target_dir = "/workspace/" + cleaned;
      }
    } else {
      std::string expanded = raw_dir;
      if (!expanded.empty() && expanded[0] == '~') {
        const char * home = getenv("HOME");
        if (home) expanded = std::string(home) + expanded.substr(1);
      }
      if (!expanded.empty() && expanded[0] == '/') {
        target_dir = expanded;
      } else {
        target_dir = "/home/hank/ROS_ABC/" + expanded;
      }
    }
    record_output_dir_ = target_dir;

    // Build topic list according to mode if record_topics is empty
    if (record_topics_.empty()) {
      build_topics_from_mode();
    }

    RCLCPP_INFO(this->get_logger(),
      "Data Logger Node initialized:\n"
      "  auto_log: %s\n"
      "  mode: %s\n"
      "  output_dir: %s\n"
      "  topics count: %zu",
      auto_log_enabled_ ? "ENABLE" : "DISABLE",
      mode_.c_str(),
      record_output_dir_.c_str(),
      record_topics_.size());

    if (auto_log_enabled_) {
      start_rosbag_recording();
    } else {
      RCLCPP_INFO(this->get_logger(), "[Auto Log] Recording is DISABLED by auto_log parameter.");
    }
  }

  ~DataLoggerNode() override {
    stop_rosbag_recording();
  }

private:
  void build_topics_from_mode() {
    std::vector<std::string> esp32_topics = {
      "/imu/data_raw",
      "/imu/mag",
      "/baro/pressure",
      "/baro/temperature",
      "/joint_states",
      "/battery_state",
      "/system_mode",
      "/pid_target",
      "/gnss/fix",
      "/tf_static"
    };

    std::vector<std::string> lidar_topics = {
      "/scan"
    };

    std::vector<std::string> camera_topics = {
      "/camera/stereo/image_raw/compressed",
      "/camera/left/image_raw/compressed",
      "/camera/right/image_raw/compressed",
      "/camera/left/camera_info",
      "/camera/right/camera_info"
    };

    if (mode_ == "esp32") {
      record_topics_ = esp32_topics;
    } else if (mode_ == "lidar") {
      record_topics_ = esp32_topics;
      record_topics_.insert(record_topics_.end(), lidar_topics.begin(), lidar_topics.end());
    } else if (mode_ == "camera") {
      record_topics_ = esp32_topics;
      record_topics_.insert(record_topics_.end(), camera_topics.begin(), camera_topics.end());
    } else if (mode_ == "total") {
      record_topics_ = esp32_topics;
      record_topics_.insert(record_topics_.end(), lidar_topics.begin(), lidar_topics.end());
      record_topics_.insert(record_topics_.end(), camera_topics.begin(), camera_topics.end());
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown mode '%s', defaulting to 'esp32' topics.", mode_.c_str());
      record_topics_ = esp32_topics;
    }
  }

  void start_rosbag_recording() {
    std::string mkdir_cmd = "mkdir -p \"" + record_output_dir_ + "\"";
    int res = system(mkdir_cmd.c_str());
    (void)res;

    std::time_t now = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));
    std::string bag_name = "rosbag2_" + mode_ + "_" + std::string(buf);
    std::string bag_path = record_output_dir_ + "/" + bag_name;

    RCLCPP_INFO(this->get_logger(), "[Auto Log] Starting rosbag recording (mode: %s) -> %s", mode_.c_str(), bag_path.c_str());

    pid_t pid = fork();
    if (pid == 0) {
      // Child process
      setsid();

      std::vector<std::string> args = {"ros2", "bag", "record", "-o", bag_path};
      for (const auto & t : record_topics_) {
        args.push_back(t);
      }

      std::vector<char*> c_args;
      for (auto & a : args) {
        c_args.push_back(const_cast<char*>(a.c_str()));
      }
      c_args.push_back(nullptr);

      int dev_null = open("/dev/null", O_WRONLY);
      if (dev_null >= 0) {
        dup2(dev_null, STDOUT_FILENO);
        dup2(dev_null, STDERR_FILENO);
        close(dev_null);
      }

      execvp("ros2", c_args.data());
      _exit(1);
    } else if (pid > 0) {
      record_pid_ = pid;
    } else {
      RCLCPP_ERROR(this->get_logger(), "[Auto Log] Failed to fork process for rosbag recording");
    }
  }

  void stop_rosbag_recording() {
    if (record_pid_ > 0) {
      int status;
      if (waitpid(record_pid_, &status, WNOHANG) == 0) {
        RCLCPP_INFO(this->get_logger(), "[Auto Log] Stopping rosbag recording process cleanly...");
        kill(-record_pid_, SIGINT);
        for (int i = 0; i < 50; ++i) {
          if (waitpid(record_pid_, &status, WNOHANG) != 0) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "[Auto Log] Rosbag recording saved successfully.");
      }
      record_pid_ = -1;
    }
  }

  bool auto_log_enabled_;
  std::string mode_;
  std::string record_output_dir_;
  std::vector<std::string> record_topics_;
  pid_t record_pid_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DataLoggerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("data_logger_node"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
