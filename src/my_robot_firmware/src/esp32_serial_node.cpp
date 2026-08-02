#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <thread>
#include <climits>

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <ctime>

// Helper to convert quaternion to Euler angles (roll, pitch, yaw) in degrees
static void euler_from_quaternion(double x, double y, double z, double w, double & roll, double & pitch, double & yaw) {
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + y * y);
  roll = std::atan2(t0, t1) * 180.0 / M_PI;

  double t2 = +2.0 * (w * y - z * x);
  t2 = t2 > +1.0 ? +1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch = std::asin(t2) * 180.0 / M_PI;

  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (y * y + z * z);
  yaw = std::atan2(t3, t4) * 180.0 / M_PI;
}

// System Modes mapping (matching hal_type_define.h)
static std::string get_system_mode_name(int mode) {
  switch (mode) {
    case 0: return "MODE_STOP";
    case 1: return "MODE_FREE";
    case 2: return "MODE_PWM";
    case 3: return "MODE_MOTOR";
    case 4: return "MODE_ANGLE";
    case 5: return "MODE_VELOCITY";
    case 6: return "MODE_REMOTE";
    default: return "UNKNOWN";
  }
}

class ESP32SerialNode : public rclcpp::Node {
public:
  ESP32SerialNode()
  : Node("esp32_serial_node"), record_pid_(-1)
  {
    // Parameters
    this->declare_parameter<double>("display_rate", 1.0);
    this->declare_parameter<bool>("auto_record", true);
    this->declare_parameter<std::string>("record_output_dir", "ros2_bag");
    this->declare_parameter<std::vector<std::string>>("record_topics", {
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
    });

    display_rate_ = this->get_parameter("display_rate").as_double();
    auto_record_ = this->get_parameter("auto_record").as_bool();
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

    RCLCPP_INFO(this->get_logger(), "Starting ESP32 Serial/Telemetry Node with display rate: %.1f Hz", display_rate_);

    if (auto_record_) {
      start_rosbag_recording();
    }

    // QoS sensor data profile
    auto qos = rclcpp::SensorDataQoS();

    // Subscribers
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", qos, std::bind(&ESP32SerialNode::imu_callback, this, std::placeholders::_1));
    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "/imu/mag", qos, std::bind(&ESP32SerialNode::mag_callback, this, std::placeholders::_1));
    pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      "/baro/pressure", qos, std::bind(&ESP32SerialNode::pressure_callback, this, std::placeholders::_1));
    temp_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
      "/baro/temperature", qos, std::bind(&ESP32SerialNode::temp_callback, this, std::placeholders::_1));
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos, std::bind(&ESP32SerialNode::joint_states_callback, this, std::placeholders::_1));
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", qos, std::bind(&ESP32SerialNode::battery_callback, this, std::placeholders::_1));
    system_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/system_mode", qos, std::bind(&ESP32SerialNode::system_mode_callback, this, std::placeholders::_1));
    delay_count_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/delay_count", qos, std::bind(&ESP32SerialNode::delay_count_callback, this, std::placeholders::_1));
    pid_target_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/pid_target", qos, std::bind(&ESP32SerialNode::pid_target_callback, this, std::placeholders::_1));

    // Periodic display timer
    double interval_sec = display_rate_ > 0.0 ? 1.0 / display_rate_ : 1.0;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(interval_sec),
      std::bind(&ESP32SerialNode::display_telemetry, this));
  }

  ~ESP32SerialNode() override {
    stop_rosbag_recording();
  }

  // Helper APIs for command publishing (currently disabled to match python behavior)
  void publish_cmd_vel(double linear_x, double angular_z) {
    (void)linear_x;
    (void)angular_z;
    RCLCPP_WARN(this->get_logger(), "publish_cmd_vel is currently DISABLED to prevent accidental command transmission.");
  }

  void publish_cmd_mode(int mode) {
    (void)mode;
    RCLCPP_WARN(this->get_logger(), "publish_cmd_mode is currently DISABLED to prevent accidental command transmission.");
  }

private:
  // Callbacks
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_msg_ = msg;
    imu_count_++;
  }

  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    mag_msg_ = msg;
    mag_count_++;
  }

  void pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    pressure_msg_ = msg;
    pressure_count_++;
  }

  void temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
    temp_msg_ = msg;
    temp_count_++;
  }

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_states_msg_ = msg;
    joint_states_count_++;
  }

  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    battery_msg_ = msg;
    battery_count_++;
  }

  void system_mode_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    system_mode_msg_ = msg;
    system_mode_count_++;
  }

  void delay_count_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    delay_count_msg_ = msg;
    delay_count_count_++;
  }

  void pid_target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    pid_target_msg_ = msg;
    pid_target_count_++;
  }

  // Display telemetry dashboard
  void display_telemetry() {
    std::ostringstream ss;
    ss << "==================================================\n";
    ss << "           ESP32 TELEMETRY MONITOR (SERIAL)       \n";
    ss << "==================================================\n";

    // 1. System Mode and Delay Count
    if (system_mode_msg_) {
      int mode = system_mode_msg_->data;
      std::string mode_name = get_system_mode_name(mode);
      std::string mode_str = mode_name + " (" + std::to_string(mode) + ")";
      ss << "[System]  Mode: " << std::left << std::setw(20) << mode_str
         << " | Delay Count: " << (delay_count_msg_ ? std::to_string(delay_count_msg_->data) : "N/A") << "\n";
    } else {
      ss << "[System]  Mode: N/A                  | Delay Count: "
         << (delay_count_msg_ ? std::to_string(delay_count_msg_->data) : "N/A") << "\n";
    }

    // 2. Battery
    if (battery_msg_) {
      ss << "[Battery] Voltage: " << std::fixed << std::setprecision(2) << battery_msg_->voltage << " V\n";
    } else {
      ss << "[Battery] Voltage: N/A\n";
    }

    // 3. Environment
    ss << "[Env]     Temp: ";
    if (temp_msg_) {
      ss << std::fixed << std::setprecision(2) << std::setw(6) << temp_msg_->temperature << " °C";
    } else {
      ss << "N/A         ";
    }
    ss << " | Pressure: ";
    if (pressure_msg_) {
      ss << std::fixed << std::setprecision(2) << (pressure_msg_->fluid_pressure / 100.0) << " hPa\n";
    } else {
      ss << "N/A\n";
    }

    // 4. IMU
    if (imu_msg_) {
      double r, p, y;
      euler_from_quaternion(
        imu_msg_->orientation.x, imu_msg_->orientation.y, imu_msg_->orientation.z, imu_msg_->orientation.w,
        r, p, y);
      ss << "[IMU]     Roll: " << std::fixed << std::setprecision(1) << std::setw(6) << r
         << "° | Pitch: " << std::setw(6) << p << "° | Yaw: " << std::setw(6) << y << "°\n";
      ss << "          Accel: [x: " << std::setprecision(2) << std::setw(6) << imu_msg_->linear_acceleration.x
         << ", y: " << std::setw(6) << imu_msg_->linear_acceleration.y
         << ", z: " << std::setw(6) << imu_msg_->linear_acceleration.z << "] m/s²\n";
      ss << "          Gyro:  [x: " << std::setprecision(2) << std::setw(6) << imu_msg_->angular_velocity.x
         << ", y: " << std::setw(6) << imu_msg_->angular_velocity.y
         << ", z: " << std::setw(6) << imu_msg_->angular_velocity.z << "] rad/s\n";
    } else {
      ss << "[IMU]     No Data\n";
    }

    // 5. Magnetometer
    if (mag_msg_) {
      double mx = mag_msg_->magnetic_field.x * 1e6;
      double my = mag_msg_->magnetic_field.y * 1e6;
      double mz = mag_msg_->magnetic_field.z * 1e6;
      ss << "[Mag]     Field: [x: " << std::fixed << std::setprecision(1) << std::setw(6) << mx
         << ", y: " << std::setw(6) << my << ", z: " << std::setw(6) << mz << "] uT\n";
    } else {
      ss << "[Mag]     No Data\n";
    }

    // 6. Joint States
    if (joint_states_msg_ && joint_states_msg_->name.size() >= 2) {
      size_t l_idx = 0, r_idx = 1;
      for (size_t i = 0; i < joint_states_msg_->name.size(); ++i) {
        if (joint_states_msg_->name[i] == "left_wheel") l_idx = i;
        else if (joint_states_msg_->name[i] == "right_wheel") r_idx = i;
      }

      double l_pos = joint_states_msg_->position.size() > l_idx ? joint_states_msg_->position[l_idx] : 0.0;
      double r_pos = joint_states_msg_->position.size() > r_idx ? joint_states_msg_->position[r_idx] : 0.0;
      double l_vel = joint_states_msg_->velocity.size() > l_idx ? joint_states_msg_->velocity[l_idx] : 0.0;
      double r_vel = joint_states_msg_->velocity.size() > r_idx ? joint_states_msg_->velocity[r_idx] : 0.0;
      double l_eff = joint_states_msg_->effort.size() > l_idx ? joint_states_msg_->effort[l_idx] : 0.0;
      double r_eff = joint_states_msg_->effort.size() > r_idx ? joint_states_msg_->effort[r_idx] : 0.0;

      ss << "[Joints]  left_wheel  | pos: " << std::fixed << std::setprecision(2) << std::setw(6) << l_pos
         << " rad | vel: " << std::setw(6) << l_vel << " rad/s | pwm: " << std::setprecision(1) << std::setw(5) << l_eff << "\n";
      ss << "          right_wheel | pos: " << std::fixed << std::setprecision(2) << std::setw(6) << r_pos
         << " rad | vel: " << std::setw(6) << r_vel << " rad/s | pwm: " << std::setprecision(1) << std::setw(5) << r_eff << "\n";
    } else {
      ss << "[Joints]  No Data\n";
    }

    // 7. PID Target
    if (pid_target_msg_ && pid_target_msg_->data.size() >= 6) {
      const auto & pt = pid_target_msg_->data;
      ss << "[PID Tar] Target Val: " << std::fixed << std::setprecision(2) << std::setw(6) << pt[0]
         << " | Motor RPM L: " << std::setprecision(1) << std::setw(6) << pt[1]
         << " | R: " << std::setw(6) << pt[2] << "\n";
      ss << "          Pitch Tar:  " << std::setprecision(2) << std::setw(6) << pt[3]
         << " | Vel Tar:     " << std::setw(6) << pt[4]
         << " | Steer RPM: " << std::setprecision(1) << std::setw(6) << pt[5] << "\n";
    } else {
      ss << "[PID Tar] No Data\n";
    }

    ss << "--------------------------------------------------\n";
    ss << "Message rates overview (received counts):\n";
    ss << "  imu:" << imu_count_ << ", mag:" << mag_count_ << ", pressure:" << pressure_count_
       << ", temperature:" << temp_count_ << ", joint_states:" << joint_states_count_
       << ", battery:" << battery_count_ << ", system_mode:" << system_mode_count_
       << ", delay_count:" << delay_count_count_ << ", pid_target:" << pid_target_count_ << "\n";
    ss << "==================================================";

    std::cout << "\033[H\033[J" << ss.str() << std::endl;
  }

  // Auto record management
  void start_rosbag_recording() {
    std::string mkdir_cmd = "mkdir -p \"" + record_output_dir_ + "\"";
    int res = system(mkdir_cmd.c_str());
    (void)res;

    std::time_t now = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));
    std::string bag_name = "rosbag2_" + std::string(buf);
    std::string bag_path = record_output_dir_ + "/" + bag_name;

    RCLCPP_INFO(this->get_logger(), "[Auto Record] Starting rosbag recording -> %s", bag_path.c_str());

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
      RCLCPP_ERROR(this->get_logger(), "[Auto Record] Failed to fork process for rosbag recording");
    }
  }

  void stop_rosbag_recording() {
    if (record_pid_ > 0) {
      int status;
      if (waitpid(record_pid_, &status, WNOHANG) == 0) {
        RCLCPP_INFO(this->get_logger(), "[Auto Record] Stopping rosbag recording process cleanly...");
        kill(-record_pid_, SIGINT);
        for (int i = 0; i < 50; ++i) {
          if (waitpid(record_pid_, &status, WNOHANG) != 0) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "[Auto Record] Rosbag recording saved successfully.");
      }
      record_pid_ = -1;
    }
  }

  // Parameters
  double display_rate_;
  bool auto_record_;
  std::string record_output_dir_;
  std::vector<std::string> record_topics_;
  pid_t record_pid_;

  // State messages
  sensor_msgs::msg::Imu::SharedPtr imu_msg_;
  sensor_msgs::msg::MagneticField::SharedPtr mag_msg_;
  sensor_msgs::msg::FluidPressure::SharedPtr pressure_msg_;
  sensor_msgs::msg::Temperature::SharedPtr temp_msg_;
  sensor_msgs::msg::JointState::SharedPtr joint_states_msg_;
  sensor_msgs::msg::BatteryState::SharedPtr battery_msg_;
  std_msgs::msg::Int32::SharedPtr system_mode_msg_;
  std_msgs::msg::Int32::SharedPtr delay_count_msg_;
  std_msgs::msg::Float32MultiArray::SharedPtr pid_target_msg_;

  // Counters
  uint64_t imu_count_{0};
  uint64_t mag_count_{0};
  uint64_t pressure_count_{0};
  uint64_t temp_count_{0};
  uint64_t joint_states_count_{0};
  uint64_t battery_count_{0};
  uint64_t system_mode_count_{0};
  uint64_t delay_count_count_{0};
  uint64_t pid_target_count_{0};

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr system_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr delay_count_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pid_target_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ESP32SerialNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("esp32_serial_node"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
