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

#include <chrono>
#include <memory>
#include <functional>

class ESP32SerialNode : public rclcpp::Node {
public:
  ESP32SerialNode()
  : Node("esp32_serial_node"),
    is_connected_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Starting ESP32 Serial Telemetry Node (status logs on connect/disconnect)");

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

    // Watchdog timer to detect connection loss (1 sec interval)
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ESP32SerialNode::check_connection_watchdog, this));
  }

  ~ESP32SerialNode() override = default;

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
  void update_connection_status() {
    last_msg_time_ = std::chrono::steady_clock::now();
    if (!is_connected_) {
      is_connected_ = true;
      RCLCPP_INFO(this->get_logger(), "ESP32 serial telemetry connection: ESTABLISHED (receiving data)");
    }
  }

  void check_connection_watchdog() {
    if (is_connected_) {
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - last_msg_time_).count();
      if (elapsed >= 3) {
        is_connected_ = false;
        RCLCPP_WARN(this->get_logger(), "ESP32 serial telemetry connection: LOST (no data for >3s)");
      }
    }
  }

  // Callbacks
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_msg_ = msg;
    update_connection_status();
  }

  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    mag_msg_ = msg;
    update_connection_status();
  }

  void pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    pressure_msg_ = msg;
    update_connection_status();
  }

  void temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
    temp_msg_ = msg;
    update_connection_status();
  }

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_states_msg_ = msg;
    update_connection_status();
  }

  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    battery_msg_ = msg;
    update_connection_status();
  }

  void system_mode_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    system_mode_msg_ = msg;
    update_connection_status();
  }

  void delay_count_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    delay_count_msg_ = msg;
    update_connection_status();
  }

  void pid_target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    pid_target_msg_ = msg;
    update_connection_status();
  }

  // Connection state
  bool is_connected_;
  std::chrono::steady_clock::time_point last_msg_time_;

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

  // Watchdog timer
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
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
