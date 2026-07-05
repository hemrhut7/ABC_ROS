#include "hal_microros.h"
#include "app/app_lidar.h"
#include "app/app_mode.h"

extern AppMode app_mode;

static void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  // Safely enqueue target velocity and steer to app_mode
  app_mode.enqueue_target(msg->linear.x, msg->angular.z);
}

static void cmd_mode_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  // Safely enqueue system mode to app_mode
  app_mode.enqueue_mode(static_cast<Mode_t>(msg->data));
}

#define FORCE_UNUSED(expr) do { rcl_ret_t _res = (expr); (void)_res; } while(0)

extern "C" {
  bool arduino_transport_open(struct uxrCustomTransport * transport) {
    return true;
  }

  bool arduino_transport_close(struct uxrCustomTransport * transport) {
    return true;
  }

  size_t arduino_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode) {
    (void)errcode;
    return Serial1.write(buf, len);
  }

  size_t arduino_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode) {
    (void)errcode;
    Serial1.setTimeout(timeout);
    return Serial1.readBytes((char *)buf, len);
  }
}

HAL_MicroROS::HAL_MicroROS() :
    state(WAITING_AGENT),
    uros_node(rcl_get_zero_initialized_node()),
    left_joint_pos(0.0),
    right_joint_pos(0.0),
    last_pub_time(0),
    last_ping_check(0),
    last_sync_time(0) {
    memset(&uros_support, 0, sizeof(uros_support));
    memset(&uros_executor, 0, sizeof(uros_executor));
    memset(&uros_imu_publisher, 0, sizeof(uros_imu_publisher));
    memset(&uros_joint_state_publisher, 0, sizeof(uros_joint_state_publisher));
    memset(&uros_mag_publisher, 0, sizeof(uros_mag_publisher));
    memset(&uros_baro_publisher, 0, sizeof(uros_baro_publisher));
    memset(&uros_battery_publisher, 0, sizeof(uros_battery_publisher));
    memset(&uros_scan_publisher, 0, sizeof(uros_scan_publisher));
    memset(&uros_temp_publisher, 0, sizeof(uros_temp_publisher));
    memset(&uros_mode_publisher, 0, sizeof(uros_mode_publisher));
    memset(&uros_delay_publisher, 0, sizeof(uros_delay_publisher));
    memset(&uros_pid_target_publisher, 0, sizeof(uros_pid_target_publisher));
    memset(&uros_cmd_vel_subscriber, 0, sizeof(uros_cmd_vel_subscriber));
    memset(&uros_cmd_vel_msg, 0, sizeof(uros_cmd_vel_msg));
    memset(&uros_cmd_mode_subscriber, 0, sizeof(uros_cmd_mode_subscriber));
    memset(&uros_cmd_mode_msg, 0, sizeof(uros_cmd_mode_msg));

    memset(&uros_battery_msg, 0, sizeof(uros_battery_msg));
    memset(&uros_scan_msg, 0, sizeof(uros_scan_msg));
    memset(&uros_temp_msg, 0, sizeof(uros_temp_msg));
    memset(&uros_mode_msg, 0, sizeof(uros_mode_msg));
    memset(&uros_delay_msg, 0, sizeof(uros_delay_msg));
    memset(&uros_pid_target_msg, 0, sizeof(uros_pid_target_msg));

    joint_positions[0] = 0.0;
    joint_positions[1] = 0.0;
    joint_velocities[0] = 0.0;
    joint_velocities[1] = 0.0;
    joint_efforts[0] = 0.0;
    joint_efforts[1] = 0.0;
    memset(scan_ranges, 0, sizeof(scan_ranges));
    memset(scan_intensities, 0, sizeof(scan_intensities));
    memset(pid_target_data, 0, sizeof(pid_target_data));
}

HAL_MicroROS::~HAL_MicroROS() {
    if (state == AGENT_CONNECTED) {
        destroy_node_and_publishers();
    }
}

void HAL_MicroROS::init() {
    // Initialize micro-ROS transport
    set_microros_transports();
}

bool HAL_MicroROS::init_node_and_publishers() {
  uros_allocator = rcl_get_default_allocator();

  // Create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, uros_allocator) != RCL_RET_OK) return false;

  // Initialize rclc support
  if (rclc_support_init_with_options(&uros_support, 0, NULL, &init_options, &uros_allocator) != RCL_RET_OK) {
    FORCE_UNUSED(rcl_init_options_fini(&init_options));
    return false;
  }
  FORCE_UNUSED(rcl_init_options_fini(&init_options));

  // Create node
  if (rclc_node_init_default(&uros_node, "abc_controller_node", "", &uros_support) != RCL_RET_OK) {
    FORCE_UNUSED(rclc_support_fini(&uros_support));
    return false;
  }

  // Create publishers
  if (rclc_publisher_init_default(
      &uros_imu_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu/data_raw") != RCL_RET_OK) {
    FORCE_UNUSED(rcl_node_fini(&uros_node));
    FORCE_UNUSED(rclc_support_fini(&uros_support));
    return false;
  }

  if (rclc_publisher_init_default(
      &uros_joint_state_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/joint_states") != RCL_RET_OK) {
    FORCE_UNUSED(rcl_publisher_fini(&uros_imu_publisher, &uros_node));
    FORCE_UNUSED(rcl_node_fini(&uros_node));
    FORCE_UNUSED(rclc_support_fini(&uros_support));
    return false;
  }

  if (rclc_publisher_init_default(
      &uros_mag_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
      "/imu/mag") != RCL_RET_OK) {
    FORCE_UNUSED(rcl_publisher_fini(&uros_joint_state_publisher, &uros_node));
    FORCE_UNUSED(rcl_publisher_fini(&uros_imu_publisher, &uros_node));
    FORCE_UNUSED(rcl_node_fini(&uros_node));
    FORCE_UNUSED(rclc_support_fini(&uros_support));
    return false;
  }

  if (rclc_publisher_init_default(
      &uros_baro_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
      "/baro/pressure") != RCL_RET_OK) {
    FORCE_UNUSED(rcl_publisher_fini(&uros_mag_publisher, &uros_node));
    FORCE_UNUSED(rcl_publisher_fini(&uros_joint_state_publisher, &uros_node));
    FORCE_UNUSED(rcl_publisher_fini(&uros_imu_publisher, &uros_node));
    FORCE_UNUSED(rcl_node_fini(&uros_node));
    FORCE_UNUSED(rclc_support_fini(&uros_support));
    return false;
  }

  // Create publishers for battery, scan, temp, mode, delay, pid_target
  if (rclc_publisher_init_default(
      &uros_battery_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "/battery_state") != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  if (rclc_publisher_init_default(
      &uros_scan_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
      "/scan") != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  if (rclc_publisher_init_default(
      &uros_temp_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
      "/baro/temperature") != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  if (rclc_publisher_init_default(
      &uros_mode_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/system_mode") != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  if (rclc_publisher_init_default(
      &uros_delay_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/delay_count") != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  if (rclc_publisher_init_default(
      &uros_pid_target_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/pid_target") != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  // Initialize cmd_vel subscriber
  if (rclc_subscription_init_default(
      &uros_cmd_vel_subscriber,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel") != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  // Initialize cmd_mode subscriber
  if (rclc_subscription_init_default(
      &uros_cmd_mode_subscriber,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/cmd_mode") != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  // Create executor with 2 handles (for the 2 subscriptions)
  if (rclc_executor_init(&uros_executor, &uros_support.context, 2, &uros_allocator) != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  // Add cmd_vel subscription to executor
  if (rclc_executor_add_subscription(
      &uros_executor,
      &uros_cmd_vel_subscriber,
      &uros_cmd_vel_msg,
      &cmd_vel_callback,
      ON_NEW_DATA) != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  // Add cmd_mode subscription to executor
  if (rclc_executor_add_subscription(
      &uros_executor,
      &uros_cmd_mode_subscriber,
      &uros_cmd_mode_msg,
      &cmd_mode_callback,
      ON_NEW_DATA) != RCL_RET_OK) {
    destroy_node_and_publishers();
    return false;
  }

  // Initialize messages static parts
  uros_imu_msg.header.frame_id.data = (char*)"imu_link";
  uros_imu_msg.header.frame_id.size = strlen(uros_imu_msg.header.frame_id.data);
  uros_imu_msg.header.frame_id.capacity = uros_imu_msg.header.frame_id.size + 1;

  uros_mag_msg.header.frame_id.data = (char*)"mag_link";
  uros_mag_msg.header.frame_id.size = strlen(uros_mag_msg.header.frame_id.data);
  uros_mag_msg.header.frame_id.capacity = uros_mag_msg.header.frame_id.size + 1;

  uros_baro_msg.header.frame_id.data = (char*)"baro_link";
  uros_baro_msg.header.frame_id.size = strlen(uros_baro_msg.header.frame_id.data);
  uros_baro_msg.header.frame_id.capacity = uros_baro_msg.header.frame_id.size + 1;

  uros_battery_msg.header.frame_id.data = (char*)"battery_link";
  uros_battery_msg.header.frame_id.size = strlen(uros_battery_msg.header.frame_id.data);
  uros_battery_msg.header.frame_id.capacity = uros_battery_msg.header.frame_id.size + 1;

  uros_temp_msg.header.frame_id.data = (char*)"baro_link";
  uros_temp_msg.header.frame_id.size = strlen(uros_temp_msg.header.frame_id.data);
  uros_temp_msg.header.frame_id.capacity = uros_temp_msg.header.frame_id.size + 1;

  uros_scan_msg.header.frame_id.data = (char*)"laser_link";
  uros_scan_msg.header.frame_id.size = strlen(uros_scan_msg.header.frame_id.data);
  uros_scan_msg.header.frame_id.capacity = uros_scan_msg.header.frame_id.size + 1;

  uros_scan_msg.ranges.capacity = MAX_LIDAR_POINTS;
  uros_scan_msg.ranges.size = 0;
  uros_scan_msg.ranges.data = scan_ranges;

  uros_scan_msg.intensities.capacity = MAX_LIDAR_POINTS;
  uros_scan_msg.intensities.size = 0;
  uros_scan_msg.intensities.data = scan_intensities;

  uros_pid_target_msg.data.capacity = 6;
  uros_pid_target_msg.data.size = 6;
  uros_pid_target_msg.data.data = pid_target_data;

  joint_names[0].data = (char*)"left_wheel";
  joint_names[0].size = strlen(joint_names[0].data);
  joint_names[0].capacity = joint_names[0].size + 1;
  joint_names[1].data = (char*)"right_wheel";
  joint_names[1].size = strlen(joint_names[1].data);
  joint_names[1].capacity = joint_names[1].size + 1;

  uros_joint_state_msg.name.capacity = 2;
  uros_joint_state_msg.name.size = 2;
  uros_joint_state_msg.name.data = joint_names;

  uros_joint_state_msg.position.capacity = 2;
  uros_joint_state_msg.position.size = 2;
  uros_joint_state_msg.position.data = joint_positions;

  uros_joint_state_msg.velocity.capacity = 2;
  uros_joint_state_msg.velocity.size = 2;
  uros_joint_state_msg.velocity.data = joint_velocities;

  uros_joint_state_msg.effort.capacity = 2;
  uros_joint_state_msg.effort.size = 2;
  uros_joint_state_msg.effort.data = joint_efforts;

  return true;
}

void HAL_MicroROS::destroy_node_and_publishers() {
  FORCE_UNUSED(rclc_executor_fini(&uros_executor));
  FORCE_UNUSED(rcl_subscription_fini(&uros_cmd_vel_subscriber, &uros_node));
  FORCE_UNUSED(rcl_subscription_fini(&uros_cmd_mode_subscriber, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_pid_target_publisher, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_delay_publisher, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_mode_publisher, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_temp_publisher, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_scan_publisher, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_battery_publisher, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_baro_publisher, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_mag_publisher, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_joint_state_publisher, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_imu_publisher, &uros_node));
  FORCE_UNUSED(rcl_node_fini(&uros_node));
  FORCE_UNUSED(rclc_support_fini(&uros_support));
}

static void euler_to_quaternion(float roll, float pitch, float yaw, geometry_msgs__msg__Quaternion &q) {
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
}

extern AppLidar app_lidar;

void HAL_MicroROS::update(const system_state_t &state_data) {
    switch (state) {
      case WAITING_AGENT: {
        uint32_t now_ms = millis();
        if (now_ms - last_ping_check > 1000) {
          last_ping_check = now_ms;
          if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            state = AGENT_AVAILABLE;
          }
        }
        break;
      }

      case AGENT_AVAILABLE:
        if (init_node_and_publishers()) {
          state = AGENT_CONNECTED;
          last_sync_time = millis();
          rmw_uros_sync_session(100);
        } else {
          state = WAITING_AGENT;
        }
        break;

      case AGENT_CONNECTED: {
        // Check if agent is still alive (every 2 seconds or so)
        uint32_t now_ms = millis();
        if (now_ms - last_ping_check > 2000) {
          last_ping_check = now_ms;
          if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            state = AGENT_DISCONNECTED;
            break;
          }
        }

        // Periodically sync micro-ROS clock with Agent (every 15 seconds)
        if (now_ms - last_sync_time > 15000) {
          last_sync_time = now_ms;
          rmw_uros_sync_session(10);
        }

        // Get dt for joint position accumulation
        uint32_t current_time_us = micros();
        double dt = (last_pub_time > 0) ? (current_time_us - last_pub_time) * 1e-6 : 0.01;
        last_pub_time = current_time_us;

        // Convert RPM to joint velocities (rad/s)
        double left_vel = state_data.abc_state.motor_state.rpm_L * (2.0 * M_PI / 60.0);
        double right_vel = state_data.abc_state.motor_state.rpm_R * (2.0 * M_PI / 60.0);

        // Accumulate joint positions (rad)
        left_joint_pos += left_vel * dt;
        right_joint_pos += right_vel * dt;

        // Populate Stamp Headers
        if (rmw_uros_epoch_synchronized()) {
          int64_t time_ns = rmw_uros_epoch_nanos();
          
          // Compensate for scheduling/sampling delay since data->timestamp was captured
          uint32_t delay_imu_us = current_time_us - (uint32_t)state_data.abc_state.ahrs_data.imu_data.timestamp;
          int64_t imu_time_ns = time_ns - (int64_t)delay_imu_us * 1000;
          uros_imu_msg.header.stamp.sec = imu_time_ns / 1000000000LL;
          uros_imu_msg.header.stamp.nanosec = imu_time_ns % 1000000000LL;

          uint32_t delay_mag_us = current_time_us - (uint32_t)state_data.mag_data.timestamp;
          int64_t mag_time_ns = time_ns - (int64_t)delay_mag_us * 1000;
          uros_mag_msg.header.stamp.sec = mag_time_ns / 1000000000LL;
          uros_mag_msg.header.stamp.nanosec = mag_time_ns % 1000000000LL;

          uint32_t delay_baro_us = current_time_us - (uint32_t)state_data.baro_data.timestamp;
          int64_t baro_time_ns = time_ns - (int64_t)delay_baro_us * 1000;
          uros_baro_msg.header.stamp.sec = baro_time_ns / 1000000000LL;
          uros_baro_msg.header.stamp.nanosec = baro_time_ns % 1000000000LL;
        } else {
          uros_imu_msg.header.stamp.sec = state_data.abc_state.ahrs_data.imu_data.timestamp / 1000000;
          uros_imu_msg.header.stamp.nanosec = (state_data.abc_state.ahrs_data.imu_data.timestamp % 1000000) * 1000;

          uros_mag_msg.header.stamp.sec = state_data.mag_data.timestamp / 1000000;
          uros_mag_msg.header.stamp.nanosec = (state_data.mag_data.timestamp % 1000000) * 1000;

          uros_baro_msg.header.stamp.sec = state_data.baro_data.timestamp / 1000000;
          uros_baro_msg.header.stamp.nanosec = (state_data.baro_data.timestamp % 1000000) * 1000;
        }

        // Gyroscope is in rad/s in calibrated imu_data
        uros_imu_msg.angular_velocity.x = state_data.abc_state.ahrs_data.imu_data.gyro[0];
        uros_imu_msg.angular_velocity.y = state_data.abc_state.ahrs_data.imu_data.gyro[1];
        uros_imu_msg.angular_velocity.z = state_data.abc_state.ahrs_data.imu_data.gyro[2];

        // Accelerometer is in m/s^2 in calibrated imu_data
        uros_imu_msg.linear_acceleration.x = state_data.abc_state.ahrs_data.imu_data.accl[0];
        uros_imu_msg.linear_acceleration.y = state_data.abc_state.ahrs_data.imu_data.accl[1];
        uros_imu_msg.linear_acceleration.z = state_data.abc_state.ahrs_data.imu_data.accl[2];

        // Populate orientation quaternion from Euler angles
        euler_to_quaternion(state_data.abc_state.ahrs_data.euler[0], state_data.abc_state.ahrs_data.euler[1], state_data.abc_state.ahrs_data.euler[2], uros_imu_msg.orientation);

        // Populate Magnetometer Message (convert uT to Tesla)
        uros_mag_msg.magnetic_field.x = state_data.mag_data.mag[0] * 1e-6;
        uros_mag_msg.magnetic_field.y = state_data.mag_data.mag[1] * 1e-6;
        uros_mag_msg.magnetic_field.z = state_data.mag_data.mag[2] * 1e-6;
        memset(uros_mag_msg.magnetic_field_covariance, 0, sizeof(uros_mag_msg.magnetic_field_covariance));

        // Populate Barometer Message (convert hPa to Pascals)
        uros_baro_msg.fluid_pressure = state_data.baro_data.pressure * 100.0;
        uros_baro_msg.variance = 0.0; // 0 means variance unknown

        // Populate JointState Message
        uros_joint_state_msg.header.stamp.sec = uros_imu_msg.header.stamp.sec;
        uros_joint_state_msg.header.stamp.nanosec = uros_imu_msg.header.stamp.nanosec;

        uros_joint_state_msg.position.data[0] = left_joint_pos;
        uros_joint_state_msg.position.data[1] = right_joint_pos;

        uros_joint_state_msg.velocity.data[0] = left_vel;
        uros_joint_state_msg.velocity.data[1] = right_vel;

        uros_joint_state_msg.effort.data[0] = state_data.abc_state.motor_state.pwm_out_L;
        uros_joint_state_msg.effort.data[1] = state_data.abc_state.motor_state.pwm_out_R;

        // Populate Battery State Message
        uros_battery_msg.header.stamp = uros_imu_msg.header.stamp;
        uros_battery_msg.voltage = state_data.battery_v;

        // Populate Temperature Message
        uros_temp_msg.header.stamp = uros_imu_msg.header.stamp;
        uros_temp_msg.temperature = state_data.baro_data.temperature;

        // Populate System Mode Message
        uros_mode_msg.data = state_data.cmd.mode;

        // Populate Delay Count Message
        uros_delay_msg.data = state_data.delay_count;

        // Populate PID Target Message
        pid_target_data[0] = state_data.cmd.target_value;
        pid_target_data[1] = state_data.pid_target.rpm_L;
        pid_target_data[2] = state_data.pid_target.rpm_R;
        pid_target_data[3] = state_data.pid_target.pitch;
        pid_target_data[4] = state_data.pid_target.velocity;
        pid_target_data[5] = state_data.pid_target.steer_rpm;

        // Publish Lidar Scan (if available and new)
        app_lidar.get_latest_scan(_temp_scan);
        static uint64_t last_published_scan_ts = 0;
        if (_temp_scan.count > 0 && _temp_scan.timestamp != last_published_scan_ts) {
            last_published_scan_ts = _temp_scan.timestamp;
            uros_scan_msg.header.stamp.sec = _temp_scan.timestamp / 1000000;
            uros_scan_msg.header.stamp.nanosec = (_temp_scan.timestamp % 1000000) * 1000;
            uros_scan_msg.angle_min = 0.0f;
            uros_scan_msg.angle_max = 2.0f * M_PI;
            uros_scan_msg.angle_increment = (2.0f * M_PI) / _temp_scan.count;
            uros_scan_msg.time_increment = 0.0f;
            uros_scan_msg.scan_time = 0.1f;
            uros_scan_msg.range_min = 0.12f;
            uros_scan_msg.range_max = 3.5f;

            uint16_t pt_count = _temp_scan.count;
            if (pt_count > MAX_LIDAR_POINTS) pt_count = MAX_LIDAR_POINTS;
            uros_scan_msg.ranges.size = pt_count;
            uros_scan_msg.intensities.size = pt_count;

            for (uint16_t i = 0; i < pt_count; i++) {
                scan_ranges[i] = _temp_scan.points[i].distance;
                scan_intensities[i] = (float)_temp_scan.points[i].intensity;
            }
            FORCE_UNUSED(rcl_publish(&uros_scan_publisher, &uros_scan_msg, NULL));
        }

        // Publish all messages
        FORCE_UNUSED(rcl_publish(&uros_imu_publisher, &uros_imu_msg, NULL));
        FORCE_UNUSED(rcl_publish(&uros_joint_state_publisher, &uros_joint_state_msg, NULL));
        FORCE_UNUSED(rcl_publish(&uros_mag_publisher, &uros_mag_msg, NULL));
        FORCE_UNUSED(rcl_publish(&uros_baro_publisher, &uros_baro_msg, NULL));
        FORCE_UNUSED(rcl_publish(&uros_battery_publisher, &uros_battery_msg, NULL));
        FORCE_UNUSED(rcl_publish(&uros_temp_publisher, &uros_temp_msg, NULL));
        FORCE_UNUSED(rcl_publish(&uros_mode_publisher, &uros_mode_msg, NULL));
        FORCE_UNUSED(rcl_publish(&uros_delay_publisher, &uros_delay_msg, NULL));
        FORCE_UNUSED(rcl_publish(&uros_pid_target_publisher, &uros_pid_target_msg, NULL));

        // Spin executor
        rclc_executor_spin_some(&uros_executor, RCL_MS_TO_NS(10));
        break;
      }

      case AGENT_DISCONNECTED:
        destroy_node_and_publishers();
        state = WAITING_AGENT;
        last_pub_time = 0;
        break;
    }
}
