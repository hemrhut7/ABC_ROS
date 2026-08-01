# ins_ekf

ROS 2 package for INS/GNSS Fusion Navigation using an Error-State Kalman Filter (ESKF). This package is integrated from the [INS_python](file:///C:/Users/hemrh/Documents/GitHub/Python/INS_python) repository.

## 1. Features
- **Sensor Fusion:** Fuses IMU, GNSS (NavSatFix), Magnetometer, Barometer, and AGV/Odometer velocity.
- **Numba Accelerated:** High-performance math compiled at runtime with Numba.
- **Indirect EKF:** Estimates error states (position, velocity, attitude, IMU biases) and compensates the navigation state.

## 2. Package Structure
- `ins_ekf/`: ROS 2 Node interface (`ins_ekf_node.py`).
- `nav_ekf/`: Core EKF library (copied from the original project).
- `launch/`: ROS 2 launch file (`ins_ekf.launch.py`).

## 3. Installation & Building

1. Ensure the Python dependencies are installed in your active Python environment:
   ```bash
   pip install numpy scipy matplotlib pandas numba pygeomag pymavlink tqdm PyYAML
   ```
2. Build the package in your ROS 2 workspace:
   ```bash
   colcon build --packages-select ins_ekf
   ```
3. Source the workspace:
   ```bash
   # Windows PowerShell:
   .\install\setup.ps1
   # Linux (if applicable):
   source install/setup.bash
   ```

## 4. How to Run

Launch the node with default parameters:
```bash
ros2 launch ins_ekf ins_ekf.launch.py
```

## 5. Subscribed Topics
- `/imu/data_raw` (`sensor_msgs/msg/Imu`): High-frequency IMU inputs.
- `/gnss/fix` (`sensor_msgs/msg/NavSatFix`): GNSS position fix.
- `/mag/data` (`sensor_msgs/msg/MagneticField`): Magnetometer field data (if enabled).
- `/pressure` (`sensor_msgs/msg/FluidPressure`): Barometer pressure (if enabled).
- `/velocity` (`geometry_msgs/msg/TwistStamped`): Body-frame AGV speed (if enabled).

## 6. Published Topics
- `ins/odometry` (`nav_msgs/msg/Odometry`): Fused ENU position and velocity.
- `ins/pose` (`geometry_msgs/msg/PoseStamped`): Fused pose.
- `/tf`: Transform between `map` and `base_link` frames (if `publish_tf` is set to `True`).
