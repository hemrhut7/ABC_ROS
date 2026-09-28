# 🚀 ROS 2 與 Docker 常用操作手冊 (USAGE.md)

本文件整理了本專案在日常開發、演算法驗證、Rosbag 測試與實體車操作時**最常使用的指令**，依使用頻率由高至低依序排列。

---

## 📑 目錄 (Table of Contents)
1. [x86_64 PC 常用開發與離線測試 (最常用)](#1-x86_64-pc-常用開發與離線測試-最常用)
   - [進入 Docker 容器](#11-進入-docker-容器)
   - [編譯工作空間](#12-編譯工作空間)
   - [VINS-Fusion + ESS + NVblox 3D 建圖測試 (經典三終端)](#13-vins-fusion--ess--nvblox-3d-建圖測試-經典三終端)
   - [常用 Launch 啟動組合 (獨立演算法)](#14-常用-launch-啟動組合-獨立演算法)
   - [Rosbag 常用重播指令](#15-rosbag-常用重播指令)
2. [實時偵錯與話題 (Topic) 監控](#2-實時偵錯與話題-topic-監控)
3. [Jetson 邊緣端實體車運行](#3-jetson-邊緣端實體車運行)
4. [Docker 容器生命週期管理](#4-docker-容器生命週期管理)
5. [常見問題快速排錯 (FAQ)](#5-常見問題快速排錯-faq)

---

## 1. x86_64 PC 常用開發與離線測試 (最常用)

### 1.1 進入 Docker 容器
專案已提供包含 GPU、GUI (X11) 與 FastDDS 網路自動配置的一鍵腳本：

```bash
# 在 Host 端專案根目錄執行 (自動掛載 GPU 與當前程式碼工作區)
./tools/run_isaac_ros_container.sh
```

進入容器後，先載入環境設定：
```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

---

### 1.2 編譯工作空間
當修改了 C++ 節點（例如 `image_splitter_node.cpp` 或 Launch 配置）時：

```bash
# 推薦：僅編譯專案相關套件 (加速編譯)
colcon build --symlink-install --packages-select my_robot_perception my_robot_bringup

# 重新載入環境
source install/setup.bash
```

---

### 1.3 VINS-Fusion + ESS + NVblox 3D 建圖測試 (經典三終端)

此組合為評估雙目深度推論與 3D 空間建圖的主要測試流。

#### 終端 1：啟動感知與建圖管線 (Launch)
```bash
# 預設使用 Light-ESS (480x288)
ros2 launch my_robot_bringup vins_ess_nvblox.launch.py use_sim_time:=true

# 若要使用標準版 Full ESS (960x576)
ros2 launch my_robot_bringup vins_ess_nvblox.launch.py \
  engine_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine \
  use_sim_time:=true

# 💡 若測試舊的 rosbag (需要自動覆蓋極線校正 CameraInfo)
ros2 launch my_robot_bringup vins_ess_nvblox.launch.py \
  use_sim_time:=true \
  override_bag_camera_info:=true
```

#### 終端 2：開啟 RViz2 視覺化介面
```bash
# 直接啟動專屬預設介面 (包含 3D Mesh、LiDAR 點雲、VIO 軌跡與彩色點雲)
rviz2 -d src/my_robot_bringup/rviz/vins_ess_nvblox.rviz
```

#### 終端 3：播放測試 Rosbag
```bash
# 播放指定 Rosbag (務必加上 --clock)
ros2 bag play ros2_bag/rosbag2_total_20260903_203935 --clock

# 減速播放 (0.5倍速，便於詳細觀察建圖過程)
ros2 bag play ros2_bag/rosbag2_total_20260903_203935 --clock -r 0.5
```

---

### 1.4 常用 Launch 啟動組合 (獨立演算法)

* **單獨測試 Isaac ROS ESS 深度推論：**
  ```bash
  ros2 launch my_robot_bringup isaac_ess.launch.py use_sim_time:=true
  ```

* **單獨測試 VINS-Fusion VIO (雙目 + IMU)：**
  ```bash
  ros2 launch my_robot_bringup vins_fusion.launch.py use_sim_time:=true
  # 視覺化
  rviz2 -d src/my_robot_bringup/rviz/vins_fusion.rviz
  ```

* **單獨測試 NVIDIA cuVSLAM (純雙目 / 視覺里程計)：**
  ```bash
  ros2 launch my_robot_bringup isaac_visual_slam.launch.py use_sim_time:=true enable_imu_fusion:=false
  # 視覺化
  rviz2 -d src/my_robot_bringup/rviz/isaac_visual_slam.rviz
  ```

---

### 1.5 Rosbag 常用重播指令

```bash
# 查看 Bag 資訊 (主題、幀率、時長、訊息數量)
ros2 bag info ros2_bag/rosbag2_total_20260903_203935

# 循環播放 (適合演算法調參)
ros2 bag play ros2_bag/rosbag2_total_20260903_203935 --clock -l

# 指定從第 5 秒開始播放，持續播放 10 秒
ros2 bag play ros2_bag/rosbag2_total_20260903_203935 --clock --start-offset 5 --duration 10
```

---

## 2. 實時偵錯與話題 (Topic) 監控

在容器內或本機終端中隨時檢查系統運行狀態：

```bash
# 查看所有節點清單
ros2 node list

# 查看特定主題的發布頻率 (Hz)
ros2 topic hz /stereo/depth
ros2 topic hz /nvblox_node/mesh
ros2 topic hz /odometry

# 監控 VINS-Fusion 當前里程計座標數值
ros2 topic echo /odometry --flow-style

# 檢查 TF 座標樹連線是否完整 (輸出 frames.pdf)
ros2 run tf2_tools view_frames

# 儲存 NVblox 累積建立好的 3D Mesh (PLV 格式，可用 MeshLab/CloudCompare 打開)
ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath "{file_path: '/workspaces/isaac_ros-dev/output/map.ply'}"
```

---

## 3. Jetson 邊緣端實體車運行

當登入實體車上的 NVIDIA Jetson 時使用：

### 3.1 啟動 Jetson 專用 Docker 容器
```bash
# 啟動包含硬體權限與 VPI 3 的容器
jetson-containers run \
  --privileged \
  -v /dev:/dev \
  -v ~/ROS_ABC:/workspace \
  -v /opt/nvidia/vpi3:/opt/nvidia/vpi3 \
  -e CMAKE_PREFIX_PATH=/opt/ros/humble:/opt/nvidia/vpi3/lib/aarch64-linux-gnu/cmake/vpi \
  -e vpi_DIR=/opt/nvidia/vpi3/lib/aarch64-linux-gnu/cmake/vpi \
  ros:humble-ros-base-hank bash
```

### 3.2 實體機器人一鍵啟動
```bash
# 進入容器後執行：
source /workspace/install/setup.bash

# 一鍵啟動 micro-ROS Agent、雙目相機切割節點、LiDAR 節點與底盤遙測
ros2 launch my_robot_bringup robot.launch.py

# 若需同時自動錄製 Rosbag (模式可選: esp32, lidar, camera, total)
ros2 launch my_robot_bringup robot.launch.py auto_log:=true mode:=total
```

### 3.3 搭配演算法在實體車運行
* **實體車運行 VINS-Fusion + ESS + NVblox：**
  ```bash
  ros2 launch my_robot_bringup vins_ess_nvblox.launch.py
  ```

---

## 4. Docker 容器生命週期管理

### 4.1 進入已在運行的容器 (另開終端)
如果容器已經在執行，不要重複啟動新的，直接附載進去：

```bash
# 1. 查詢正在運行的容器名稱或 ID
docker ps

# 2. 開啟新的 Bash Shell 進入該容器
docker exec -it <CONTAINER_NAME_OR_ID> bash
```

### 4.2 背景分離與重新連接
* **暫時分離 (Detach)**：按住 `<kbd>Ctrl</kbd> + <kbd>P</kbd>`，接著按 `<kbd>Ctrl</kbd> + <kbd>Q</kbd>`，容器會保持在背景運行。
* **重新連接 (Attach)**：
  ```bash
  docker attach <CONTAINER_NAME_OR_ID>
  ```

### 4.3 關閉容器
在容器內直接輸入 `exit` 或按下 `<kbd>Ctrl</kbd> + <kbd>D</kbd>`。

---

## 5. 常見問題快速排錯 (FAQ)

### Q1: `rviz2` 提示 `Could not connect to display` 或畫面打不開？
在 **Host 主機端**執行允許 root 存取 X11 權限：
```bash
xhost +local:root
```

### Q2: Host 端與 Docker 容器內互相收不到 Topic (`ros2 topic list` 為空)？
這是 FastDDS 共享記憶體 (SHM) 權限隔離問題。確保啟動容器時帶有以下環境變數（`run_isaac_ros_container.sh` 已內建）：
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml
```

### Q3: ESS 切換 `ess.engine` 後提示維度錯誤？
- **Light-ESS (`light_ess.engine`)**：輸入維度為 **$480 \times 288$**。
- **Full ESS (`ess.engine`)**：輸入維度為 **$960 \times 576$**。
Launch 檔現已支援自動偵測，無須手動指定解析度。

### Q4: 舊的 Rosbag 深度圖全空白？
舊 Bag 錄製時缺少雙目共面極線旋轉矩陣 ($R=I_3$)，啟動時請加上覆蓋參數：
```bash
ros2 launch my_robot_bringup vins_ess_nvblox.launch.py use_sim_time:=true override_bag_camera_info:=true
```
