# 🤖 My Robot ROS 2 Workspace (ABC_ROS)

本專案為一個基於 **ROS 2 (Humble)** 系統的機器人控制與感測開發工作空間。
主要運行於 **NVIDIA Jetson** 邊緣運算平台，並結合 **ESP32 微控制器**（運行 micro-ROS）以及**雙目 (Stereo) 相機**，實現機器人的即時遙測監控、動力控制與視覺感知。

---

## 🏗️ 系統架構圖 (System Architecture)

以下為專案的硬體與軟體資料流拓撲圖，展現了感測器數據上傳與控制命令下發的流程：

```mermaid
graph TD
    subgraph ESP32 [ESP32 微控制器端]
        MC_IMU[IMU 數據] -->|/imu/data_raw| MC_uROS(micro-ROS Client)
        MC_MAG[磁力計] -->|/imu/mag| MC_uROS
        MC_BARO[氣壓計] -->|/baro/pressure| MC_uROS
        MC_TEMP[溫度計] -->|/baro/temperature| MC_uROS
        MC_JOINTS[輪胎編碼器] -->|/joint_states| MC_uROS
        MC_BATT[電池電壓] -->|/battery_state| MC_uROS
        MC_LIDAR[雷達] -->|/scan| MC_uROS
        MC_PID[PID 目標資訊] -->|/pid_target| MC_uROS
        MC_uROS -->|執行馬達動作| Motors[左/右馬達 PWM]
    end

    subgraph Jetson [NVIDIA Jetson 主機端 (Docker Container)]
        uROS_Agent(micro_ros_agent Node) <-->|Serial 通訊 /dev/ttyTHS1 或 /dev/ttyUSB0| MC_uROS
        
        %% 遙測監控節點
        uROS_Agent -->|感測器 Topic 轉發| TelemetryNode(esp32_serial_node.py)
        
        %% 控制命令發送
        TelemetryNode -->|/cmd_vel & /cmd_mode| uROS_Agent
        
        %% 雙目視覺節點
        Cam[雙目相機 Hardware] -->|USB MJPEG 串流| SplitterNode(image_splitter_node.py)
        SplitterNode -->|發布左右/立體影像| StereoTopics[影像主題 /camera/...]
    end

    subgraph PC [Windows PC 監控端]
        FoxgloveBridge(foxglove_bridge Node) <-->|WebSocket 連線 ws://Jetson_IP:8765| FoxgloveStudio[Foxglove Studio 視覺化軟體]
        StereoTopics --> FoxgloveBridge
        TelemetryNode --> FoxgloveBridge
    end
```

---

## 📂 專案套件結構說明

本工作空間包含三大核心 ROS 2 套件，您可以使用以下連結直接瀏覽與編輯對應檔案：

1. **[my_robot_bringup](src/my_robot_bringup)**
   - **功能描述**：負責整個機器人系統的一鍵啟動與全域參數管理。
   - **關鍵檔案**：
     - [robot.launch.py](src/my_robot_bringup/launch/robot.launch.py)：一鍵啟動 micro-ROS Agent、遙測節點與雙目相機影像切割節點。
     - [params.yaml](src/my_robot_bringup/config/params.yaml)：包含相機解析度、FPS、串流裝置路徑以及遙測更新頻率等參數。

2. **[my_robot_firmware](src/my_robot_firmware)**
   - **功能描述**：負責對接微控制器底層的串流與遙測數據處理。
   - **關鍵檔案**：
     - [esp32_serial_node.py](src/my_robot_firmware/my_robot_firmware/esp32_serial_node.py)：訂閱 ESP32 發布的 IMU、姿態、馬達編碼器等 Topic，並在終端機渲染出即時的 CLI 遙測儀表板。同時提供向 ESP32 發送速度與模式指令的 API。
     - [hal_microros.cpp](src/my_robot_firmware/ref/hal_microros.cpp)：ESP32 端的 micro-ROS 韌體底層參考程式碼，展示如何與 Jetson 端進行資料對接。

3. **[my_robot_perception](src/my_robot_perception)**
   - **功能描述**：負責雙目相機影像的擷取、切割與壓縮傳輸。
   - **關鍵檔案**：
     - [image_splitter_node.py](src/my_robot_perception/my_robot_perception/image_splitter_node.py)：讀取雙目廣角相機的 2560x720 影像，切割為左右兩張 1280x720 影像，並同時發布 Raw 與 Compressed 格式，以及 CameraInfo 校正資訊，降低網路頻寬消耗。

4. **[isaac_ros (cuVSLAM)](src/isaac_ros)**
   - **功能描述**：NVIDIA 硬體加速雙目視覺里程計與特徵點雲建圖。由專案根目錄之 [`isaac_ros.repos`](isaac_ros.repos) 統一進行版本宣告管理（NVIDIA 官方 `v3.2.0`）。
   - **關鍵模組**：
     - [isaac_ros_visual_slam](src/isaac_ros/isaac_ros_visual_slam)：cuVSLAM 核心節點，支援雙目視覺里程計與 IMU 融合（VIO）。
     - [isaac_ros_nitros](src/isaac_ros/isaac_ros_nitros)：NVIDIA GXF 底層零拷貝記憶體與硬體加速傳輸框架。
     - [isaac_ros_common](src/isaac_ros/isaac_ros_common)：Isaac ROS 通用 Launch 與測試工具鏈。
     - [isaac_ros_nvblox](src/isaac_ros/isaac_ros_nvblox)：GPU 3D 稠密即時建圖與障礙物重建。

---

## 📦 第三方套件版本控管 (Isaac ROS Repos 管理)

本專案採用 ROS 2 官方標準工具 `vcstool` 管理所有大型第三方 Isaac ROS 套件，主 Repository 僅追蹤宣告式設定檔 [`isaac_ros.repos`](isaac_ros.repos)，保持工作區輕量：

### 1. 下載 / 匯入所有 Isaac ROS 套件
在新環境或新電腦上，只需在專案根目錄執行以下指令，即可依據 `isaac_ros.repos` 自動抓取鎖定之 `v3.2.0` 版本：
```bash
vcs import < isaac_ros.repos
```

### 2. 驗證所有來源與版本
```bash
vcs validate < isaac_ros.repos
```

### 3. 查看當前所有套件狀態
```bash
vcs status
```

---

## 🔨 環境建置與執行教學 (Docker 容器環境)

本專案之主控程式運行於 NVIDIA Jetson 之 Docker 容器環境中，以確保與 Host 硬體（V4L2 相機、Serial 序列埠）的順暢串接。專案根目錄已提供完整且透明化的 [`Dockerfile`](file:///home/hank/ROS_ABC/Dockerfile)，整合了所有系統依賴與軟體庫。

### Step 1: 建置 Docker 鏡像 (Build Docker Image)
如需從頭建置或更新開發容器鏡像，請在 Host 端專案根目錄（`~/ROS_ABC`）執行以下建置指令：

```bash
cd ~/ROS_ABC

# 使用專案根目錄的 Dockerfile 建置鏡像
docker build -t ros:humble-ros-base-hank -f Dockerfile .
```

> [!NOTE]
> **Dockerfile 整合內容**：
> - **基底鏡像**：繼承官方 `ros:humble-ros-base-l4t-r36.5.0`。
> - **系統與 ROS 2 依賴**：自動安裝 `libtbb-dev`, `ros-humble-cv-bridge`, `ros-humble-tf2-ros`, `ros-humble-sensor-msgs`, `ros-humble-nav-msgs`, `ros-humble-geometry-msgs`, `ros-humble-cartographer-ros`。
> - **共享庫自動修復**：內建自動補全 `libtbb.so.2` 符號連結，修復 OpenCV TBB 2020 符號尋找錯誤 (`symbol lookup error`)。
> - **Python 演算法加速依賴**：自動安裝 `numba` (JIT 加速), `scipy`, `numpy`, `matplotlib`, `pandas`, `tqdm`, `pygeomag`, `pymavlink`, `pyyaml` 等 `ins_ekf` 與 `INS_python` 所需演算法庫。

### Step 2: 啟動 Jetson 專屬 ROS 2 容器
我們使用 `jetson-containers` 工具鏈，並掛載專案資料夾與設備驅動權限。請在 Jetson Host 端執行：

```bash
jetson-containers run \
  --privileged \
  -v /dev:/dev \
  -v ~/ROS_ABC:/workspace \
  $(autotag ros:humble-ros-base)
```

```bash
jetson-containers run \
  --privileged \
  -v /dev:/dev \
  -v ~/ROS_ABC:/workspace \
  -v /opt/nvidia/vpi3:/opt/nvidia/vpi3 \
  -e CMAKE_PREFIX_PATH=/opt/ros/humble:/opt/nvidia/vpi3/lib/aarch64-linux-gnu/cmake/vpi \
  -e vpi_DIR=/opt/nvidia/vpi3/lib/aarch64-linux-gnu/cmake/vpi \
  ros:humble-ros-base-hank bash
```

> [!NOTE]
> - `--privileged`：提供容器直接讀寫 Host 硬體（如 USB 轉序列埠 `/dev/ttyUSB*`、GPIO UART `/dev/ttyTHS*` 以及 USB 相機 `/dev/video*`）的權限。
> - `-v ~/ROS_ABC:/workspace`：將專案目錄掛載至容器內的 `/workspace` 路徑，實現即時程式修改與持續編譯。
> - `$(autotag ros:humble-ros-base)`：自動解析並優先調用本機建立好的 `ros:humble-ros-base-hank` 鏡像。

### Step 3: 在容器內編譯工作空間
進入容器終端機後，執行以下命令進行 Colcon 編譯：

```bash
# 切換至工作空間根目錄
cd /workspace

# 編譯所有套件（啟用軟連結安裝，方便開發調試 Python 節點）
colcon build --symlink-install
```

> [!TIP]
> 由於在編譯時使用了 `--symlink-install` 參數，Python 程式的變更會即時生效，您**不需要**重新執行 `colcon build`！

### Step 4: 載入環境變數
編譯完成後，必須將工作空間的環境設定檔載入當前 Shell：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Step 5: 一鍵啟動機器人系統
使用 Launch 檔案同時運行 micro-ROS Agent（使用 `/dev/ttyTHS1` GPIO 串列、鮑率 2000000）、遙測監控節點與相機影像切割節點：

```bash
ros2 launch my_robot_bringup robot.launch.py
```

或者啟動單一節點
```bash
ros2 run my_robot_perception image_splitter_node
```

---

## 👁️ NVIDIA cuVSLAM (Visual SLAM) 啟動與驗證指引

專案已整合 **NVIDIA Isaac ROS cuVSLAM**（`isaac_ros_visual_slam`），可利用 Jetson Orin Nano 的 GPU 與 VPI 3 硬體加速實現高幀率的雙目視覺里程計（Visual Odometry）與 SLAM 地圖特徵建置。

### 1. 啟動包含 GPU 與 VPI 3 的 Docker 容器
在 Jetson Host 端執行以下指令（需掛載 VPI 3 與設備驅動）：

```bash
docker run -it --rm --runtime nvidia \
  --net=host \
  --privileged \
  -v /dev:/dev \
  -v ~/ROS_ABC:/workspace \
  -v /opt/nvidia/vpi3:/opt/nvidia/vpi3 \
  -e CMAKE_PREFIX_PATH=/opt/ros/humble:/opt/nvidia/vpi3/lib/aarch64-linux-gnu/cmake/vpi \
  -e vpi_DIR=/opt/nvidia/vpi3/lib/aarch64-linux-gnu/cmake/vpi \
  ros:humble-ros-base-hank bash
```

進入容器後載入工作空間：
```bash
source /opt/ros/humble/install/setup.bash
source /workspace/install/setup.bash
```

### 2. 搭配機器人現有硬體 (`my_robot_bringup`) 一鍵啟動

* **終端 A（啟動機器人硬體與雙目相機影像切割）：**
  ```bash
  ros2 launch my_robot_bringup robot.launch.py
  ```

* **終端 B（一鍵啟動 cuVSLAM 視覺里程計，預設純雙目+地面約束，最平穩無漂移）：**
  ```bash
  ros2 launch my_robot_bringup isaac_visual_slam.launch.py enable_imu_fusion:=false enable_ground_constraint:=true
  ```

> [!TIP]
> - **純雙目立體模式（推薦）**：設定 `enable_imu_fusion:=false`，直接利用雙目 51.912mm 物理基線提供真實尺度，完全不受 USB/序列埠異步時延影響。
> - **地面 2D 約束**：設定 `enable_ground_constraint:=true`，可將機器人姿態約束於水平地面，消除 Z 軸高度漂移。
> - **IMU 融合模式**：若要測試 IMU 融合，設定 `enable_imu_fusion:=true`（Launch 檔已自動套用 ICM-20948 專用雜訊矩陣參數）。

### 3. Rosbag 離線資料回放驗證

如需使用錄製好的 Rosbag 進行離線演算法評估與軌跡回放：

* **終端 A（啟動 cuVSLAM 並啟用模擬時鐘）：**
  ```bash
  ros2 launch my_robot_bringup isaac_visual_slam.launch.py use_sim_time:=true enable_imu_fusion:=false enable_ground_constraint:=true
  ```

* **終端 B（回放指定 Rosbag）：**
  ```bash
  ros2 bag play <rosbag資料夾路徑> --clock
  ```

### 4. 關鍵主題 (Topics) 列表

| 類型 | 主題名稱 | 訊息格式 | 說明 |
| :--- | :--- | :--- | :--- |
| **輸入** | `visual_slam/image_0`, `image_1` | `sensor_msgs/msg/Image` | 左 / 右鏡頭影像輸入 (`mono8`) |
| **輸入** | `visual_slam/camera_info_0`, `1` | `sensor_msgs/msg/CameraInfo` | 左 / 右鏡頭相機標定內參與基線 |
| **輸入** | `visual_slam/imu` | `sensor_msgs/msg/Imu` | IMU 姿態/加速度數據 (選填) |
| **輸出** | `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | 即時估算的機器人位姿與速度 |
| **輸出** | `/visual_slam/tracking/vo_path` | `nav_msgs/msg/Path` | 運動歷史軌跡 Path |
| **輸出** | `/visual_slam/vis/landmarks_cloud`| `sensor_msgs/msg/PointCloud2` | 3D 特徵點雲（視覺化地圖） |
| **輸出** | `/visual_slam/vis/observations_cloud`| `sensor_msgs/msg/PointCloud2` | 即時觀測特徵點雲 |
| **輸出** | `/tf` | `tf2_msgs/msg/TFMessage` | `odom` $\rightarrow$ `base_link` 坐標變換 |

### 5. PC 端 RViz2 遠端視覺化監控

強烈建議將 RViz2 運行在同區域網路的 **PC 端**（保持 Jetson 端無頭運行，以節省 GPU/記憶體頻寬）：

1. 確保 PC 與 Jetson 設定相同的 Domain ID：`export ROS_DOMAIN_ID=0`
2. 在 PC 端載入工作空間並一鍵開啟專屬配置：
   ```bash
   rviz2 -d $(ros2 pkg prefix my_robot_bringup)/share/my_robot_bringup/rviz/isaac_visual_slam.rviz
   ```
   *(或手動將 Fixed Frame 設為 `odom`，並訂閱 `/visual_slam/tracking/odometry` 與 `/visual_slam/vis/landmarks_cloud`)*

---

## 🧠 NVIDIA Isaac ROS ESS (DNN Stereo Depth Estimation) 啟動與驗證指引

專案已完整整合並編譯 **NVIDIA Isaac ROS ESS**（`isaac_ros_ess` 與 `isaac_ros_stereo_image_proc`），利用 Jetson Orin Nano 的 TensorRT 加速雙目立體神經網路，實現高幀率（>150 FPS）、高精度的視差圖（Disparity）與度量深度圖（Depth Map）估算。

### 1. 深度推論模型規格 (TensorRT FP16)
- **Light ESS Engine (`light_ess.engine`)**：
  - 輸入解析度：`480 x 288`
  - 推論延遲：**6.70 ms**
  - 運算吞吐量：**154.5 FPS**（推薦用於機器人即時建圖與避障）
- **Full ESS Engine (`ess.engine`)**：
  - 輸入解析度：`960 x 576`
  - 推論延遲：**23.6 ms**
  - 運算吞吐量：**43.8 FPS**

### 2. 即時連線啟動 ESS 深度管線
搭配實體雙目相機（`image_splitter_node`）即時推論：

* **終端 A（啟動機器人硬體與雙目相機串流）：**
  ```bash
  ros2 launch my_robot_bringup robot.launch.py
  ```

* **終端 B（一鍵啟動 ESS 視差與深度圖推論節點）：**
  ```bash
  ros2 launch my_robot_bringup isaac_ess.launch.py
  ```

### 3. Rosbag 離線資料回放驗證
使用錄製好的 Rosbag 回放進行深度神經網路推論驗證：

* **終端 A（啟動 ESS 深度推論節點並啟用模擬時鐘）：**
  ```bash
  ros2 launch my_robot_bringup isaac_ess.launch.py use_sim_time:=true
  ```

* **終端 B（回放指定 Rosbag）：**
  ```bash
  ros2 bag play <rosbag資料夾路徑> --clock
  ```

### 4. 關鍵深度主題 (Topics) 列表

| 類型 | 主題名稱 | 訊息格式 | 說明 |
| :--- | :--- | :--- | :--- |
| **輸入** | `/camera/left/image_raw`, `/camera/right/image_raw` | `sensor_msgs/msg/Image` | 左 / 右鏡頭影像（支援 `mono8` 與 `rgb8`） |
| **輸入** | `/camera/left/camera_info`, `/camera/right/camera_info` | `sensor_msgs/msg/CameraInfo` | 雙目相機標定內參與基線 |
| **輸出** | `/stereo/disparity` | `stereo_msgs/msg/DisparityImage` | ESS 輸出的 GPU 視差圖 |
| **輸出** | `/stereo/depth` | `sensor_msgs/msg/Image` (`32FC1`) | 轉換後的公尺度量深度圖（Metric Depth Map） |


---

## 🧭 VINS-Fusion (Stereo + IMU) 啟動與驗證指引

專案已完整整合並支援在 Docker 容器內編譯與運行 **VINS-Fusion**，適用於本車雙目立體相機與 ESP32 (ICM-20948) IMU 數據。

### 1. 啟動 VINS-Fusion 節點 (即時或 Rosbag 模式)

* **離線 Rosbag 回放模式 (使用模擬時間)：**
  ```bash
  ros2 launch my_robot_bringup vins_fusion.launch.py use_sim_time:=true
  ```

* **實體機器人即時模式：**
  ```bash
  ros2 launch my_robot_bringup vins_fusion.launch.py
  ```

### 2. 關鍵主題 (Topics) 列表

| 類型 | 主題名稱 | 訊息格式 | 說明 |
| :--- | :--- | :--- | :--- |
| **輸入** | `/camera/left/image_mono`, `/camera/right/image_mono` | `sensor_msgs/msg/Image` | 左 / 右鏡頭灰階影像 (`640x480 mono8`) |
| **輸入** | `/imu/data_raw` | `sensor_msgs/msg/Imu` | ESP32 IMU 加速度與角速度數據 |
| **輸出** | `/odometry` | `nav_msgs/msg/Odometry` | VIO 即時估算之里程計位姿與速度 |
| **輸出** | `/path` | `nav_msgs/msg/Path` | 估算之完整運動軌跡 |
| **輸出** | `/image_track` | `sensor_msgs/msg/Image` | 帶有特徵點追蹤可視化的左右影像 |
---

## 🧊 VINS-Fusion + ESS + NVblox 整合感知與稠密建圖架構

本專案將 **VINS-Fusion（雙目+IMU 視覺慣性里程計）**、**NVIDIA Isaac ROS ESS（雙目度量深度圖推論）** 與 **NVIDIA Isaac ROS NVblox（GPU 3D TSDF 重建與 2D ESDF/代價地圖生成）** 三大核心管線深度整合，實現機器人即時自主導航所需的環境感知與稠密建圖能力。

### 1. 系統資料流架構

* **狀態估算（VINS-Fusion）**：訂閱雙目灰階影像與 ESP32 IMU，輸出平滑高頻的 `/odometry` 與動態坐標變換 `world -> base_link`。
* **雙目校正與深度估算（Isaac ROS ESS）**：先經由硬體加速 `RectifyNode` 消除鏡頭畸變並將極線嚴格水平對齊，再送入 TensorRT Light ESS 模型輸出公尺級度量深度圖 `/stereo/depth`。
* **GPU 稠密建圖（Isaac ROS NVblox）**：在 GPU 內透過 TSDF 體素網格（Voxel Size: 5cm）即時融合深度圖與 VIO 位姿，輸出立體 3D 著色網格面（`/nvblox_node/mesh`）與 2D 避障代價地圖（`/nvblox_node/static_occupancy_grid`）。

### 2. 一鍵啟動完整架構

* **離線 Rosbag 回放驗證模式（使用模擬時間）：**
  ```bash
  ros2 launch my_robot_bringup vins_ess_nvblox.launch.py use_sim_time:=true
  ```

* **實體機器人即時模式：**
  ```bash
  ros2 launch my_robot_bringup vins_ess_nvblox.launch.py
  ```

* **同時開啟專屬 RViz2 視覺化介面（監控 3D Mesh、VIO 軌跡與 2D 代價地圖）：**
  ```bash
  ros2 launch my_robot_bringup vins_ess_nvblox.launch.py use_sim_time:=true rviz:=true
  ```

### 3. 關鍵主題 (Topics) 列表

| 模組 | 主題名稱 | 訊息格式 | 說明 |
| :--- | :--- | :--- | :--- |
| **VIO** | `/odometry` | `nav_msgs/msg/Odometry` | 機器人世界坐標即時位姿與速度 |
| **VIO** | `/path` | `nav_msgs/msg/Path` | 機器人運動軌跡 |
| **深度感知** | `/stereo/depth` | `sensor_msgs/msg/Image` (`32FC1`) | ESS 輸出的公尺級度量深度圖 |
| **深度感知** | `/camera/left/image_rect` | `sensor_msgs/msg/Image` | GPU 硬體校正後的左眼彩色影像 |
| **3D 建圖** | `/nvblox_node/mesh` | `nvblox_msgs/msg/Mesh` | 即時更新的 3D 著色重建網格面 |
| **2D 導航** | `/nvblox_node/static_occupancy_grid` | `nav_msgs/msg/OccupancyGrid` | 2D 障礙物切片（可直接供 Nav2 導航避障使用） |
| **點雲輸出** | `/nvblox_node/static_esdf_pointcloud` | `sensor_msgs/msg/PointCloud2` | 靜態 ESDF 距離場點雲 |

---

## 💻 x86_64 PC 開發與 Rosbag 回放快速指引 (一鍵腳本)

本專案提供專屬腳本與 Dockerfile，可直接在配備 NVIDIA 顯示卡的 x86_64 PC 上啟動 Isaac ROS 與演算法回放：

### 1. 一鍵啟動容器 (自動掛載 GPU、X11 顯示與工作空間)
```bash
./tools/run_isaac_ros_container.sh
```

### 2. 在容器終端中執行演算法節點 (終端 A)
* **執行 cuVSLAM：**
  ```bash
  ros2 launch my_robot_bringup isaac_visual_slam.launch.py use_sim_time:=true enable_imu_fusion:=false
  ```
* **或執行 ESS 深度推論：**
  ```bash
  ros2 launch my_robot_bringup isaac_ess.launch.py use_sim_time:=true
  ```
* **或執行 VINS-Fusion：**
  ```bash
  ros2 launch my_robot_bringup vins_fusion.launch.py use_sim_time:=true
  ```

### 3. 另開終端回放 Rosbag (終端 B)
```bash
./tools/run_isaac_ros_container.sh
ros2 bag play ros2_bag/rosbag2_total_20260903_203935 --clock
```

### 4. 開啟 RViz2 進行即時可視化 (終端 C)
```bash
./tools/run_isaac_ros_container.sh
# 依據執行的套件選擇對應設定檔：
rviz2 -d src/my_robot_bringup/rviz/isaac_visual_slam.rviz
# 或
rviz2 -d src/my_robot_bringup/rviz/vins_fusion.rviz
```

---
## 🛡️ 核心維運與安全指南

### 2. 獨立啟動 micro-ROS Agent (數據偵錯與硬體切換)
如果需要測試不同的硬體連線模式（例如從 GPIO UART 切換至 USB 連接），可以手動啟動 micro-ROS Agent。

#### 模式 A：USB 連接 (常見於 ESP32 開發板透過 MicroUSB/Type-C 連接 Jetson)
```bash
docker run -it --rm --net=host --privileged -v /dev:/dev microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 921600
```

#### 模式 B：GPIO UART 連接 (常見於直連 Jetson 的 40-Pin UART 接口，推薦)
```bash
docker run -it --rm --net=host --privileged -v /dev:/dev microros/micro-ros-agent:humble serial --dev /dev/ttyTHS1 -b 2000000
```

> [!IMPORTANT]
> 鮑率 (Baud Rate) 參數 `-b` 必須與 ESP32 韌體中的設定相符。若使用 `my_robot_bringup` 中的一鍵啟動，請至 [params.yaml](src/my_robot_bringup/config/params.yaml) 或 [robot.launch.py](src/my_robot_bringup/launch/robot.launch.py) 中調整對應參數。

---

### 3. Docker 容器生命週期管理

#### 退出與關閉容器
如果您已完成測試，想要直接關閉並退出容器：
- **指令方式**：在容器內輸入 `exit` 並按下 `<kbd>Enter</kbd>`。
- **快捷鍵方式**：直接按下 `<kbd>Ctrl</kbd> + <kbd>D</kbd>`。

> [!NOTE]
> 由於 `jetson-containers run` 啟動參數預設帶有 `--rm`（自動刪除），使用上述方式退出時，該容器實例將會被自動銷毀並釋放記憶體。
> 
> **程式碼安全保障**：您在容器內 `/workspace` 目錄下修改的任何程式碼，皆已同步映射至本機的 `~/ROS_ABC`，檔案絕不會遺失。

#### 暫時分離 (Detach) 容器 (讓程式在背景繼續運行)
如果您在容器中啟動了 micro-ROS Agent 或平衡車控制節點，不希望因為關閉終端機或 SSH 斷線而導致程式中斷，可以將容器移至背景運行：
- **快捷鍵方式**：先按住 `<kbd>Ctrl</kbd> + <kbd>P</kbd>`，接著按 `<kbd>Ctrl</kbd> + <kbd>Q</kbd>`。

此時終端機會返回到 Host 主控端環境（例如 `hank@hank-desktop:~$`），但該 Docker 容器仍在背景持續運作。

#### 🔄 如何重新附載 (Attach) 回背景運行的容器？
1. 先查詢正在運作的容器 ID 或名稱：
   ```bash
   docker ps
   ```
2. 使用 `attach` 指令重新連接至該容器畫面：
   ```bash
   docker attach <CONTAINER_ID_OR_NAME>
   ```

---

## 🛠️ 自定義開發與調整指引

### 1. 修改節點參數
您可以直接調整 [params.yaml](src/my_robot_bringup/config/params.yaml) 檔：
```yaml
image_splitter_node:
  ros__parameters:
    video_device: 0       # USB 相機裝置代號 (/dev/video0)
    width: 2560           # 雙目相機的總寬度解析度
    height: 720           # 雙目相機的總高度解析度
    fps: 30               # 擷取更新率
    frame_id: "camera_link"

esp32_serial_node:
  ros__parameters:
    display_rate: 1.0     # 終端機遙測 CLI 儀表板的更新頻率 (Hz)
```

---

## ⚡ Jetson 常用硬體效能控制指令

### 查看系統效能與資源佔用率
在 Host 終端機執行 `jtop`（由 `jetson-stats` 提供），可即時查看 CPU、GPU 負載、記憶體與晶片溫度：
```bash
jtop
```

### 檢視當前功率效能模式
```bash
sudo nvpmodel -q
```

### 切換至最高效能模式 (Max Performance Mode)
```bash
sudo nvpmodel -m 0
```

### 切換資料夾的使用者權限
```bash
sudo chown -R hank:hank ros2_bag/
```

---

## 🚀 x86_64 PC (RTX 3060) 深度學習與 SLAM 評估 (Isaac ROS & VINS-Fusion)

專案提供了一鍵啟動的 Docker 容器與環境設定，可直接在本機 (x86_64 + NVIDIA GPU) 重放 ROS 2 Bag，並評估多種演算法：

### 1. 啟動開發容器 (含 GPU 加速與 FastDDS 跨容器通訊)
```bash
./tools/run_isaac_ros_container.sh
```
> [!NOTE]
> 容器自動配置 `FASTRTPS_DEFAULT_PROFILES_FILE`（強制 UDPv4），徹底解決 Host（非 root）與 Container（root）因 FastDDS SHM 權限隔離導致 `ros2 topic echo` 收不到資料的問題。

### 2. 重放 ROS 2 Bag 與演算法啟動

#### A. Isaac ROS ESS 深度推論 (DNN Stereo Disparity & Depth)
在容器內啟動 ESS 節點：
```bash
ros2 launch my_robot_bringup isaac_ess.launch.py use_sim_time:=true
```
在另一個終端（容器內或 Host 端皆可）重放資料集：
```bash
ros2 bag play ros2_bag/rosbag2_total_20260903_203935 --clock
```
驗證深度與視差發布：
```bash
ros2 topic hz /stereo/disparity
ros2 topic hz /stereo/depth
```

#### B. Isaac ROS cuVSLAM (硬體加速雙目視覺里程計)
```bash
ros2 launch my_robot_bringup isaac_visual_slam.launch.py use_sim_time:=true enable_imu_fusion:=false
```
驗證里程計輸出：
```bash
ros2 topic echo /visual_slam/tracking/odometry
```

#### C. VINS-Fusion (雙目 + IMU 緊耦合 VIO)
```bash
ros2 launch my_robot_bringup vins_fusion.launch.py use_sim_time:=true
```
使用 RViz2 視覺化：
```bash
rviz2 -d src/my_robot_bringup/rviz/vins_fusion.rviz
```

#### D. VINS-Fusion + ESS + NVblox 3D 重建與 2D LiDAR 對比
啟動完整整合節點（VINS-Fusion VIO + 雙目立體校正 + ESS 深度推論 + 3D 點雲轉換 + NVblox TSDF/Mesh/Costmap）：
```bash
ros2 launch my_robot_bringup vins_ess_nvblox.launch.py use_sim_time:=true
```
啟動整合 RViz2 視覺化介面（已配置 LiDAR 紅色雷達點、ESS 彩色 3D 點雲、NVblox 3D Mesh 與 2D 障礙物代價地圖）：
```bash
rviz2 -d src/my_robot_bringup/rviz/vins_ess_nvblox.rviz
```
重放資料集（含 LiDAR `/scan`、相機 `/camera/...` 與 IMU `/imu/...`）：
```bash
ros2 bag play ros2_bag/rosbag2_total_20260903_203935 --clock
```

### 3. 一鍵自動化驗證所有管道
```bash
./tools/run_isaac_ros_container.sh "/workspaces/isaac_ros-dev/tools/test_all_pipelines.sh"
```

