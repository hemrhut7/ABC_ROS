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
