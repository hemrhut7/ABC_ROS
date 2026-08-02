# 📸 My Robot Perception Package (my_robot_perception)

本套件負責機器人雙目相機影像擷取、切割、JPEG 壓縮傳輸、相機資訊發布以及 N10 LiDAR 光學雷達資料處理。

> [!NOTE]
> 🚀 **性能優化升級 (C++ / rclcpp)**：本套件已全面採用 ROS 2 C++ API (`rclcpp`) 重構，取代原先 Python 實現，大幅降低 CPU 資源佔用，提升雙目 60 FPS 影像處理與光學雷達封包解析效能。

---

## 🚀 執行與部署教學

本套件之節點包含兩個核心 C++ 可執行檔：
1. `image_splitter_node`：雙目相機影像擷取與切割節點
2. `n10_lidar_node`：N10 光學雷達 Serial 驅動與 `/scan` 發布節點

### 1. 執行相機分割節點

```bash
ros2 run my_robot_perception image_splitter_node
```

#### 正常啟動輸出：
```text
[INFO] [image_splitter_node]: Initializing Image Splitter Node (rclcpp C++):
  Device: /dev/video0
  Target Resolution: 1280x480
  Target FPS: 60
  Target Format (FOURCC): MJPG
  Frame ID: camera_link
  Rotation Angle: 180
  JPEG Quality: 60
[INFO] [image_splitter_node]: Camera opened successfully.
  Actual Format: MJPG
  Actual Resolution: 1280x480
  Actual FPS: 60.0
```

> [!NOTE]
> 節點預設讀取 `/dev/video0`，並以解析度 `1280x480` (MJPG)、FPS `60` 進行雙目擷取與分割。參數可於 `my_robot_bringup` 的 [params.yaml](file:///home/hank/Github/ABC_ROS/src/my_robot_bringup/config/params.yaml) 中動態配置。

---

### 2. 執行 N10 LiDAR 節點

```bash
ros2 run my_robot_perception n10_lidar_node
```

#### 正常啟動輸出：
```text
[INFO] [n10_lidar_node]: Initializing N10 LiDAR Node (rclcpp C++ Standalone):
  Target Port: Auto-detecting USB port
  Baud Rate: 230400
  Frame ID: laser_frame
  Output Topic: /scan
  Range Min/Max: 0.05m / 12.00m
[INFO] [n10_lidar_node]: Connected successfully to LiDAR at port: /dev/ttyUSB0
```

---

### 3. 驗證 Topic 數據輸出

```bash
# 檢查 Topic 列表
ros2 topic list

# 檢查影像與雷達更新頻率
ros2 topic hz /camera/left/image_raw
ros2 topic hz /scan
```

已發布的主題包含：
- `/camera/stereo/image_raw`（雙目原始畫面）
- `/camera/left/image_raw`（左眼獨立分割畫面）
- `/camera/left/image_raw/compressed`（左眼 JPEG 壓縮畫面）
- `/camera/left/camera_info`（左相機校正資訊）
- `/camera/right/image_raw`（右眼獨立分割畫面）
- `/camera/right/image_raw/compressed`（右眼 JPEG 壓縮畫面）
- `/camera/right/camera_info`（右相機校正資訊）
- `/scan`（N10 360 度 LaserScan 點雲數據）

---

## 💻 Foxglove Studio 視覺化監控配置

Foxglove Studio 適合用於即時觀看機器人雙目影像、雷達點雲 `/scan` 及感測器姿態。

### 啟動 Foxglove 橋接節點：
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```