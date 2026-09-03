# 📸 My Robot Perception Package (`my_robot_perception`)

本套件專為 **Jetson Orin Nano** 開發，負責機器人雙目 USB 相機影像硬體加速擷取、180° 幾何翻轉、多流分流（AI 彩色流 / VSLAM 灰階去噪流）、JPEG 壓縮傳輸、相機資訊發布以及 N10 LiDAR 光學雷達資料處理。

> [!NOTE]
> 🚀 **Jetson 硬體加速與多流分流架構 (VIC + Composable Node)**：
> - **解碼與翻轉**：採用 SIMD 多執行緒 JPEG 解碼結合 **Jetson 硬體 VIC 引擎 (`nvvidconv`)** 進行零延遲 180° 翻轉與色彩空間轉換，實測吞吐量達 **72+ FPS**。
> - **多流分流**：
>   - **彩色分支 (`bgr8` @ 15 Hz)**：降頻節流供 AI 物件辨識與語意分割使用。
>   - **灰階去噪分支 (`mono8` @ 60 Hz)**：全速發布並導入 $3 \times 3$ 高斯平滑濾波（$\sigma = 0.5$），平滑 MJPEG 壓縮偽影，專供 cuVSLAM / VIO 光流特徵追蹤。
> - **ROS 2 座標系標準 (REP-103)**：影像 `header.frame_id` 綁定標準光學座標系（`left_camera_optical_frame` / `right_camera_optical_frame`），完美支援 RViz2 與 3D 投影渲染。

---

## 🏗️ 核心節點與架構

本套件提供兩個核心可執行檔與 Composable Node 元件：

| 節點 / 元件名稱 | 類型 | 說明 |
| :--- | :--- | :--- |
| `image_splitter_node` / `ImageSplitterNode` | C++ (Composable Node) | 雙目相機硬體擷取、ROI 切割、多速率分流與相機資訊發布 |
| `n10_lidar_node` | C++ Standalone | N10 360° 光學雷達 Serial 驅動與 `/scan` 發布 |

---

## 🚀 執行與部署教學

### 1. 啟動相機分割節點 (獨立執行)

```bash
ros2 run my_robot_perception image_splitter_node --ros-args --params-file /workspace/src/my_robot_bringup/config/params.yaml
```

#### 正常啟動輸出：
```text
[INFO] [image_splitter_node]: ====================================================
[INFO] [image_splitter_node]: Initializing Jetson Stereo Camera Pipeline (rclcpp Composable Node):
[INFO] [image_splitter_node]:   HW Acceleration: Enabled (Jetson VIC) (Decoder: jpegdec)
[INFO] [image_splitter_node]:   Preferred Device: /dev/video0 (Auto Scan: Enabled)
[INFO] [image_splitter_node]:   Target Resolution: 1280x480 @ 60 FPS (FOURCC: MJPG)
[INFO] [image_splitter_node]:   Rotation Angle: 180 deg (Hardware VIC flip-method=2)
[INFO] [image_splitter_node]:   Frames IDs: Left='left_camera_optical_frame', Right='right_camera_optical_frame'
[INFO] [image_splitter_node]:   Streams Config:
[INFO] [image_splitter_node]:     - BGR8 Stream: Enabled (Rate: 15 Hz)
[INFO] [image_splitter_node]:     - Mono8 Stream: Enabled (Rate: Full Speed (60 Hz), Gaussian Filter: ON [k=3, s=0.50])
[INFO] [image_splitter_node]:     - Compressed Stream: Enabled (Rate: 15 Hz, Quality: 60)
[INFO] [image_splitter_node]: ====================================================
[INFO] [image_splitter_node]: Successfully opened hardware decode pipeline on /dev/video0 (CCB Camera: CCB Camera)!
```

---

### 2. 啟動 N10 LiDAR 節點

```bash
ros2 run my_robot_perception n10_lidar_node --ros-args --params-file /workspace/src/my_robot_bringup/config/params.yaml
```

---

### 3. 一鍵啟動全機器人感知系統 (`robot.launch.py`)

在 `my_robot_bringup` 中已整合全機啟動檔：
```bash
ros2 launch my_robot_bringup robot.launch.py auto_log:=false
```

---

## 📡 ROS 2 介面與主題說明

### 發布之主題列表 (Published Topics)

| 主題名稱 (Topic) | 訊息型態 (Message Type) | 頻率 (Rate) | 用途說明 |
| :--- | :--- | :---: | :--- |
| `/camera/left/image_raw` | `sensor_msgs/msg/Image` (bgr8) | 15 Hz | 左眼彩色畫面（AI 物件辨識 / 語意分割） |
| `/camera/right/image_raw` | `sensor_msgs/msg/Image` (bgr8) | 15 Hz | 右眼彩色畫面（AI 物件辨識 / 語意分割） |
| `/camera/left/image_mono` | `sensor_msgs/msg/Image` (mono8) | 60 Hz | 左眼灰階去噪畫面（cuVSLAM / VIO 特徵追蹤） |
| `/camera/right/image_mono` | `sensor_msgs/msg/Image` (mono8) | 60 Hz | 右眼灰階去噪畫面（cuVSLAM / VIO 特徵追蹤） |
| `/camera/left/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 15 Hz | 左眼 JPEG 壓縮影像（RViz / Web 遠端監控） |
| `/camera/right/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 15 Hz | 右眼 JPEG 壓縮影像（RViz / Web 遠端監控） |
| `/camera/left/camera_info` | `sensor_msgs/msg/CameraInfo` | 60 Hz | 左相機內參與立體幾何投影資訊 |
| `/camera/right/camera_info` | `sensor_msgs/msg/CameraInfo` | 60 Hz | 右相機內參與立體幾何投影資訊 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | N10 360° 平面雷達點雲掃描 |

---

## ⚙️ 參數設定說明 (`params.yaml`)

可於 `src/my_robot_bringup/config/params.yaml` 中調整以下感知參數：

```yaml
image_splitter_node:
  ros__parameters:
    use_hardware_decode: true
    hw_decoder_type: "jpegdec"        # Options: "jpegdec" (推薦，結合 VIC 達 70+ FPS), "nvv4l2decoder", "nvjpegdec"
    video_device: 0
    auto_detect_device: true
    width: 1280
    height: 480
    fps: 60
    fourcc: "MJPG"
    frame_id: "camera_link"
    left_frame_id: "left_camera_optical_frame"
    right_frame_id: "right_camera_optical_frame"
    rotation_angle: 180                # 0, 90, 180, 270 (由硬體 VIC nvvidconv 自動旋轉)
    publish_bgr: true
    publish_mono: true
    publish_compressed: true
    bgr_publish_fps: 15                # 彩色流節流頻率 (Hz)
    mono_publish_fps: 0                # 灰階流頻率 (0 = 全速 60 Hz)
    compressed_publish_fps: 15         # 壓縮流頻率 (Hz)
    enable_mono_filter: true           # 啟用 3x3 高斯去噪以平滑 MJPEG 壓縮塊
    mono_filter_ksize: 3
    mono_filter_sigma: 0.5
    jpeg_quality: 60                   # Compressed 串流品質 (1-100)
```

---

## 🧭 座標系定義 (TF Tree - REP-103)

相機座標系符合 ROS REP-103 規範：
- **物理座標系 (`left_camera_link` / `right_camera_link`)**：用於機器人 URDF 外殼與本體干涉計算（$X$ 朝前、$Y$ 朝左、$Z$ 朝上）。
- **光學座標系 (`left_camera_optical_frame` / `right_camera_optical_frame`)**：用於相機影像投影、OpenCV 與 cuVSLAM（$Z$ 朝前光軸、$X$ 朝右、$Y$ 朝下）。

```text
base_link -> camera_link -> left_camera_link  -> left_camera_optical_frame  (/camera/left/image_*)
                         -> right_camera_link -> right_camera_optical_frame (/camera/right/image_*)
```

---

## 💻 視覺化監控 (RViz2 & Foxglove Studio)

### 1. RViz2 監控
在 PC 端啟動 RViz2，新增 **Image** 或 **Camera** 顯示插件：
- **Topic**: `/camera/left/image_raw` 或 `/camera/left/image_raw/compressed`
- **Fixed Frame**: `base_link` 或 `left_camera_optical_frame`

### 2. Foxglove Studio 橋接監控
啟動 Foxglove WebSocket Bridge 供網頁與跨平台客戶端即時監控：
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```