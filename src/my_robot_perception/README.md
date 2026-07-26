# 📸 My Robot Perception Package (my_robot_perception)

本套件負責機器人雙目相機影像的擷取、切割、壓縮傳輸與相機資訊發布，為機器人系統提供核心的視覺感知能力。

---

## 🚀 執行與部署教學

本套件之節點建議運行於 NVIDIA Jetson 之 Docker 容器環境中，以確保與 Host 端的 V4L2 相機硬體順暢串接。

### 執行相機分割節點

```bash
ros2 run my_robot_perception image_splitter_node
```

#### 正常啟動輸出：
```
Initializing Image Splitter Node...
Camera opened successfully. Actual Resolution: 1280x480 Actual FPS: 60
```
> [!NOTE]
> 節點預設會讀取 `/dev/video0` 裝置，並以解析度 `1280x480`（MJPG）、FPS `60` 進行雙目擷取與分割。若需要調整參數，請至 `my_robot_bringup` 的 [params.yaml](file:///home/hank/ROS_ABC/src/my_robot_bringup/config/params.yaml) 中設定。

---

### 驗證 Topic 數據輸出
請在 PC 端透過另一個 SSH 連線，或使用 TMUX 開啟新視窗，連入同一個運行中的容器：

```bash
# 1. 查詢運行中的容器 ID
docker ps

# 2. 進入該容器的 Bash 終端機
docker exec -it <您的容器ID或名稱> bash

# 3. 載入 ROS 2 與工作空間環境變數
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

#### 驗證指令與預期結果：
1. **確認 Topic 列表**：
   ```bash
   ros2 topic list
   ```
   您應會看到已成功發布以下主題：
   - `/camera/stereo/image_raw`（雙目原始拼接畫面）
   - `/camera/left/image_raw`（左眼獨立分割畫面）
   - `/camera/left/image_raw/compressed`（左眼壓縮畫面）
   - `/camera/left/camera_info`（左相機校正資訊）
   - `/camera/right/image_raw`（右眼獨立分割畫面）
   - `/camera/right/image_raw/compressed`（右眼壓縮畫面）
   - `/camera/right/camera_info`（右相機校正資訊）

2. **檢查發布更新率 (Hz)**：
   ```bash
   ros2 topic hz /camera/left/image_raw
   ```
   影像發布頻率應穩定維持在約 **60.0 Hz**。

---

## 💻 Foxglove Studio 視覺化監控配置

Foxglove Studio 適合用於即時觀看機器人雙目影像、點雲及感測器姿態。以下是連接與配置步驟：

### 1. Jetson 容器端：安裝並啟動 Foxglove 橋接節點
在容器內啟動 `foxglove_bridge`：
```bash
# 若未安裝橋接套件，請先執行安裝
sudo apt update && sudo apt install -y ros-humble-foxglove-bridge

# 啟動橋接節點
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

可指定topic進行發布，降低系統負載
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml topic_whitelist:="['/camera/right/image_raw']"
```

### 2. PC 端：連線與顯示影像
1. 前往官網下載並開啟 Windows 版 [Foxglove Studio](https://foxglove.dev/)。
2. 點選 **Open Connection**，連線類型選擇 **Foxglove WebSocket**。
3. 輸入連線網址：`ws://<Jetson_IP>:8765`（請將 `<Jetson_IP>` 替換為您的 Jetson 實際 IP），並點選 **Connect**。
4. 在 Foxglove 介面中新增一個 **Image Panel**。
5. 將主題選擇設為 `/camera/left/image_raw/compressed` 或 `/camera/right/image_raw/compressed`，即可即時瀏覽切割後的 1280x720 影像畫面。