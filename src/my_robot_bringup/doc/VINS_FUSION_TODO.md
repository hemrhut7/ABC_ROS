# 🧭 VINS-Fusion 整合 TODO & 注意事項

> 本文件記錄 VINS-Fusion Stereo+IMU 整合至 ABC_ROS 工作空間時的代辦事項、注意事項與建議。

---

## 📋 TODO 待辦清單

### 🔴 必要 (Must Do)

- [ X ] **IMU 噪聲參數校正**
  - 目前 [`vins_fusion_stereo_imu_config.yaml`](../config/vins_fusion_stereo_imu_config.yaml) 中的 `acc_n`, `gyr_n`, `acc_w`, `gyr_w` 為**通用預設值**
  - 建議使用 [imu_utils](https://github.com/gaowenliang/imu_utils) 或 Allan Variance 分析工具，錄製靜止狀態下的 IMU 數據進行校正
  - 校正完成後，替換 config 中對應的四個參數

- [ X ] **Camera-IMU 外參精確校正**
  - 目前 `body_T_cam0` / `body_T_cam1` 是從 URDF 機械尺寸推導的**初始估計值**
  - `estimate_extrinsic: 1` 已啟用線上優化，但精確的初始值能大幅改善收斂速度
  - 建議使用 [Kalibr](https://github.com/ethz-asl/kalibr) 進行 Camera-IMU 聯合校正
  - 校正完成後：
    1. 更新 `body_T_cam0` / `body_T_cam1` 的值

- [ ] **確認 VINS-Fusion ROS 2 套件的 executable 名稱**
  - Launch 檔案中假設為 `vins_fusion_node`
  - 請用以下指令確認：
    ```bash
    ros2 pkg executables vins_fusion_ros2
    ```
  - 若 executable 名稱不同，請修改 [`vins_fusion.launch.py`](../launch/vins_fusion.launch.py) 中的 `executable` 欄位

### 🟡 建議 (Should Do)

- [ ] **測試 Loop Closure 功能**
  - 目前 `loop_closure: 0`（關閉），以降低 Jetson 運算負載
  - 在確認系統穩定運行後，可嘗試開啟 (`loop_closure: 1`)
  - 需要安裝 DBoW2 的詞彙表檔案

### 🟢 可選 (Nice to Have)

- [ ] **RViz 配置優化**
  - [`vins_fusion.rviz`](../rviz/vins_fusion.rviz) 提供基本可視化
  - 可根據實際使用需求新增 MarkerArray、Image 等顯示項目

- [ ] **整合至 robot.launch.py**
  - 若需要一鍵啟動所有節點（含 VINS-Fusion），可在 `robot.launch.py` 中使用 `IncludeLaunchDescription` 引入 `vins_fusion.launch.py`

---

## ⚠️ 注意事項

### 1. 影像翻轉變更

本次整合修改了影像處理流程：

| 項目 | 修改前 | 修改後 |
|------|--------|--------|
| `image_splitter_node.cpp` 預設 `rotation_angle` | `0` | `180` |
| `params.yaml` 中 `rotation_angle` | `0` | `180` |
| `robot.urdf` 中 camera_joint `rpy` | `3.1415926 0 0` | `0 0 0` |

**影響範圍：**
- 所有訂閱 `/camera/left/image_raw` 與 `/camera/right/image_raw` 的節點，現在收到的影像是**正向**的
- URDF 中的 TF 座標系不再包含 180° 旋轉
- 若有其他節點依賴舊的影像方向或 TF 座標系，需要確認是否受影響

### 2. Config 檔案格式

VINS-Fusion 使用 **`%YAML:1.0`** 格式（OpenCV YAML），與 ROS 2 標準的 YAML 格式不同。注意事項：
- 矩陣使用 `!!opencv-matrix` 標記
- 不要嘗試用 ROS 2 的 `parameters: [config_file]` 方式載入此檔案
- VINS-Fusion 節點使用 `config_file` 參數指定 config 路徑，自行解析

### 3. 基線單位轉換

`stereo_params.yaml` 中的 T 向量單位為**毫米 (mm)**：
```
T = [-52.936, -0.265, -0.868] mm
```
已轉換為**公尺 (m)** 填入 `body_T_cam1` 中。若重新校正，注意單位換算。

---

## 🚀 快速啟動指南

```bash
# 1. 編譯工作空間
cd /workspace
colcon build --symlink-install --packages-select my_robot_bringup my_robot_perception

# 2. 載入環境
source install/setup.bash

# 3. 啟動相機與 IMU 節點
ros2 launch my_robot_bringup robot.launch.py

# 4. 在另一個終端啟動 VINS-Fusion
ros2 launch my_robot_bringup vins_fusion.launch.py

# 5. (可選) 使用 rosbag 回放
ros2 launch my_robot_bringup vins_fusion.launch.py use_sim_time:=true

# 6. 確認輸出
ros2 topic list | grep vins
ros2 topic echo /vins_fusion/odometry --once
```

---

## 📁 本次新增/修改的檔案清單

### 新增檔案
| 檔案 | 說明 |
|------|------|
| `config/vins_fusion_stereo_imu_config.yaml` | VINS-Fusion 主設定檔 |
| `config/cam0_pinhole.yaml` | 左相機內參 (Pinhole 模型) |
| `config/cam1_pinhole.yaml` | 右相機內參 (Pinhole 模型) |
| `launch/vins_fusion.launch.py` | VINS-Fusion launch 檔案 |
| `rviz/vins_fusion.rviz` | RViz2 可視化設定 |
| `doc/VINS_FUSION_TODO.md` | 本文件 (TODO & 注意事項) |

### 修改檔案
| 檔案 | 變更內容 |
|------|----------|
| `config/params.yaml` | `rotation_angle: 0` → `180` |
| `config/robot.urdf` | camera_joint `rpy="3.14... 0 0"` → `"0 0 0"` |
| `image_splitter_node.cpp` | 預設 `rotation_angle` `0` → `180` |
