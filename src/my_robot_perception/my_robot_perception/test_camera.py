#!/usr/bin/env python3
"""
test_camera.py - 快速驗證相機讀取腳本

用法 (Usage)：
  python3 test_camera.py [device_index] [width] [height] [fps] [format] [options]

選項與參數說明 (Options & Arguments)：
  -h, --help           顯示此說明訊息並離開。
  device_index         相機裝置號碼，例如 0 表示 /dev/video0 (預設: 0)。
  width                目標影像寬度 (預設: 640)。
  height               目標影像高度 (預設: 480)。
  fps                  目標 FPS 幀率 (預設: 30)。
  format / --format    像素格式 (FOURCC): YUYV 或 MJPG (預設: YUYV)。
  --gray-y             僅讀取/擷取 Y 灰階亮度通道 (不進行 RGB/BGR 轉換，省去解碼 CPU 負載)。
  --headless           無視窗模式 (SSH 或無桌面 GUI 環境使用)。

使用範例 (Examples)：
  python3 test_camera.py --help
  python3 test_camera.py 0 640 480 30 YUYV
  python3 test_camera.py 0 1280 720 30 MJPG --headless
  python3 test_camera.py 0 640 480 30 YUYV --gray-y --headless
"""

import sys
import time
import cv2

def print_help():
    print(__doc__)
    sys.exit(0)

def main():
    # 檢查是否要求 Help 說明
    if '-h' in sys.argv or '--help' in sys.argv:
        print_help()

    # 解析命令列參數與選項
    gray_y = '--gray-y' in sys.argv
    headless = '--headless' in sys.argv

    format_arg = "YUYV"
    for arg in sys.argv[1:]:
        if arg.startswith('--format='):
            format_arg = arg.split('=')[1].upper()
        elif arg.upper() in ['MJPG', 'YUYV']:
            format_arg = arg.upper()

    # 過濾出非 option 參數作為位置參數
    positional_args = [
        arg for arg in sys.argv[1:]
        if not arg.startswith('--') and not arg.startswith('-') and arg.upper() not in ['MJPG', 'YUYV']
    ]

    device_idx = int(positional_args[0]) if len(positional_args) > 0 else 0
    width = int(positional_args[1]) if len(positional_args) > 1 else 1280
    height = int(positional_args[2]) if len(positional_args) > 2 else 480
    fps = int(positional_args[3]) if len(positional_args) > 3 else 30

    device_path = f"/dev/video{device_idx}" if isinstance(device_idx, int) else str(device_idx)

    print(f"==================================================")
    print(f" 🎥 相機讀取快速驗證工具 (Camera Test Tool)")
    print(f"==================================================")
    print(f" 裝置: {device_path}")
    print(f" 要求編碼: {format_arg}")
    print(f" 目標解析度: {width}x{height}")
    print(f" 目標 FPS: {fps}")
    print(f" 灰階模式 (--gray-y): {'開啟 (僅擷取 Y 亮度通道)' if gray_y else '關閉 (預設全彩)'}")
    print(f" 模式: {'無視窗 (Headless/SSH)' if headless else '視窗顯示 (GUI/OpenCV Window)'}")
    print(f"==================================================")

    # 1. 嘗試開啟相機
    cap = cv2.VideoCapture(device_idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[ERR] 無法開啟相機裝置: {device_path}")
        print("請確認：")
        print(" 1. 相機已正確連接並被系統識別 (`ls -l /dev/video*`)")
        print(" 2. 是否有權限讀取 `/dev/video*` (例如: `sudo chmod 666 /dev/video0` 或將使用者加入 video 群組)")
        print(" 3. 是否已被其他程式 (如 image_splitter_node) 佔用")
        sys.exit(1)

    # 若指定僅讀取 Y 灰階亮度，關閉 V4L2 自動轉 RGB/BGR 運算
    if gray_y:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

    # 2. 設定參數 (設定指定的像素格式 FOURCC)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*format_arg))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # 取得實際參數
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    actual_fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"[OK] 相機已順利開啟！")
    print(f"     實際編碼格式: {actual_fourcc_str}")
    print(f"     實際解析度: {actual_w}x{actual_h}")
    print(f"     實際 FPS 設定值: {actual_fps}")
    print(f"--------------------------------------------------")
    print(f" 開始測試讀取畫面 (按 Ctrl+C 或在視窗按 'q' 退出)...")

    frame_count = 0
    start_time = time.time()
    last_stat_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print(f"[WARN] 無法擷取影像畫面 (Frame capture failed)")
                time.sleep(0.05)
                continue

            # 若啟用 --gray-y，從 raw frame 中擷取 Y (Luminance) 亮度通道
            if gray_y:
                if len(frame.shape) == 3 and frame.shape[2] == 2:
                    # YUYV Raw Data Format (H, W, 2): 第 0 個通道即為 Y 灰階亮度
                    display_frame = frame[:, :, 0]
                elif len(frame.shape) == 3 and frame.shape[2] == 3:
                    # 相機已是 BGR/RGB Format
                    display_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                else:
                    display_frame = frame
            else:
                display_frame = frame

            frame_count += 1
            curr_time = time.time()

            # 每 2 秒統計一次實際運行的平均 FPS 與影像資訊
            if curr_time - last_stat_time >= 2.0:
                elapsed = curr_time - start_time
                calc_fps = frame_count / elapsed
                shape_info = "x".join(map(str, display_frame.shape))
                mode_str = "Y-Luminance (Gray)" if gray_y else "Full Color"
                print(f"[INFO] 已讀取 {frame_count} 幀 | 實測平均 FPS: {calc_fps:.2f} | 模式: {mode_str} | 尺寸: {shape_info}")
                last_stat_time = curr_time

            # 若非 headless 模式則嘗試開啟 OpenCV 視窗顯示
            if not headless:
                try:
                    cv2.imshow("Camera Verification Test (Press 'q' to quit)", display_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:  # 'q' 或 ESC
                        print("[INFO] 使用者按下 'q' / ESC 結束測試。")
                        break
                except cv2.error:
                    print(f"[WARN] 無法初始化 GUI 顯示介面 (GTK Backend 失敗，如無 DISPLAY 或未啟用 X11)。")
                    print(f"[WARN] 自動切換為無視窗 (Headless) 模式繼續進行測試...")
                    headless = True

    except KeyboardInterrupt:
        print("\n[INFO] 使用者按下 Ctrl+C 中斷測試。")
    finally:
        cap.release()
        if not headless:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        
        total_time = time.time() - start_time
        final_fps = (frame_count / total_time) if total_time > 0 else 0
        print(f"==================================================")
        print(f" 測試完成！總共擷取 {frame_count} 幀，耗時 {total_time:.2f} 秒")
        print(f" 最終平均 FPS: {final_fps:.2f}")
        print(f"==================================================")

if __name__ == '__main__':
    main()
