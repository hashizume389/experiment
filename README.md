# 2/2実験用コマンドまとめ

# Jetson

## Docker

### コンテナ起動

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```

### 依存関係のインストール
```bash
sudo apt-get update
```

```bash
rosdep update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_visual_slam/isaac_ros_visual_slam --ignore-src -y
```
### 環境の有効化

```bash
source install/setup.bash
```

### カメラが認識されているかの確認

```bash
rs-enumerate-devices --compact
```

### Visual SLAMの起動（通常モード）

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

### Visual SLAM — 録画モード（カメラのみ、SLAMなし）

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py mode:=record
```

### Visual SLAM — 再生モード（SLAMのみ、カメラなし）

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py mode:=playback
```

### rosbagの記録（カメラトピック）

録画モード中に別ターミナルで実行する。

```bash
ros2 bag record  \
  /camera/camera/infra1/image_rect_raw \
  /camera/camera/infra1/camera_info \
  /camera/camera/infra2/image_rect_raw \
  /camera/camera/infra2/camera_info \
  /tf_static
```

### rosbagの再生（Visual SLAM用）

再生モード中に別ターミナルで実行する。

```bash
ros2 bag play ~/experiment/bag
```

## ローカル

### CAN通信の有効化

```bash
cd ~/scout_ws && source install/setup.bash && sudo ip link set can0 up type can bitrate 500000 
```

### SCOUT MINIノードの起動

```bash
cd ~/scout_ws && ros2 launch scout_base scout_mini_base.launch.py use_joystick:=false
```

# PC

## GitHub

### Gitの最新情報を取得

```bash
git fetch origin
```

### 最新情報をPull

```bash
git pull origin main
```

## ノードの起動

### 環境の有効化

```bash
cd ~/experiment && source install/setup.bash
```

## EuRoC MH_01をIsaac ROS Visual SLAM向けに整える

cuVSLAMへEuRoC raw画像を直接入れると、rectification、CameraInfoのbaseline、bag時刻とheader時刻のズレで精度が大きく崩れる可能性があります。以下でrectified + restamped bagを作成します。

```bash
cd ~/experiment
python3 scripts/rectify_euroc_stereo_bag.py MH_01_easy_ros2 MH_01_easy_ros2_rectified --overwrite
python3 scripts/restamp_ros2_bag_by_header.py MH_01_easy_ros2_rectified MH_01_easy_ros2_rectified_restamped --overwrite
python3 scripts/diagnose_euroc_bag.py MH_01_easy_ros2_rectified_restamped
```

Isaac ROS Visual SLAM側では、入力を以下へremapしてください。

```text
visual_slam/image_0        -> /cam0/image_rect
visual_slam/camera_info_0  -> /cam0/camera_info
visual_slam/image_1        -> /cam1/image_rect
visual_slam/camera_info_1  -> /cam1/camera_info
visual_slam/imu            -> /imu0
```

詳細は`docs/euroc_cuvslam_preparation.md`を参照してください。

### ノードの起動
```bash
ros2 run square_vs square_vs
```

### rosbagの記録
```bash
ros2 bag record -a -o ~/experiment/bag
```
### rosbagの再生
```bash
ros2 bag play ~/experiment/bag/rosbag2_2026_02_16-13_21_04
```

## infra_enhancer パッケージ

### 個別ノードの起動

```bash
# オールインワンノード（後方互換）
ros2 run infra_enhancer infra_enhancer_node

# デノイズノード
ros2 run infra_enhancer denoise_node

# CLAHEノード
ros2 run infra_enhancer clahe_node

# 正規化ノード
ros2 run infra_enhancer normalize_node

# LDFE-SLAMノード
ros2 run infra_enhancer ldfe_node
```

### Launchファイル（パイプライン起動）

```bash
# Denoise → CLAHE → Normalize 3段パイプライン（左右画像）
ros2 launch infra_enhancer infra_pipeline_launch.py

# パラメータ指定例
ros2 launch infra_enhancer infra_pipeline_launch.py denoise_method:=bilateral
ros2 launch infra_enhancer infra_pipeline_launch.py clahe_clip:=3.0 clahe_tile:=16

# LDFE-SLAM 照明適応型前処理パイプライン（左右画像）
ros2 launch infra_enhancer ldfe_pipeline_launch.py
```

### 柿農園実験用パイプライン

```bash
# パイプライン1: バイラテラルフィルタ → LDFE
ros2 launch infra_enhancer bilateral_ldfe_launch.py

# パイプライン2: バイラテラルフィルタ → LDFE → アンシャープマスク
ros2 launch infra_enhancer bilateral_ldfe_unsharp_launch.py

# パイプライン3: バイラテラルフィルタ → LDFE → Sobel
ros2 launch infra_enhancer bilateral_ldfe_sobel_launch.py
```
