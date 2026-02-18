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
sudo rosdep update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_visual_slam/isaac_ros_visual_slam --ignore-src -y
```
### 環境の有効化

```bash
source install/setup.bash
```

### カメラが認識されているかの確認

```bash
rs-enumerate-devices --compact
```

### Visual SLAMの起動

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
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