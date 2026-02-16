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
sudo apt-get update && rosdep update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_visual_slam/isaac_ros_visual_slam --ignore-src -y
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