# EuRoC MH_01 cuVSLAM Preparation

This repository does not contain Isaac ROS Visual SLAM itself. The scripts here prepare a EuRoC bag so that Isaac ROS Visual SLAM receives rectified stereo images, consistent CameraInfo, static TF, and header-aligned bag timestamps.

## Create a Rectified Bag

```bash
cd ~/experiment
python3 scripts/rectify_euroc_stereo_bag.py \
  MH_01_easy_ros2 \
  MH_01_easy_ros2_rectified \
  --overwrite
```

This creates:

- `/cam0/image_rect`
- `/cam1/image_rect`
- `/cam0/camera_info`
- `/cam1/camera_info`
- `/tf_static`

The rectified right CameraInfo has `P[3] = fx * Tx`, giving a baseline of about `0.110078 m` for MH_01.

## Restamp Bag Time from Header Time

```bash
python3 scripts/restamp_ros2_bag_by_header.py \
  MH_01_easy_ros2_rectified \
  MH_01_easy_ros2_rectified_restamped \
  --overwrite
```

Use `MH_01_easy_ros2_rectified_restamped` for playback. This avoids the 25-90 ms bag-time/header-time offset seen in the converted bag.

## Diagnose the Prepared Bag

```bash
python3 scripts/diagnose_euroc_bag.py MH_01_easy_ros2_rectified_restamped
```

Expected key checks:

- left/right image rate is 20 Hz
- bag-header offset is 0 ms
- left-right header offset is 0 us
- left frame is `cam0_rect`
- right frame is `cam1_rect`
- right CameraInfo baseline from `P` is about `0.110078 m`

## Isaac ROS Visual SLAM Remaps

Use the rectified, restamped topics as Visual SLAM inputs:

```text
visual_slam/image_0        -> /cam0/image_rect
visual_slam/camera_info_0  -> /cam0/camera_info
visual_slam/image_1        -> /cam1/image_rect
visual_slam/camera_info_1  -> /cam1/camera_info
visual_slam/imu            -> /imu0
```

If the Isaac ROS launch file has fixed Realsense topic names, update or override its remappings to the topics above. Also check that stereo mode and IMU usage are enabled in the Isaac ROS Visual SLAM configuration being launched.

Playback:

```bash
ros2 bag play ~/experiment/MH_01_easy_ros2_rectified_restamped --clock
```

Then record/evaluate `/visual_slam/tracking/odometry` with `mh01_eval_logger` and `mh01_metrics`.
