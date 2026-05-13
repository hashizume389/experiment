# orchard_vslam_preprocess

`orchard_vslam_preprocess` は、柿農園のように葉の揺れ、樹冠、空、局所的な照明変化が多い環境で Isaac ROS Visual SLAM に入力するステレオ画像を軽量に前処理する ROS 2 Humble 向け C++17 パッケージです。

このノードは Visual SLAM 内部の特徴点抽出器を直接変更しません。CLAHE、ガンマ補正、Sobel/Laplacian、局所分散、フレーム間差分、画像上部ペナルティから信頼度マップを作り、入力画像を重み付けすることで、低信頼領域から特徴点が出にくくなるように間接的に誘導します。

## 入出力

入力:

- `/camera/camera/infra1/image_rect_raw`
- `/camera/camera/infra2/image_rect_raw`
- `/camera/camera/infra1/camera_info`
- `/camera/camera/infra2/camera_info`

出力:

- `/orchard/left/image_preprocessed`
- `/orchard/right/image_preprocessed`
- `/orchard/left/camera_info`
- `/orchard/right/camera_info`
- `/orchard/left/reliability_map`
- `/orchard/right/reliability_map`
- `/orchard/left/debug_keypoints`
- `/orchard/right/debug_keypoints`

画像と CameraInfo は `sensor_data` QoS を使用します。左右画像は `message_filters` の ApproximateTime で同期して処理します。CameraInfo は画像サイズを変えないため、そのまま転送します。

## ビルド

ワークスペース直下で実行します。

```bash
colcon build --packages-select orchard_vslam_preprocess
source install/setup.bash
```

## 起動

```bash
ros2 launch orchard_vslam_preprocess orchard_vslam_preprocess.launch.py
```

設定は `config/orchard_vslam_preprocess.yaml` で変更できます。

## パラメータ

- `use_gamma`: ガンマ補正を有効化します。
- `gamma`: ガンマ値です。初期値は `0.8` です。
- `use_clahe`: CLAHE を有効化します。
- `clahe_clip_limit`: CLAHE の clip limit です。
- `clahe_tile_grid_size`: CLAHE の tile grid size です。
- `w_sobel`: Sobel 勾配強度の重みです。
- `w_laplacian`: Laplacian 応答の重みです。
- `w_variance`: 局所分散の重みです。
- `w_temporal`: フレーム間差分ペナルティの重みです。
- `w_top`: 画像上部ペナルティの重みです。
- `top_penalty_ratio`: 上部何割にペナルティをかけるかを指定します。
- `min_intensity_weight`: 低信頼領域を暗くしすぎないための下限重みです。
- `mask_power`: 信頼度マスクの強さを調整します。
- `reliability_blur_kernel_size`: 信頼度マップにかける GaussianBlur のカーネルサイズです。`1` 以下で無効です。
- `local_variance_kernel_size`: 局所分散計算の blur カーネルサイズです。
- `grid_rows`: デバッグ特徴点分布評価のグリッド行数です。
- `grid_cols`: デバッグ特徴点分布評価のグリッド列数です。
- `max_keypoints_per_cell`: 各グリッドセルで採用する最大特徴点数です。
- `orb_nfeatures`: デバッグ用 ORB の最大特徴点数です。
- `log_interval`: 何フレームごとに評価指標を ROS ログへ出すかを指定します。

## Isaac ROS Visual SLAM への接続例

Isaac ROS Visual SLAM 側の stereo image input を以下にリマップします。

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py \
  left_image_topic:=/orchard/left/image_preprocessed \
  right_image_topic:=/orchard/right/image_preprocessed \
  left_camera_info_topic:=/orchard/left/camera_info \
  right_camera_info_topic:=/orchard/right/camera_info
```

実際の launch ファイルやパラメータ名は使用中の Isaac ROS Visual SLAM launch 構成に合わせて調整してください。

## デバッグ画像の見方

`/orchard/left/reliability_map` と `/orchard/right/reliability_map` は 0-255 の `mono8` 画像です。明るいほど幾何的に安定していると推定された領域です。

`/orchard/left/debug_keypoints` と `/orchard/right/debug_keypoints` は評価用の `bgr8` 画像です。ORB で検出した特徴点のうち、信頼度スコアとグリッド分布制約で採用された点を緑、除外された点を赤で表示します。このデバッグ特徴点は Isaac ROS Visual SLAM へ渡していません。

ROS ログには一定間隔で以下を出力します。

- `total_keypoints`
- `selected_keypoints`
- `selected_keypoints / total_keypoints`
- `occupied_grid_cells / total_grid_cells`
- `grid_entropy`
- `mean_reliability_score`
- `mean_temporal_difference`
- `processing_time_ms`

## 注意点

このノードはセグメンテーション AI、YOLO、重いニューラルネットワークを使いません。OpenCV と ROS 2 標準的なパッケージだけで動く軽量な前処理ノードです。

信頼度マップは特徴点抽出を間接的に誘導するためのものです。Visual SLAM 内部の特徴点抽出器、追跡器、外れ値除去を置き換えるものではありません。強くマスクしすぎると有効な特徴点も減るため、`min_intensity_weight`、`mask_power`、各重みは実機ログを見ながら調整してください。
