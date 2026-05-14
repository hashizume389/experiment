# orchard_vslam_preprocess

`orchard_vslam_preprocess` は、Isaac ROS Visual SLAM に入力するステレオ赤外画像を軽量に前処理する ROS 2 / C++17 パッケージです。

このノードは Isaac ROS Visual SLAM 内部の特徴点を直接変更しません。Visual SLAM は画像入力のみを受け取る前提なので、本ノードでは ORB または FAST で外部特徴点候補を検出し、その信頼度に基づいて入力画像を局所的に変調します。外部特徴点候補は Visual SLAM へ渡す特徴点ではなく、元画像をどこで強調または軽く抑制するかを決めるガイドです。

Visual SLAM 入力用の `/orchard/*/image_preprocessed` は、特徴点だけを描いた画像ではありません。`CLAHE(gamma(I))` をベースに、信頼できる特徴点候補の周辺へ滑らかなガウシアン状の重みを加えた `mono8` 画像です。特徴点だけの白点画像は、元画像の自然な輝度勾配、局所テクスチャ、パッチ情報を破壊し、Visual SLAM 内部の特徴抽出と追跡に必要な情報を失わせるため、SLAM 入力には使いません。

`debug_keypoints` は可視化専用です。採用された特徴点を緑、除外された特徴点を赤で描画します。この debug 画像は Visual SLAM 入力には使わないでください。

## 処理フロー

入力画像を `mono8` 化し、必要に応じてガンマ補正と CLAHE を適用します。ORB または FAST で特徴点候補を検出し、Sobel 勾配、Laplacian 応答、局所分散、フレーム間差分、画像上部ペナルティ、特徴点密集ペナルティから各候補の信頼度を 0-1 に正規化します。

画像を `grid_rows x grid_cols` に分割し、各セルから `max_keypoints_per_cell` 個まで高信頼候補を採用します。採用点の周辺にガウシアン状の boost を加え、画素信頼度に基づく低信頼領域の軽い抑制と合わせて変調マップを作ります。

```text
I_base = CLAHE(gamma(I))
M = clip(min_modulation_weight + (1 - min_modulation_weight) * reliability + keypoint_boost,
         min_modulation_weight, max_modulation_weight)
I_out = clip(I_base * M, 0, 255)
```

## トピック

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

入力画像と CameraInfo は sensor data QoS で購読します。出力画像、出力 CameraInfo、debug 画像は RViz2 のデフォルト設定でも確認しやすいよう `RELIABLE / VOLATILE` で publish します。Isaac ROS Visual SLAM 側が sensor data QoS の `BEST_EFFORT` subscriber でも、`RELIABLE` publisher とはQoS互換です。デフォルトでは左右画像を独立に処理するため、片側の画像が欠落している場合や左右timestampが揃わない場合でも、届いた側の出力トピックをpublishします。`use_stereo_sync: true` にすると `message_filters` の ApproximateTime で左右同期処理します。CameraInfo は画像サイズを変えないため、そのまま転送します。`reliability_map` は 0-255 の `mono8` で publish されます。

## 接続例

Isaac ROS Visual SLAM 側の stereo image input を前処理画像へリマップします。

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py \
  left_image_topic:=/orchard/left/image_preprocessed \
  right_image_topic:=/orchard/right/image_preprocessed \
  left_camera_info_topic:=/orchard/left/camera_info \
  right_camera_info_topic:=/orchard/right/camera_info
```

debug 確認例:

```bash
ros2 run rqt_image_view rqt_image_view /orchard/left/debug_keypoints
ros2 run rqt_image_view rqt_image_view /orchard/left/reliability_map
```

## パラメータ

- `use_gamma`: ガンマ補正を有効化します。
- `gamma`: ガンマ値です。
- `use_clahe`: CLAHE を有効化します。
- `clahe_clip_limit`: CLAHE の clip limit です。
- `clahe_tile_grid_size`: CLAHE の tile grid size です。
- `feature_detector_type`: `ORB` または `FAST` を選択します。
- `orb_nfeatures`: ORB の最大特徴点数です。
- `fast_threshold`: FAST の閾値です。
- `w_sobel`: Sobel 勾配強度の重みです。
- `w_laplacian`: Laplacian 応答の重みです。
- `w_variance`: 局所分散の重みです。
- `w_temporal`: フレーム間差分ペナルティの重みです。
- `w_top`: 画像上部ペナルティの重みです。
- `w_density`: 特徴点密集ペナルティの重みです。
- `grid_rows`, `grid_cols`: 特徴点選択グリッドの行数と列数です。
- `max_keypoints_per_cell`: 各セルで採用する最大特徴点数です。
- `top_penalty_ratio`: 画像上部の何割にペナルティをかけるかを指定します。
- `local_variance_kernel_size`: 局所分散計算のカーネルサイズです。
- `reliability_blur_kernel_size`: 信頼度マップ平滑化のカーネルサイズです。
- `gaussian_sigma`: 採用特徴点周辺に加えるガウシアン boost の広がりです。
- `keypoint_boost_strength`: 採用特徴点周辺の強調量です。
- `min_modulation_weight`: 最低輝度保持率です。低信頼領域も完全な黒にはしません。
- `max_modulation_weight`: 最大強調率です。
- `publish_debug`: debug_keypoints の publish を有効化します。
- `use_stereo_sync`: `true` で左右ApproximateTime同期処理、`false` で左右独立処理です。デフォルトは `false` です。
- `log_interval`: 何フレームごとに評価ログを出すかを指定します。

トピック名も YAML から変更できます。

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

別の設定ファイルを使う場合:

```bash
ros2 launch orchard_vslam_preprocess orchard_vslam_preprocess.launch.py \
  params_file:=/path/to/orchard_vslam_preprocess.yaml
```

## 評価ログ

`log_interval` ごとに、左右それぞれの `total_keypoints`、`selected_keypoints`、選択率、グリッド占有率、`normalized_grid_entropy`、`mean_reliability_score`、`mean_temporal_difference` と、ステレオ処理全体の `processing_time_ms` を出力します。
