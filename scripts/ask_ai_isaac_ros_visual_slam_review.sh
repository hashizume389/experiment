#!/usr/bin/env bash
set -euo pipefail

cat <<'PROMPT'
# 依頼: Isaac ROS Visual SLAMパッケージ側の設定差分を調査してください

あなたは Isaac ROS Visual SLAM / cuVSLAM 側を担当するAIです。
目的は、EuRoC MH_01 の Stereo mode, IMUなし評価で、原著論文の精度

- aveRTE: 0.29 %
- aveRE: 1.96 deg
- RMSE APE: 0.13 m

に対して、手元の検証結果

- aveRTE: 110 %
- aveRE: 20 deg
- RMSE APE: 0.20 m

が大きく異なる原因のうち、Isaac ROS Visual SLAMパッケージ側で確認・修正すべき要素を洗い出し、具体的な確認コマンド、launch/remap/parameter修正案、期待される正常値を示してください。

## 調査対象の責任範囲

この依頼では、以下だけを扱ってください。

- Isaac ROS Visual SLAM / cuVSLAM のlaunch設定
- remap設定
- parameter設定
- frame設定
- TF設定
- 入力topicの選び方
- 出力topicの選び方
- Stereo-only, IMU無効化の確認
- rectified画像を使う場合のVisual SLAM側設定
- Visual SLAM実行時のdrop frame、QoS、処理負荷、再生速度
- Isaac ROS / cuVSLAM のバージョン差

rosbag変換処理、CameraInfo生成処理、評価指標の実装、GTとのtimestamp対応、評価スクリプトのRTE/RE定義は別AIの担当です。必要なら「別担当に確認」とだけ書いてください。

## 手元リポジトリの前提

作業ディレクトリ:

```bash
/home/hashizume/experiment
```

このリポジトリ自体には Isaac ROS Visual SLAM 本体は含まれていません。
EuRoC bagをIsaac ROS Visual SLAMへ入力する準備用スクリプトと評価用ノードがあります。

関連ドキュメント:

```bash
/home/hashizume/experiment/docs/euroc_cuvslam_preparation.md
/home/hashizume/experiment/README.md
```

現在想定している入力topic:

```text
visual_slam/image_0        -> /cam0/image_rect
visual_slam/camera_info_0  -> /cam0/camera_info
visual_slam/image_1        -> /cam1/image_rect
visual_slam/camera_info_1  -> /cam1/camera_info
visual_slam/imu            -> /imu0
```

Stereo-only, IMUなし評価のため、`visual_slam/imu` を実際に使わせない設定になっているか確認してください。

LDFE前処理ありの場合の画像topic:

```text
visual_slam/image_0        -> /mh01/cam0/image_ldfe
visual_slam/camera_info_0  -> /cam0/camera_info
visual_slam/image_1        -> /mh01/cam1/image_ldfe
visual_slam/camera_info_1  -> /cam1/camera_info
```

Orchard preprocessありの場合の画像topic:

```text
visual_slam/image_0        -> /mh01/cam0/image_orchard_preprocessed
visual_slam/camera_info_0  -> /mh01/cam0/camera_info
visual_slam/image_1        -> /mh01/cam1/image_orchard_preprocessed
visual_slam/camera_info_1  -> /mh01/cam1/camera_info
```

## 特に確認してほしい論点

1. 原著論文の Stereo mode, IMUなし評価が、Isaac ROS Visual SLAM のどの設定に対応するか。
   - `enable_imu_fusion` のようなIMU関連パラメータ
   - stereo modeの有効化
   - localization/mapping/loop closureの有無
   - VO出力かSLAM pose出力か

2. 評価に使うべき出力topic。
   - `/visual_slam/tracking/odometry`
   - `/visual_slam/tracking/slam_path`
   - `/visual_slam/tracking/vo_path`
   - `/tf` または `/tf_static` 経由の `map -> base`
   - その他、現在のIsaac ROS Visual SLAMで推奨されるpose出力

3. `odom_frame`, `map_frame`, `base_frame`, camera optical frame の設定。
   - EuRoC GTがIMU/body frameである点を踏まえ、Visual SLAM側でどのframeをbaseにすべきか。
   - `base_link` を cam0 に置くべきか、IMU/bodyに置くべきか。
   - static TF をVisual SLAM側で読ませる必要があるか。

4. rectified画像入力時の設定。
   - `rectified_images` 相当のパラメータが必要か。
   - raw画像用設定のままrectified画像を入れていないか。
   - `CameraInfo` の `D/R/P` をVisual SLAMがどう解釈するか。

5. left/right topic順、baseline符号、frame_id の扱い。
   - Visual SLAM側の `image_0` が左、`image_1` が右でよいか。
   - `camera_info_0`, `camera_info_1` の対応が逆になっていないか。

6. EuRoC bag再生時の処理負荷。
   - `ros2 bag play --clock` とVisual SLAMのuse_sim_time設定。
   - 再生速度、frame drop、QoS不一致。
   - RVizや記録ノードを同時起動した場合の影響。

7. バージョン差。
   - 原著論文のcuVSLAMバージョンと手元のIsaac ROS Visual SLAMバージョンで、同じ精度表を再現できるか。
   - 既知のパラメータ名変更や出力topic変更があるか。

## 手元で実行してほしい確認コマンド例

Isaac ROS workspace側で、以下に相当する確認をしてください。
パスは環境に合わせて読み替えて構いません。

```bash
ros2 pkg prefix isaac_ros_visual_slam
ros2 pkg executables isaac_ros_visual_slam
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py --show-args
ros2 param list /visual_slam
ros2 param dump /visual_slam
ros2 topic list -t | grep -E 'visual_slam|tf|cam0|cam1|imu|odom|path'
ros2 topic info -v /visual_slam/tracking/odometry
ros2 topic hz /cam0/image_rect
ros2 topic hz /cam1/image_rect
ros2 topic hz /visual_slam/tracking/odometry
ros2 run tf2_tools view_frames
```

可能なら、起動launchファイルとparameter yamlも確認してください。

```bash
grep -R "rectified\\|imu\\|stereo\\|base_frame\\|odom_frame\\|map_frame\\|enable" -n \
  ${ISAAC_ROS_WS}/src/isaac_ros_visual_slam 2>/dev/null
```

## 期待する回答形式

以下の形式で回答してください。

```markdown
## 結論
Isaac ROS Visual SLAM側で最も疑わしい点を3つ以内で要約。

## 優先チェックリスト
- [ ] 項目、確認コマンド、正常時の期待値、異常時の修正案

## launch/remap修正案
必要なら具体的なlaunch引数、remap、parameter yamlを提示。

## 評価に使うべき出力
どのtopic/frameを評価すべきか、その理由。

## 別担当に渡すべき事項
rosbag/評価側でないと判断できない事項。
```
PROMPT
