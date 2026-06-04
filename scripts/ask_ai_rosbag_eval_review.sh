#!/usr/bin/env bash
set -euo pipefail

cat <<'PROMPT'
# 依頼: EuRoC rosbag・評価スクリプト側の差分を調査してください

あなたは rosbag変換・timestamp・CameraInfo・評価スクリプト側を担当するAIです。
目的は、EuRoC MH_01 の Stereo mode, IMUなし評価で、原著論文の精度

- aveRTE: 0.29 %
- aveRE: 1.96 deg
- RMSE APE: 0.13 m

に対して、手元の検証結果

- aveRTE: 110 %
- aveRE: 20 deg
- RMSE APE: 0.20 m

が大きく異なる原因のうち、rosbag・評価スクリプト側で確認・修正すべき要素を洗い出し、具体的なコード修正案、確認コマンド、期待される正常値を示してください。

## 調査対象の責任範囲

この依頼では、以下だけを扱ってください。

- EuRoC dataset から ROS 2 bag への変換
- rectification
- CameraInfo生成
- bag timestamp と header timestamp
- left/right同期
- static TF
- Ground Truth CSV
- evaluation log CSV
- APE/RTE/REの定義
- SE(3)/Sim(3) alignment
- pose frame変換
- quaternion順序
- `T_world_body` / `T_body_world` の向き
- 単位、percent/radian/degree
- 評価対象区間、初期化区間、tracking lost区間

Isaac ROS Visual SLAMのlaunch設定、cuVSLAM内部パラメータ、出力topic選択、IMU有効/無効設定は別AIの担当です。必要なら「別担当に確認」とだけ書いてください。

## 手元リポジトリの前提

作業ディレクトリ:

```bash
/home/hashizume/experiment
```

関連ファイル:

```bash
/home/hashizume/experiment/docs/euroc_cuvslam_preparation.md
/home/hashizume/experiment/scripts/rectify_euroc_stereo_bag.py
/home/hashizume/experiment/scripts/restamp_ros2_bag_by_header.py
/home/hashizume/experiment/scripts/diagnose_euroc_bag.py
/home/hashizume/experiment/scripts/add_camera_info_to_ros2_bag.py
/home/hashizume/experiment/src/mh01_eval_logger/mh01_eval_logger/mh01_eval_logger.py
/home/hashizume/experiment/src/mh01_eval_logger/mh01_eval_logger/mh01_metrics_node.py
/home/hashizume/experiment/MH_01_easy_leica_position.csv
/home/hashizume/experiment/results/*.csv
```

現在の準備手順:

```bash
cd /home/hashizume/experiment
python3 scripts/rectify_euroc_stereo_bag.py MH_01_easy_ros2 MH_01_easy_ros2_rectified --overwrite
python3 scripts/restamp_ros2_bag_by_header.py MH_01_easy_ros2_rectified MH_01_easy_ros2_rectified_restamped --overwrite
python3 scripts/diagnose_euroc_bag.py MH_01_easy_ros2_rectified_restamped
```

期待しているbag診断値:

- left/right image rate: 20 Hz
- bag-header offset: 0 ms
- left-right header offset: 0 us
- left frame: `cam0_rect`
- right frame: `cam1_rect`
- right CameraInfo baseline from `P`: 約 `0.110078 m`

現在の評価ノード:

- logger: `src/mh01_eval_logger/mh01_eval_logger/mh01_eval_logger.py`
- metrics: `src/mh01_eval_logger/mh01_eval_logger/mh01_metrics_node.py`

重要: `mh01_metrics_node.py` の現状実装は、評価log CSVから位置 `odom_x/y/z` だけを読み込み、GT位置にrigid alignmentしてAPEを計算しています。`compute_relative_errors()` は一定時間窓の位置差分ベクトルから `avgRTE` と `avgRE` を計算しており、姿勢quaternionは使っていません。この実装が原著論文の `aveRE` 定義と一致するかを必ず監査してください。

## 特に確認してほしい論点

1. 原著論文の指標定義との一致。
   - `aveRTE` は何mまたは何秒の相対区間で計算されているか。
   - `aveRE` は相対回転誤差なのか、移動方向ベクトルの角度なのか。
   - `avgRTE 0.29%` を再現するには、どの評価ツール・alignment・区間長が必要か。

2. 現在の `aveRTE 110%` の異常原因。
   - `relative_window_sec=1.0` が短すぎないか。
   - 静止または低速区間で `gt_distance` が小さくなり、percentが爆発していないか。
   - timestamp対応がずれて、対応するGT位置が別時刻になっていないか。
   - 座標軸やposeの向きが違っていないか。

3. APE alignment。
   - 現状は位置のみのSE(3) rigid alignment。
   - 原著がSE(3), Sim(3), yaw-only, first-pose alignment のどれか。
   - Stereoはscale既知なのでSim(3) scale補正の有無を明示すること。

4. frame変換。
   - EuRoC GTがIMU/body frameかLeica/prism frameかを確認。
   - Visual SLAM出力がcamera/base frameの場合、`T_BS`, `T_cam_imu`, `T_base_camera` を挟む必要があるか。
   - `T_world_body` と `T_body_world` の逆向きミスを検出する方法。

5. timestamp。
   - image header stamp, bag message timestamp, odometry header stamp, GT timestamp の関係。
   - `restamp_ros2_bag_by_header.py` 後に本当にbag timeとheader timeが一致しているか。
   - loggerが `msg.header.stamp` を使っていることが妥当か。
   - `max_match_dt_sec` が大きすぎて不正対応を許していないか。

6. quaternion / orientation。
   - loggerは `odom_qx/qy/qz/qw` を保存している。
   - metricsは姿勢を使っていない。
   - 原著の `aveRE` が相対回転誤差なら、GT姿勢が必要。現在の `MH_01_easy_leica_position.csv` は位置のみの可能性が高い。

7. EuRoC GTの選択。
   - Leica position CSVだけで原著のpose評価を再現できるか。
   - EuRoCの `state_groundtruth_estimate0/data.csv` を使うべきか。
   - LeicaのframeとIMU/body frameの違い。

8. rosbag内容。
   - `/cam0/image_rect`, `/cam1/image_rect`, `/cam0/camera_info`, `/cam1/camera_info`, `/tf_static` が正しいか。
   - left/right画像数、timestamp、frame_id、CameraInfo.P、baseline符号。

## 手元で実行してほしい確認コマンド例

```bash
cd /home/hashizume/experiment
python3 scripts/diagnose_euroc_bag.py MH_01_easy_ros2_rectified_restamped
head -5 MH_01_easy_leica_position.csv
tail -5 MH_01_easy_leica_position.csv
ls -lh results/*.csv
head -5 results/mh01_eval_log_*.csv
```

評価コード監査:

```bash
sed -n '1,260p' src/mh01_eval_logger/mh01_eval_logger/mh01_eval_logger.py
sed -n '1,360p' src/mh01_eval_logger/mh01_eval_logger/mh01_metrics_node.py
grep -R "relative_window_sec\\|avgRTE\\|avgRE\\|align_rigid\\|odom_q" -n \
  src/mh01_eval_logger scripts
```

bag metadata:

```bash
ros2 bag info MH_01_easy_ros2_rectified_restamped
ros2 topic list -t
ros2 topic hz /cam0/image_rect
ros2 topic hz /cam1/image_rect
ros2 topic echo --once /cam0/camera_info
ros2 topic echo --once /cam1/camera_info
```

## 期待する回答形式

以下の形式で回答してください。

```markdown
## 結論
rosbag・評価側で最も疑わしい点を3つ以内で要約。

## 優先チェックリスト
- [ ] 項目、確認コマンド、正常時の期待値、異常時の修正案

## 評価コード修正案
原著のaveRTE/aveRE/RMSE APEへ近づけるための具体的な実装修正案。
必要なら関数単位で示す。

## データ変換・bag修正案
rectification、CameraInfo、timestamp、TF、GT CSVについての修正案。

## 別担当に渡すべき事項
Isaac ROS Visual SLAM側でないと判断できない事項。
```
PROMPT
