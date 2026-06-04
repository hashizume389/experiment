# MH_01 Evaluation Logger

EuRoCの`state_groundtruth_estimate0/data.csv`、または従来の`MH_01_easy_leica_position.csv`をground truthとして読み込み、`/visual_slam/tracking/odometry`を受信するたびに同じ時刻のground truthを補間してCSVへ保存します。

原著cuVSLAM Table 2のStereo EuRoC Odom行と比較する場合、`/visual_slam/tracking/odometry`を`T_odom_imu0`、つまり`T_world_body`相当としてそのまま評価します。逆行列は取りません。Visual SLAM側は`base_frame:=imu0`、odometryの`child_frame_id:=imu0`にしてください。

## Build

```bash
cd ~/experiment
colcon build --packages-select mh01_eval_logger
source install/setup.bash
```

## Run

Visual SLAM再生モードを起動したあと、別ターミナルでログノードを起動します。

```bash
cd ~/experiment
source install/setup.bash
ros2 launch mh01_eval_logger mh01_eval_logger.launch.py
```

EuRoC pose GTを使う場合:

```bash
ros2 launch mh01_eval_logger mh01_eval_logger.launch.py \
  ground_truth_csv:=/path/to/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv
```

MH_01のrosbagを再生します。

```bash
ros2 bag play ~/experiment/MH_01_easy_ros2_rectified_restamped --clock
```

出力先はデフォルトで`~/experiment/results/mh01_eval_log_YYYYmmdd_HHMMSS.csv`です。

## Metrics

保存済みログから`avgRTE [%]`、`avgRE [deg]`、`RMSE APE [m]`を計算し、CSVとPNGを出力します。`avgRE`はquaternionから相対回転誤差として計算するため、原著比較ではEuRoC pose GTを指定してください。位置のみGTはロギングには使えますが、metricsノードでは原著の`avgRE`を再現できません。

```bash
cd ~/experiment
source install/setup.bash
ros2 launch mh01_eval_logger mh01_metrics.launch.py
```

デフォルトでは`results/mh01_eval_log_20260528_150755.csv`を評価し、次のファイルを作成します。

- `results/mh01_eval_log_20260528_150755_metrics_summary.csv`
- `results/mh01_eval_log_20260528_150755_metrics_plot.png`

計算では、odometry軌跡をground truth軌跡へ整列してから評価します。デフォルトは原著APE条件に寄せた`alignment:=sim3`です。Stereoの物理スケール確認では`alignment:=se3`も併記してください。`avgRTE`と`avgRE`はデフォルトで`relative_delta:=1.0 relative_delta_unit:=m`の距離ベース相対poseから計算します。`relative_delta_unit:=s`を指定した場合だけ`relative_window_sec`を使います。

## Parameters

- `ground_truth_csv`: EuRoC pose GT CSV、または従来の位置GT CSVのパス
- `odom_topic`: ログ対象のOdometryトピック
- `output_dir`: 出力CSVディレクトリ
- `max_match_dt_sec`: odometry時刻と補間に使うground truth時刻が離れている場合に警告する閾値
- `qos_reliability`: Odometry購読QoS。受信できない場合は`best_effort`も試す

metricsノードの追加パラメータ:

- `eval_log_csv`: 評価対象のログCSV
- `relative_delta`: `avgRTE`/`avgRE`の相対評価delta
- `relative_delta_unit`: `m`、`s`、または`frames`
- `relative_window_sec`: `relative_delta_unit:=s`のときの相対評価窓
- `alignment`: 軌跡整列。`sim3`または`se3`
- `min_relative_distance_m`: 低速/短距離区間でpercentが爆発するのを避ける最小距離
- `segment_end_sec`: 初期区間だけを再アライン評価する終了時刻
- `jump_threshold_m`: 推定軌跡のジャンプとして数える1サンプル間の移動量
- `plot`: PNG可視化を保存するか
