# MH_01 Evaluation Logger

`MH_01_easy_leica_position.csv`をground truthとして読み込み、`/visual_slam/tracking/odometry`を受信するたびに同じ時刻のground truth位置を線形補間してCSVへ保存します。CSVには生座標の誤差に加えて、初回サンプルを原点にした相対座標の誤差も出力します。

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

MH_01のrosbagを再生します。

```bash
ros2 bag play ~/experiment/MH_01_easy_ros2 --clock
```

出力先はデフォルトで`~/experiment/results/mh01_eval_log_YYYYmmdd_HHMMSS.csv`です。

## Metrics

保存済みログから`avgRTE [%]`、`avgRE [deg]`、`RMSE APE [m]`を計算し、CSVとPNGを出力します。`RMSE APE`はx/y/zを含む3D誤差です。CSVには軸ごとの`RMSE_X`、`RMSE_Y`、`RMSE_Z`、平均絶対誤差、最大絶対誤差も出力します。

```bash
cd ~/experiment
source install/setup.bash
ros2 launch mh01_eval_logger mh01_metrics.launch.py
```

デフォルトでは`results/mh01_eval_log_20260528_150755.csv`を評価し、次のファイルを作成します。

- `results/mh01_eval_log_20260528_150755_metrics_summary.csv`
- `results/mh01_eval_log_20260528_150755_metrics_plot.png`

計算では、odometry軌跡をground truth軌跡へスケール固定の剛体整列（回転+並進）してから評価します。`avgRTE`と`avgRE`は`relative_window_sec`秒離れた2点間の相対移動で計算し、デフォルトは1.0秒です。ロスト/再初期化後の影響を分けるため、全体評価に加えて`segment_end_sec`秒までの初期区間を別アラインして評価します。

## Parameters

- `ground_truth_csv`: ground truth CSVのパス
- `odom_topic`: ログ対象のOdometryトピック
- `output_dir`: 出力CSVディレクトリ
- `max_match_dt_sec`: odometry時刻と補間に使うground truth時刻が離れている場合に警告する閾値
- `qos_reliability`: Odometry購読QoS。受信できない場合は`best_effort`も試す

metricsノードの追加パラメータ:

- `eval_log_csv`: 評価対象のログCSV
- `relative_window_sec`: `avgRTE`/`avgRE`の相対評価窓
- `segment_end_sec`: 初期区間だけを再アライン評価する終了時刻
- `jump_threshold_m`: 推定軌跡のジャンプとして数える1サンプル間の移動量
- `plot`: PNG可視化を保存するか
