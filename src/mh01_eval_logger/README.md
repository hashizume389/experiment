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

## Parameters

- `ground_truth_csv`: ground truth CSVのパス
- `odom_topic`: ログ対象のOdometryトピック
- `output_dir`: 出力CSVディレクトリ
- `max_match_dt_sec`: odometry時刻と補間に使うground truth時刻が離れている場合に警告する閾値
