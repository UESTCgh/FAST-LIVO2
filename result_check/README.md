# GEODE Shield3 / Shield4 ATE 复测

本目录默认采用本地 ETH COIN-BIEVR `c26f92d` 的 Shield3 / Shield4 复现口径。评估器源自 [COIN-BIEVR-unofficial](https://github.com/qiyu-lu/COIN-BIEVR-unofficial/blob/22e87bcea372cd699c48c1461b222cc6ea9a8c00/scripts/evaluate_ate.py)，保留其 [BSD-3-Clause 许可](LICENSE-COIN-BIEVR)。[算法复现和运行参数](/home/gh/catkin_ws/reproduction/coin_bievr/official/README.md)

## 运行评估

在 FAST-LIVO2 仓库根目录运行。需要 Python 3 和 NumPy，绘图时还需要 Matplotlib。将 `estimate` 换成需要评估的 FAST-LIVO2 TUM 轨迹，`shield` 选择对应的真值序列。

```bash
shield=4  # 3 或 4
truth="$HOME/data/GEODE/groundtruth/traj/metro_tunnel/Shield_tunnel${shield}.txt"
estimate="$HOME/catkin_ws/reproduction/coin_bievr/official/shield-tunnel${shield}-gamma/traj.txt"

# 主结果，默认等价于 --time-offset auto
python3 result_check/evaluate_ate.py "$estimate" "$truth" --json --plot ate.png > eval.json

# 原始时间戳对照
python3 result_check/evaluate_ate.py "$estimate" "$truth" --time-offset 0 --json > eval-zero-offset.json
```

- 用 TUM 格式 `t x y z qx qy qz qw` 读取估计轨迹和全站仪位置真值。Shield3 / Shield4 在真值时刻线性插值估计位置，只使用有效重叠区间，最大插值间隔为 1 秒。
- 对每条估计轨迹搜索一个**加到估计时间戳**的常量，以全程三维位置 ATE RMSE 最小为准。搜索对每个候选时移执行无尺度 SE(3) 刚体对齐。匹配点数可能随时移变化，比较结果时一并记录 `time_offset` 和 `n_pairs`。
- 复现口径的 IMU 到棱镜杆臂取零，命令省略 `--lever-arm`。时移只应用在评估读取的轨迹副本中。

## 核对结果

下表使用本地 ETH COIN-BIEVR 官方版复现保存的轨迹。原始时间戳列保留同一评估器、杆臂和空间对齐设置，时移设为零。

| 序列 | 输出位姿 | 自动拟合时移 | 自动拟合匹配数 | 自动拟合 ATE RMSE | 零时移匹配数 | 零时移 ATE RMSE |
|---|---:|---:|---:|---:|---:|---:|
| Shield3 | 3,386 | −0.717 s | 919 / 919 | 0.181 m | 919 / 919 | 1.175 m |
| Shield4 | 3,402 | −1.238 s | 907 / 911 | 0.171 m | 901 / 911 | 1.944 m |

[Shield3 指标](/home/gh/catkin_ws/reproduction/coin_bievr/official/shield-tunnel3-gamma/eval.json) · [Shield4 指标](/home/gh/catkin_ws/reproduction/coin_bievr/official/shield-tunnel4-gamma/eval.json) · [论文 Table I](https://icra2026-rigorous-perception.github.io/pdf/pfreundschuh2026.pdf#page=3)

## 从原始 bag 生成 ETH COIN-BIEVR 轨迹

需要已编译的 ROS Noetic 官方 COIN-BIEVR 工作区。下列命令使用已保存的 Gamma 配置和默认强度参数，在新目录保存轨迹与日志。运行完毕后，将评估命令的 `estimate` 指向新轨迹。

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/.local/share/coin_bievr/official_ws/devel/setup.bash"

shield=4  # 3 或 4
saved="$HOME/catkin_ws/reproduction/coin_bievr/official/shield-tunnel${shield}-gamma"
bag="$HOME/data/GEODE/sensor_data/metro_tunnel/Shield_tunnel${shield}_gamma.bag"
run_dir=$(mktemp -d /tmp/coin-bievr-shield.XXXXXX)
estimate="$run_dir/traj.txt"

sed -E "s@^  trajectory_path:.*@  trajectory_path: \"$estimate\"@" \
  "$saved/params.yaml" > "$run_dir/params.yaml"

port=$(python3 -c 'import socket; s = socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1])')
ROS_MASTER_URI="http://127.0.0.1:$port" ROS_LOG_DIR="$run_dir/ros" \
  roslaunch -p "$port" coin_bievr_ros process_bag.launch \
    "sensor_config:=$saved/gamma.yaml" "params:=$run_dir/params.yaml" "rosbag:=$bag" \
    > "$run_dir/run.log" 2>&1
```

此前 ETH BIEVR-LIO `2306022` 的纯几何复测记录另有归档。[历史复测](/home/gh/fastlivo2_ws/eval_results/shield4_gamma_bievr/official_reproduction.8c1S8T/README.md)
