# GEODE Shield4 ATE 复测

ETH 原版 [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO) 的本地源码在
`/home/gh/catkin_ws/src/BIEVR-LIO`。这次使用提交 `2306022` 的原版算法，
通过 ROS1 `bievr_lio_ros/process_bag` 读取完整 Shield4 bag，使用发布的 Gamma
传感器配置。此处的评估脚本来自
[COIN-BIEVR-unofficial](https://github.com/qiyu-lu/COIN-BIEVR-unofficial/blob/22e87bcea372cd699c48c1461b222cc6ea9a8c00/scripts/evaluate_ate.py)，
保留原脚本与 [BSD-3-Clause 许可](LICENSE-COIN-BIEVR)。

## 计算 ATE

在 FAST-LIVO2 仓库根目录运行。需要 Python 3 和 NumPy，绘图时另需 Matplotlib。
以下 `estimate` 指向已经跑完的 ETH 原版轨迹，也可换成自己的 FAST-LIVO2 TUM 轨迹。

```bash
truth="$HOME/data/GEODE/groundtruth/traj/metro_tunnel/Shield_tunnel4.txt"
estimate="$HOME/fastlivo2_ws/eval_results/shield4_gamma_bievr/official_reproduction.8c1S8T/trajectory.txt"

python3 result_check/evaluate_ate.py "$estimate" "$truth" --time-offset 0 --json
python3 result_check/evaluate_ate.py "$estimate" "$truth" --time-offset auto --json
```

轨迹用 TUM 格式 `t x y z qx qy qz qw`，时间单位为秒。评估将高频轨迹线性插值
到低频轨迹的时间戳，然后做无尺度 SE(3) 刚体对齐，计算位置 RMSE。
`--time-offset auto` 为**每条估计轨迹**搜索一个加到估计时间戳上的常数，
以位置 ATE RMSE 最小为准。轨迹文件保持原样。本次采用 COIN-BIEVR 的 GEODE
参数，不传 `--lever-arm`。

| ETH 原版 Shield4 | ATE RMSE | 真值匹配数 | 加到估计时间戳的时移 |
|---|---:|---:|---:|
| 原始时间戳 | 1.860956 m | 901 | 0 s |
| COIN-BIEVR 自动拟合 | 0.489681 m | 907 | −1.148 s |

**0.490 m 对应上表的自动拟合。** 这一列依赖每条轨迹拟合的时移，比较方法时也可用
固定秒数（例如 `--time-offset=-1.20`），并检查匹配数和共同真值区间。
[ETH 论文 Table IV](https://arxiv.org/html/2604.14421v2#S4.T4) 的 0.275 m
来自论文实验。

## 从原始 bag 重跑 ETH 轨迹

需要已经编译好的 ROS Noetic 原版工作区。下面使用独立 ROS master 端口，
在临时目录写轨迹和运行日志。

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
eth="$HOME/catkin_ws/src/BIEVR-LIO"
bag="$HOME/data/GEODE/sensor_data/metro_tunnel/Shield_tunnel4_gamma.bag"
run_dir=$(mktemp -d /tmp/bievr-shield4.XXXXXX)
estimate="$run_dir/trajectory.txt"

sed -E \
  -e 's/^  dashboard: True/  dashboard: False/' \
  -e "s@^  trajectory_path:.*@  trajectory_path: \"$estimate\"@" \
  "$eth/config/params.yaml" > "$run_dir/params.yaml"

ROS_MASTER_URI=http://127.0.0.1:11321 ROS_LOG_DIR="$run_dir/ros" \
  roslaunch -p 11321 "$eth/interfaces/ros1/launch/process_bag.launch" \
    sensor_config:="$eth/config/sensor_configs/gamma.yaml" \
    params:="$run_dir/params.yaml" rosbag:="$bag" rviz:=false
```

完整回放输出 3,402 个位姿。随后用上节的 `evaluate_ate.py` 命令评估
`$estimate`。
