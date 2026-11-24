# tool/ — 轨迹评估与对比

| 文件 | 作用 |
|---|---|
| `run_sequence.sh` | 用 `rosrun`（不走 roslaunch）跑一个 bag，把 TUM 轨迹收到 `eval_results/`。支持通过环境变量指向另一个 catkin workspace 的二进制，用于跑原版基准。 |
| `convert_gt.py` | 把 GEODE 提供的全站仪真值（`Tunneling_tunnelN.txt`，四元数为 0 0 0 0）转成 evo 可读的 TUM（四元数写成 0 0 0 1）。 |
| `eval_trajectory.py` | 多序列 × 多版本 vs 真值：ATE / RPE（平移，SE(3) Umeyama 对齐），evo 风格 RPE 着色 3D 轨迹图、拼图、CSV/Markdown 指标表（含相对基准的变化百分比）。 |
| `eval_results/` | 轨迹与报告输出。`report/` 里是最新一次完整对比。 |

## 前置

- `roscore` 已启动；bag 里图像是压缩格式时，需要一个 `image_transport republish` 节点在跑（`mapping_geode_*.launch` 会自带一个 `respawn=true` 的，跑序列期间别关它）。
- `python3` 需要 `evo`（`pip install evo`）、`matplotlib`、`Pillow`。

## 完整流程（以 GEODE Tunnel4/5 为例）

```bash
cd src/FAST-LIVO2

# 1. 真值 → TUM
python3 tool/convert_gt.py            # 写 eval_results/tunnel{4,5}_gt.txt

# 2. 修改版（当前 workspace 的编译产物）
tool/run_sequence.sh config/geode_beta.yaml config/camera/camera_geode_beta.yaml \
    /home/gh/data/GEODE/Tunneling_tunnel4_beta.bag tunnel4_modified tool/eval_results
tool/run_sequence.sh config/geode_beta.yaml config/camera/camera_geode_beta.yaml \
    /home/gh/data/GEODE/Tunneling_tunnel5_beta.bag tunnel5_modified tool/eval_results

# 3. 原版基准（main 分支 worktree，单独 workspace 编译，链到本 ws 拿 vikit）
#    git worktree add /home/gh/fastlivo2_orig_ws/src/FAST-LIVO2 main
#    cp config/geode_beta.yaml config/camera/camera_geode_beta.yaml /home/gh/fastlivo2_orig_ws/src/FAST-LIVO2/config/
#    (source devel/setup.bash; cd /home/gh/fastlivo2_orig_ws && catkin_make -DCMAKE_BUILD_TYPE=Release)
FASTLIVO_WS=/home/gh/fastlivo2_orig_ws FASTLIVO_PKG_DIR=/home/gh/fastlivo2_orig_ws/src/FAST-LIVO2 \
tool/run_sequence.sh config/geode_beta.yaml config/camera/camera_geode_beta.yaml \
    /home/gh/data/GEODE/Tunneling_tunnel4_beta.bag tunnel4_original tool/eval_results
FASTLIVO_WS=/home/gh/fastlivo2_orig_ws FASTLIVO_PKG_DIR=/home/gh/fastlivo2_orig_ws/src/FAST-LIVO2 \
tool/run_sequence.sh config/geode_beta.yaml config/camera/camera_geode_beta.yaml \
    /home/gh/data/GEODE/Tunneling_tunnel5_beta.bag tunnel5_original tool/eval_results

# 4. 评估 + 出图
python3 tool/eval_trajectory.py --out tool/eval_results/report --baseline Original \
    --seq Tunnel4 tool/eval_results/tunnel4_gt.txt \
          Original=tool/eval_results/tunnel4_original.txt Modified=tool/eval_results/tunnel4_modified.txt \
    --seq Tunnel5 tool/eval_results/tunnel5_gt.txt \
          Original=tool/eval_results/tunnel5_original.txt Modified=tool/eval_results/tunnel5_modified.txt
```

输出到 `--out`：每个 (序列, 版本) 一张 `*_RPE.png` 和 `*_ATE.png`，`grid_RPE.png` 拼图（行=序列，列=版本），`metrics.md` / `metrics.csv` / `metrics_summary.txt`。

## 两个容易踩的坑

**`rosbag play --clock` 不要加。** bag 里没有 `/clock`，配合默认 `use_sim_time=false` 会让节点一直缓存消息不做同步，内存几分钟涨到 20 GB+ 且没有任何输出。

**位置-only 真值的 RPE。** 全站仪真值没有姿态。evo 默认的 RPE 把估计的相对平移表达在估计的机体系里、真值的表达在世界系里，对隧道这种来回走的轨迹会给出接近 2·delta 的假误差（0.2 m 步长算出 0.67 m）。`eval_trajectory.py` 检测到真值姿态全为单位四元数时，会自动把估计的姿态也置为单位，让 RPE 变成纯位置漂移 |(p_est_j − p_est_i) − (p_ref_j − p_ref_i)|。`--position_only` 可强制。

## 真值本身的噪声

`Tunneling_tunnel{4,5}.txt` 采样约 3 Hz，含 55 / 150 个位置零变化的重复样本（全站仪没跟上），相对 7 点滑动平均的残差 RMSE 约 0.13 m。因此 ATE RMSE 的下限本身就在 0.3 m 量级，不同算法版本之间 <1 cm 的 ATE 差别是真值噪声之下的，**不能据此判断优劣**；RPE 对这类噪声更敏感，同样要谨慎解读。
