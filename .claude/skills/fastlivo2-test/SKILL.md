---
name: fastlivo2-test
description: This skill should be used when the user asks to "run FAST-LIVO2", "test FAST-LIVO2", "play bag file", "evaluate localization accuracy", "test SLAM accuracy", "run LIVO mapping", "evaluate trajectory", "debug SLAM divergence", or mentions running fastlivo2 with a rosbag and evaluating results.
version: 1.2.0
---

# FAST-LIVO2 运行与评估 Skill

本 skill 自动化完成 FAST-LIVO2 SLAM 系统的完整流程：编译、运行系统、播放 rosbag 数据集、使用 evo 工具评估定位精度。

**始终用中文回答用户问题。**

## 适用场景

- 编译或重新构建 FAST-LIVO2
- 使用 rosbag 数据集运行 FAST-LIVO2
- 评估 SLAM 定位/轨迹精度
- 在 guandao 或其他数据集上测试 FAST-LIVO2
- 诊断轨迹发散或 VIO 退化问题

## 工作区信息

- **工作区**：`/home/gh/fastlivo2_ws`（ROS Noetic catkin 工作区）
- **FAST-LIVO2 包**：`src/FAST-LIVO2/`（包名：`fast_livo`）
- **数据集目录**：`~/data/uestc_slam_dataset`（旧）、`~/data/GEODE/`（GEODE 隧道数据集）
- **可用 bag**：
  - `dataset/guandao1.bag`（161s，18GB）、`dataset/guandao2.bag`（89s，9.1GB）
  - `~/data/GEODE/Shield_tunnel7_beta.bag`（516s，18.1GB，盾构隧道）
  - `~/data/GEODE/Shield_tunnel8_beta.bag`、`Shield_tunnel9_beta.bag`
  - `~/data/GEODE/Tunneling_tunnel4_beta.bag`、`Tunneling_tunnel5_beta.bag`
- **Launch 文件**：`mapping_avia.launch`、`mapping_super2.launch`、`mapping_ouster_ntu.launch`、`mapping_geode_beta.launch`
- **位姿输出**：TUM 格式，路径为 `src/FAST-LIVO2/Log/result/<seq_name>.txt`
- **评估工具**：`evo`（已安装：`evo_ape`、`evo_rpe`、`evo_traj`）
- **历史结果**：`eval_results/` 目录

## 重要：Shell 环境

ROS 命令必须在 bash 子 shell 中运行，默认 zsh 与 `setup.bash` 存在兼容问题。始终使用以下方式包装：

```bash
bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && <ros命令>'
```

不要在 zsh 中直接使用 `rosls`、`rospack`，会静默失败。

## 执行流程

### 第一步：预检与编译

1. 确认 bag 文件存在
2. 检查 `devel/setup.bash` 是否存在（工作区已构建）
3. 若代码有改动或首次运行，**只编译 fast_livo 包**（速度更快）：
   ```bash
   cd /home/gh/fastlivo2_ws && catkin_make --pkg fast_livo
   ```
4. 若需全量构建：
   ```bash
   cd /home/gh/fastlivo2_ws && catkin_make
   ```
5. 验证 `evo` 可用：`which evo_ape`
6. 验证包可找到：`bash -c 'source devel/setup.bash && rospack find fast_livo'`

### 第二步：启用位姿输出

修改对应 YAML 配置以启用轨迹记录：

```yaml
evo:
  seq_name: "<描述性名称>"
  pose_output_en: true
```

配置文件与 launch 文件的对应关系：
- `mapping_super2.launch` 加载 `config/mid360.yaml`（rosparam load）+ `config/camera/super2_d435i.yaml`（节点 rosparam）
- `mapping_avia.launch` 加载 `config/avia.yaml` + `config/camera/camera_pinhole.yaml`
- `mapping_geode_beta.launch` 加载 `config/geode_beta.yaml` + `config/camera/camera_geode_beta.yaml`（含 image_transport republish 节点，将 `/left_camera/image/compressed` 解压为 `/left_camera/image`）

**注意**：测试完成后恢复原始配置（`pose_output_en: false`）。

### 第三步：启动流程

所有命令在 bash 子 shell 中执行：

```bash
# 3a：roscore（先检查是否已运行）
bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && roscore' &

# 3b：FAST-LIVO2（等待 roscore 启动 3s）
bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && roslaunch fast_livo mapping_super2.launch rviz:=false' &

# 3c：播放 bag（等待 FAST-LIVO2 初始化 8s）
bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && rosbag play /home/gh/fastlivo2_ws/dataset/guandao1.bag --clock'
```

`guandao1.bag` 关键话题：
- `/synced_image`（1610 条）— 同步相机图像
- `/synced_lidar`（1610 条）— 同步 LiDAR（MID-360）
- `/livox/imu`（32201 条）— IMU 数据，约 200Hz
- `/LIVO2/imu_propagate`（32194 条）— 传播里程计

`mid360.yaml` 订阅 `/synced_image`、`/synced_lidar`、`/livox/imu`。

`Shield_tunnel7_beta.bag`（GEODE 盾构隧道）关键话题：
- `/left_camera/image/compressed`（5168 条）— 压缩图像，launch 中 republish 解压为 `/left_camera/image`
- `/ouster/points`（5168 条）— Ouster LiDAR 点云（`lidar_type: 3`，16 线）
- `/imu/data`（51681 条）— IMU，约 100Hz
- `/ouster/imu`（51676 条）— Ouster 内置 IMU（未使用）

`geode_beta.yaml` 订阅 `/left_camera/image`、`/ouster/points`、`/imu/data`。

GEODE 数据集播放命令（2倍速）：
```bash
bash -c 'source /home/gh/fastlivo2_ws/devel/setup.bash && rosbag play ~/data/GEODE/Shield_tunnel7_beta.bag -r 2 --clock'
```

### 第四步：监控、等待并收集结果

**运行中**：监控 FAST-LIVO2 日志中的警告：
- `Skip VIO Update!!!` — VIO 质量检查失败，视觉更新被跳过
- `VIO_QUALITY ... vio_ok=0` — 跟踪/内点数低于阈值
- 注意轨迹输出中的位置突变

**bag 播放完毕后**：等待 5-10 秒，然后 `killall -INT roslaunch`。

输出轨迹：`src/FAST-LIVO2/Log/result/<seq_name>.txt`，TUM 格式：
```
时间戳 x y z qx qy qz qw
```

### 第五步：精度评估

#### 5a：健全性检查——检测发散

```bash
evo_traj tum <traj.txt> --full_check
```

**发散红旗**：
- 路径长度远超预期（如 160s 室内外测试出现 40km）
- 平均速度不合理（如手持/车载超过 10 m/s）
- 终点距起点数千米

若检测到发散，定位发散帧：

```bash
awk 'BEGIN{f=0}{x=$2;y=$3;z=$4;d=sqrt(x*x+y*y+z*z);if(d>100&&f==0){print "发散行 "NR" t="$1" dist="d;f=1}}' <traj.txt>
```

提取正常部分：`head -<N> traj.txt > traj_good.txt`

#### 5b：无真值情况

```bash
evo_traj tum <traj.txt> --full_check --plot_mode xy --save_plot traj.pdf
```

注意：`evo_rpe tum A A`（同一文件两次）结果始终为零，无意义。

#### 5c：有真值情况

```bash
# 绝对位姿误差
evo_ape tum <gt.txt> <traj.txt> --align --correct_scale \
    --save_results ape_result.zip --save_plot ape.pdf

# 相对位姿误差
evo_rpe tum <gt.txt> <traj.txt> --align --correct_scale \
    --delta 1 --delta_unit m --save_results rpe_result.zip
```

NTU-VIRAL 数据集需先用 `Log/result/ntu_viral/evaluate_viral.py` 转换坐标。

### 第六步：汇报结果

汇报内容包括：
- APE/RPE 统计（均值、中位数、RMSE、标准差、最小值、最大值）
- 轨迹位姿数量 vs 预期（guandao: 1610 帧；GEODE Shield_tunnel7: 5163 帧）
- 是否发散及发散帧/时刻
- VIO 跳过率（统计日志中 `Skip VIO Update` 次数）
- 轨迹图路径（若已生成）

GEODE 数据集有真值文件：`~/data/GEODE/Shield_tunnel7.txt`（TUM 格式，1333 帧，坐标系偏移约 1000, 2001, 10m，需 `--align`）。

### 第七步：收尾清理

1. 恢复配置：YAML 中设置 `pose_output_en: false`
2. 终止残留 ROS 进程：`killall -q roscore rosmaster`
3. 复制结果到 `eval_results/<seq_name>/`

## 已知问题

### guandao1.bag（mid360 配置）

- **约 75s 处轨迹发散**：第 752 帧（t=1769934130.6）附近发散，前 752 帧正常（路径 118.7m），之后位置爆炸至数万公里。
- **频繁 VIO 跳过**：跟踪特征数远低于阈值 60，系统退化为纯 LIO。
- **最大速度尖峰**：正常段最大速度 43.5 m/s，存在短暂不稳定。

### GEODE Shield_tunnel7_beta.bag（geode_beta 配置，2026-05-11 测试）

- **VIO 100% 跳过**：623/623 帧全部 `Skip VIO Update`，系统完全退化为纯 LIO。
  - **根因**：隧道场景光度误差偏大（meanErr 常在 10-11），`vio_quality/mean_err_max=10.0` 阈值过严；同时 `tracked_min=60` 而实际 tracked 数量分布较低（中位数约 30-50）。
  - **修复方向**：将 `mean_err_max` 放宽至 15-20，将 `tracked_min` 降至 20-30，`inlier_min` 降至 15-20。
- **APE RMSE = 88.3m**（对比 `Shield_tunnel7.txt` 真值，Sim3 对齐）：纯 LIO 在 516s 长隧道中累积漂移严重。
- **轨迹统计**：5163 帧，路径长度 3127m，平均速度 6.1 m/s，最大速度 20.2 m/s，无发散。
- **真值坐标系**：GT 起点约 (1000, 2001, 10)，评估时必须加 `--align --correct_scale`。

## 附加资源

### 脚本
- **`scripts/run_fastlivo2_test.sh`** — 端到端自动化测试脚本，含清理和 evo 评估
- 支持环境变量：`BAG_RATE`、`RVIZ`、`WAIT_AFTER_BAG`

### 参考文档
- **`references/config-guide.md`** — 不同传感器配置的参数参考
- **`references/troubleshooting.md`** — 发散诊断与调参指南
