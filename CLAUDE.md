# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

**始终用中文回答用户问题。**

工作区级别的完整说明（构建命令、启动指令、整体架构）见 `/home/gh/fastlivo2_ws/CLAUDE.md`。本文件仅记录 `fast_livo` 包特有的细节。

## 数据集

数据集目录：`~/data/uestc_slam_dataset`

## 源码结构

所有 C++ 代码位于 `src/` 和 `include/` 下。入口为 `src/main.cpp` → 构造 `LIVMapper`，在独立线程中调用 `run()`。8 个 `.cpp` 文件与 `include/` 中的头文件一一对应。

## 配置与 Launch 文件对应关系

每套传感器配置需要两个文件：
- `config/<name>.yaml` — 运行时参数（话题、外参、LIO/VIO 调参）
- `config/camera/camera_<name>.yaml` — 相机内参，由 vikit 加载

`launch/` 中的 launch 文件通过 `config_path` 和 `camera_config_path` ROS 参数指向对应 YAML。

## 关键配置项

| 配置项 | 说明 |
|---|---|
| `common/img_en`、`lidar_en` | 启用/禁用传感器流 |
| `common/slam_mode` | `0`=纯 LO，`1`=纯 LIO，`2`=LIVO |
| `vio_quality` | VIO 更新接受阈值（`tracked_min`、`inlier_min`、`inlier_ratio_min`、`mean_err_max`、`max_iters`） |
| `lio_quality` | LIO 更新接受阈值（`eff_min`、`mean_abs_max`、`rmse_max`、`min_eig_min`、`cond_max`） |
| `vio/inverse_composition_en` | 正向/逆向合成光度对齐切换 |
| `vio/exposure_estimate_en` | 在线曝光时间估计（EKF 增加 1 维状态） |

## EKF 状态

`include/common_lib.h` 中的 `StatesGroup`：19 维 —— SO3 旋转、位置、速度、陀螺仪偏置、加速度计偏置、重力方向，以及当 `exposure_estimate_en: true` 时的逆曝光时间。

## 日志输出

调试记录写入 `Log/`（由 cmake 宏 `ROOT_DIR` 在编译时确定路径）。`Log/plot.py` 用于绘图。NTU-VIRAL 评估轨迹结果在 `Log/result/ntu_viral/`。
