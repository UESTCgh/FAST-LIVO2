# FAST-LIVO2 故障排查指南

## 轨迹发散诊断

### 症状
- 位置值爆炸至数千米
- `evo_traj --full_check` 显示路径长度或速度不合理
- 系统突然丢失跟踪

### 诊断步骤

1. **定位发散点**：
   ```bash
   awk 'BEGIN{f=0}{x=$2;y=$3;z=$4;d=sqrt(x*x+y*y+z*z);if(d>100&&f==0){print "发散行 "NR" t="$1" dist="d;f=1}}' trajectory.txt
   ```

2. **检查 VIO 跳过率**：
   ```bash
   grep -c "Skip VIO Update" /tmp/fastlivo2.log
   grep "VIO_QUALITY" /tmp/fastlivo2.log | tail -20
   ```

3. **检查配置中的 VIO 质量阈值**：
   ```yaml
   vio_quality:
     tracked_min: 60     # 最少跟踪特征数
     inlier_min: 40      # 优化后最少内点数
     inlier_ratio_min: 0.5
     mean_err_max: 10.0  # 最大平均光度误差
     max_iters: 10
   ```

4. **检查日志中的 LIO 质量**：
   ```bash
   grep "LIO_QUALITY\|eff_min\|mean_abs" /tmp/fastlivo2.log | tail -20
   ```

### 常见原因与修复

#### 1. VIO 退化（频繁 `Skip VIO Update`）
- **原因**：跟踪特征太少、无纹理场景、运动模糊
- **修复**：降低 `vio_quality/tracked_min` 和 `inlier_min` 阈值
- **修复**：增大 `vio/outlier_threshold` 以更宽松地接受特征
- **修复**：若图像有时间偏移，调整 `time_offset/img_time_offset`

#### 2. 纯 LiDAR 退化
- **原因**：VIO 被跳过时，LIO 单独处理状态估计。在几何退化场景（长走廊、开阔空间）中 LIO 可能发散
- **修复**：通过调整质量阈值保持 VIO 活跃
- **修复**：增大 `lio/min_eigen_value` 以拒绝退化平面

#### 3. 时间同步问题
- **原因**：LiDAR、IMU、相机时间戳不匹配
- **症状**：IMU 预积分漂移、突然跳变
- **修复**：检查 `time_offset/img_time_offset`（默认 0.1s）
- **修复**：验证 `common/ros_driver_bug_fix` 设置

#### 4. IMU 噪声参数
- **原因**：IMU 协方差过低（过度自信）或过高（权重不足）
- **修复**：调整 `imu/acc_cov`、`imu/gyr_cov` 以匹配实际传感器噪声
- **典型值**：acc_cov=0.2-0.5，gyr_cov=0.3-0.5

#### 5. 外参标定误差
- **原因**：LiDAR-相机或 LiDAR-IMU 外参错误
- **修复**：使用 FAST-Calib 或 livox_camera_calib 重新标定
- **配置**：检查 YAML 中的 `extrin_calib/Rcl`、`Pcl`

## 性能调优

### 提高精度
- 增大 `lio/max_iterations`（5-10）
- 增大 `vio/max_iterations`（5-10）
- 减小 `lio/voxel_size`（0.1-0.25）
- 增大 `lio/max_points_num`（100-200）

### 提高速度
- 减小 `lio/max_iterations`（3-4）
- 减小 `vio/max_iterations`（3）
- 增大 `lio/voxel_size`（0.5-1.0）
- 增大 `preprocess/point_filter_num`（2-4）
- 减小 `vio/patch_pyrimid_level`（2-3）

## guandao1.bag 测试结果（2026-03-05）

| 指标 | 完整（1606 帧） | 正常段（752 帧） |
|--------|-------------------|--------------------------|
| 时长 | 160.5s | 75.1s |
| 路径长度 | 40,971m（已发散） | 118.7m |
| 平均速度 | 255 m/s（无效） | 1.58 m/s |
| 最大速度 | 931 m/s（无效） | 43.5 m/s（偏高） |
| 发散帧 | 753 | N/A |
| 使用配置 | mid360.yaml + super2_d435i.yaml | |
| Launch 文件 | mapping_super2.launch | |

## GEODE Shield_tunnel7_beta.bag 测试结果（2026-05-11）

| 指标 | 值 |
|--------|---|
| 时长 | 516s |
| 帧数 | 5163 帧 |
| 路径长度 | 3127m |
| 平均速度 | 6.1 m/s |
| 最大速度 | 20.2 m/s |
| 发散 | 无 |
| VIO 跳过率 | **100%**（623/623 帧全部 Skip） |
| APE RMSE（vs GT） | 88.3m（Sim3 对齐） |
| 使用配置 | geode_beta.yaml + camera_geode_beta.yaml |
| Launch 文件 | mapping_geode_beta.launch |
| 播放速率 | -r 2 |

**根因**：`vio_quality/mean_err_max=10.0` 过严，隧道场景实测 meanErr 常在 10-11；同时 `tracked_min=60` 而隧道特征稀少。系统完全退化为纯 LIO，导致 516s 长隧道累积漂移 88m RMSE。

**建议调参**（geode_beta.yaml）：
```yaml
vio_quality:
  tracked_min: 20      # 60 → 20
  inlier_min: 15       # 40 → 15
  mean_err_max: 20.0   # 10.0 → 20.0
```
