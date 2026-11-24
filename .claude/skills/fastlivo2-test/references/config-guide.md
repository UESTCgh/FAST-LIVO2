# FAST-LIVO2 配置参数参考

## Launch 文件与配置文件对应关系

| Launch 文件 | 全局配置（rosparam load） | 节点配置（rosparam file） |
|---|---|---|
| `mapping_super2.launch` | `config/mid360.yaml` | `config/camera/super2_d435i.yaml` |
| `mapping_avia.launch` | `config/avia.yaml` | `config/camera/camera_pinhole.yaml` |
| `mapping_avia_marslvig.launch` | `config/MARS_LVIG.yaml` | `config/camera/camera_MARS_LVIG.yaml` |
| `mapping_ouster_ntu.launch` | `config/NTU_VIRAL.yaml` | `config/camera/camera_NTU_VIRAL.yaml` |
| `mapping_hesaixt32_hilti22.launch` | `config/HILTI22.yaml` | `config/camera/camera_fisheye_HILTI22.yaml` |
| `mapping_geode_beta.launch` | `config/geode_beta.yaml` | `config/camera/camera_geode_beta.yaml` |

## 话题配置

### guandao1.bag（mid360 配置）
```yaml
common:
  img_topic: "/synced_image"
  lid_topic: "/synced_lidar"
  imu_topic: "/livox/imu"
  propgrate_topic: "/LIVO2/imu_propagate"  # 用于时间同步
```

### 标准 Avia 配置
```yaml
common:
  img_topic: "/left_camera/image"
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
```

### GEODE 盾构/掘进隧道配置（geode_beta.yaml）
```yaml
common:
  img_topic: "/left_camera/image"   # launch 中 republish 从 /compressed 解压
  lid_topic: "/ouster/points"
  imu_topic: "/imu/data"

preprocess:
  lidar_type: 3   # Ouster OS1-16
  scan_line: 16

# 隧道场景 VIO 质量阈值（默认值过严，导致 100% Skip VIO）
# 建议放宽：
vio_quality:
  tracked_min: 20      # 默认 60，隧道特征稀少
  inlier_min: 15       # 默认 40
  mean_err_max: 20.0   # 默认 10.0，隧道光度误差偏大（实测 10-11）
```

真值文件：`~/data/GEODE/Shield_tunnel7.txt`（TUM，1333 帧，坐标系偏移约 1000m，评估需 `--align`）

## 位姿输出配置

```yaml
evo:
  seq_name: "my_sequence"      # 输出文件名（不含 .txt）
  pose_output_en: true         # 必须为 true 才能保存轨迹
```

输出路径：`src/FAST-LIVO2/Log/result/<seq_name>.txt`
格式：TUM（时间戳 x y z qx qy qz qw）

## LiDAR 类型值

| 值 | LiDAR |
|---|---|
| 1 | Livox Avia |
| 2 | Velodyne VLP-16 |
| 3 | Ouster OS1-64 |
| 4 | Intel L515 |
| 5 | Hesai XT32 |
| 6 | Pandar 128 |
| 7 | RoboSense |

## 关键调参参数

### VIO
- `max_iterations`：视觉优化迭代次数（默认 3-5）
- `outlier_threshold`：外点剔除的光度误差阈值
- `img_point_cov`：图像点协方差（100-1000）
- `patch_size`：视觉 patch 大小（默认 8）

### LIO
- `max_iterations`：LiDAR 优化迭代次数（默认 4-5）
- `voxel_size`：体素地图分辨率（越小越密，默认 0.25-0.5）
- `min_eigen_value`：平面有效性的最小特征值

### IMU
- `acc_cov` / `gyr_cov`：加速度计/陀螺仪噪声协方差
- `b_acc_cov` / `b_gyr_cov`：偏置随机游走协方差
