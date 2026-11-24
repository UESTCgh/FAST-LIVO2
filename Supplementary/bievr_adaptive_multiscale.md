# BIEVR 退化感知多尺度

纯 LIO 实验入口沿用 Gamma 的 LiDAR、IMU 标定和 1.0 m 三维体素。5 cm 是实验基准，现有 Gamma 配置使用 7 cm。

```bash
roslaunch fast_livo mapping_gamma_adaptive_multiscale.launch mode:=degeneracy lio_quality_log:=true
rosbag play ~/data/GEODE/sensor_data/metro_tunnel/Shield_tunnel4_gamma.bag --topics /livox/lidar /livox/imu
```

| `mode` | 高度图及选尺度策略 |
| --- | --- |
| `base` | 固定 5 cm，可用 `base_pixel_size` 覆盖 |
| `fine` | 固定 2.5 cm 对照 |
| `coarse` | 固定 10 cm 对照 |
| `density` | 5/10 cm，按对应支持率选择，通过残差质量门控 |
| `degeneracy` | 5/10 cm，按弱子空间几何信息和对应质量选择 |

原有入口默认关闭自适应。该功能的 ROS 参数在 `lio/bievr/adaptive_multiscale` 下，实验入口加载独立参数文件。 [实验参数](../config/bievr/adaptive_multiscale.yaml)

## 尺度选择

- 在 IMU 预测位姿处以 Base 层配准，用带 Huber 权重的 LiDAR 几何行累积 6×6 信息矩阵。用 5 m 参考长度归一化旋转扰动，点数充足时按最大特征值的 2% 划定弱子空间。
- 按体素分组，优先试算 Base 支持偏低的区域。用已观测像素的未平滑高度和有效权重聚合粗层，保留支持格数与像素间高度方差。粗层与 Base 共用当前体素坐标系，法向变化由现有 Base 重投影处理，后续试算从重投影后的数据生成。
- 粗层至少保留 70% 的 Base 对应，公共对应的残差需满足质量门控。退化模式还要求粗层的弱子空间投影信息达到切换条件。每体素每扫描使用一个尺度。质量或求解失败时从原预测位姿重跑 Base。
- 默认每扫描最多试算 128 个体素、保留 64 个粗层，粗层临时数据随该次配准释放。只有固定 `fine` 对照使用全图 2.5 cm。

## 诊断

`multiscale_log:=true` 写出 `Log/bievr_multiscale_<seq_name>.csv`，含归一化谱、各阶段有效匹配数、Base 弱子空间下的投影信息、粗层使用与缓存、退回 Base 状态。稀疏体素指初始线性化时 Base 对应有效率低于 60% 的体素。投影信息和稀疏体素支持率都比较同一帧、同一个预测位姿的尺度候选。

`lio_quality_log:=true` 另写出原有逐帧质量日志。对比时使用相同输入和共同的真值时间窗口，先检查扫描签名与 IMU 样本数，再评估位姿误差。

## GEODE 回放结果

Shield4 同输入的固定 5 cm 与自适应 5/10 cm 对照，在 −1.20 s 共同评估时移下，ATE RMSE 为 0.327→0.237 m，掉头轴向 RMSE 为 0.685→0.430 m。自适应组的稀疏体素匹配率为 16.45→17.63%，单扫描粗层缓存峰值约 0.29 MiB，LIO 建图耗时 P50 / P95 为 6.13 / 10.87→8.49 / 10.94 ms。以现有 Gamma 的 7 cm 为基准时，自适应 7/10 cm 的 ATE 为 0.213→0.191 m。

Shield5 以各轨迹独立拟合时移得到的最小 ATE RMSE 为主结果。各组在搜索时使用相同的 676 个真值点。搜索范围为 −3.5 至 +0.5 s，步长 1 ms，最小值附近细化至 0.01 ms。对每个候选时移执行无尺度 SE(3) 对齐，时移只加到评估时读取的估计位姿时间戳。

| Shield5 纯 LIO | 最优时移 | 最小 ATE RMSE |
| --- | ---: | ---: |
| 固定 5 cm | −2.13927 s | 0.114162 m |
| 按支持率选择 5/10 cm | −2.17863 s | 0.116854 m |
| 按退化方向选择 5/10 cm | −2.17160 s | 0.114707 m |

按退化方向选尺度使稀疏体素匹配率从 26.12% 升到 30.82%，最小 ATE 比固定 5 cm 高约 0.55 mm。原固定 −1.90 s 对照为 0.245→0.272 m，保留作时移敏感性对照。

关闭自适应后，受控回放中输出的 3,402 行 Shield4 轨迹与修改前的 7 cm 版本逐字节一致。验证覆盖粗层聚合、稀疏匹配和法向重投影，以及弱子空间、缓存预算和尺度选择。
