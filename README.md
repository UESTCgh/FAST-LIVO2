# FAST-LIVO2

## FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

### 📢 News

- 🔓 **2025-01-23**: Code released!  
- 🎉 **2024-10-01**: Accepted by **T-RO '24**!  
- 🚀 **2024-07-02**: Conditionally accepted.

### 📬 Contact

For further inquiries or assistance, please contact [zhengcr@connect.hku.hk](mailto:zhengcr@connect.hku.hk).

## 1. Introduction

FAST-LIVO2 is an efficient and accurate LiDAR-inertial-visual fusion localization and mapping system, demonstrating significant potential for real-time 3D reconstruction and onboard robotic localization in severely degraded environments.

**Developer**: [Chunran Zheng 郑纯然](https://github.com/xuankuzcr)

<div align="center">
    <img src="pics/Framework.png" width = 100% >
</div>

### 1.1 Related video

Our accompanying video is now available on [**Bilibili**](https://www.bilibili.com/video/BV1Ezxge7EEi) and [**YouTube**](https://youtu.be/6dF2DzgbtlY).

### 1.2 Related paper

[FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2408.14035)  

[FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/pdf/2501.13876)  

[FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2203.00893)

[FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second](https://www.arxiv.org/pdf/2507.17210)

### 1.3 Our hard-synchronized equipment

We open-source our handheld device, including CAD files, synchronization scheme, STM32 source code, wiring instructions, and sensor ROS driver. Access these resources at this repository: [**LIV_handhold**](https://github.com/xuankuzcr/LIV_handhold).

### 1.4 Our associate dataset: FAST-LIVO2-Dataset
Our associate dataset [**FAST-LIVO2-Dataset**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z) used for evaluation is also available online.

### 1.5 Our LiDAR-camera calibration method
The [**FAST-Calib**](https://github.com/hku-mars/FAST-Calib) toolkit is recommended. Its output extrinsic parameters can be directly filled into the YAML file. 

## 2. Prerequisited

### 2.1 Ubuntu and ROS

Ubuntu 18.04~20.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 2.2 PCL && Eigen && OpenCV

PCL>=1.8, Follow [PCL Installation](https://pointclouds.org/). 

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

OpenCV>=4.2, Follow [Opencv Installation](http://opencv.org/).

### 2.3 Sophus

Sophus Installation for the non-templated/double-only version.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

### 2.4 Vikit

Vikit contains camera models, some math and interpolation functions that we need. Vikit is a catkin project, therefore, download it into your catkin workspace source folder.

```bash
# Different from the one used in fast-livo1
cd catkin_ws/src
git clone https://github.com/xuankuzcr/rpg_vikit.git 
```

## 3. Build

Clone the repository and catkin_make:

```
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST-LIVO2
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 4. Run our examples

Download our collected rosbag files via OneDrive ([**FAST-LIVO2-Dataset**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z)). 

```
roslaunch fast_livo mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
```

### 4.1 GEODE Gamma tunnels

The Gamma launch enables the left camera and runs BIEVR and visual updates on the same state. Use `vio:=false` to run LiDAR–IMU odometry with the BIEVR residual and play the Gamma tunnel bag in a second terminal.

```bash
roslaunch fast_livo mapping_gamma.launch rviz:=false vio:=false seq_name:=Shield_tunnel4_gamma_bievr
rosbag play ~/data/GEODE/Shield_tunnel4_gamma.bag --topics /livox/lidar /livox/imu
```

For the original point-to-plane residual, add `lio_bievr:=false` and use a distinct `seq_name`. FAST-LIVO2 writes TUM trajectories to `Log/result/<seq_name>.txt`.

With the left camera enabled and `save_map:=true`, the node saves its visual landmarks and colorized point cloud on clean shutdown. `map_dir` selects the output directory.

```bash
roslaunch fast_livo mapping_gamma.launch rviz:=false vio:=true save_map:=true \
  seq_name:=Shield_tunnel4_gamma_bievr_vio map_dir:="$HOME/shield4_visual_map"
rosbag play ~/data/GEODE/Shield_tunnel4_gamma.bag --topics /livox/lidar /livox/imu /left_camera/image/compressed
```

`visual_landmarks.pcd` contains sparse visual landmarks with normals. `all_downsampled_points.pcd` contains the downsampled XYZRGB cloud, and `all_raw_points.pcd` contains the full colorized cloud. The pose file contains the final pose at each processed camera timestamp. When the visual update fails the quality check, the pose remains at the corresponding LiDAR–IMU estimate.

For comparison with the standalone BIEVR-LIO backend, build the BIEVR-LIO ROS1 package with Livox CustomMsg support and load its workspace before FAST-LIVO2.

```bash
source ~/catkin_ws/devel/setup.bash
source ~/fastlivo2_ws/devel/setup.bash --extend
roslaunch fast_livo mapping_gamma.launch bievr:=true
```

In a second terminal, play the bag with `rosbag play BAG --topics /livox/lidar /livox/imu`. RViz shows the accumulated cloud and odometry in the `odom` frame. This backend publishes `/bievr_lio/points/registered` and `/bievr_lio/odom`.

For offline processing, pass the absolute bag path with `bievr:=true bag:=/path/to/BAG rviz:=false`. Use `bievr_params:=/path/to/params.yaml` to set BIEVR-LIO options such as `debug.trajectory_path`.

## 5. License

The source code of this package is released under the [**GPLv2**](http://www.gnu.org/licenses/) license. For commercial use, please contact me at <zhengcr@connect.hku.hk> and Prof. Fu Zhang at <fuzhang@hku.hk> to discuss an alternative license.
