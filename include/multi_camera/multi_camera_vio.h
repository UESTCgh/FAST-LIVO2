#pragma once

#include "voxel_map.h"

#include <memory>

namespace vk { class AbstractCamera; }

namespace fast_livo
{
class MultiCameraVio
{
public:
  using CameraLoader = bool (*)(const std::string &, vk::AbstractCamera *&);

  MultiCameraVio(ros::NodeHandle &nh, const M3D &lidar_to_imu_rotation, const V3D &lidar_to_imu_translation,
                 CameraLoader load_camera);
  ~MultiCameraVio();

  void subscribe(ros::NodeHandle &nh);
  bool takeObservations(double previous_scan_end, double scan_end, double imu_watermark);
  void update(double scan_end, const V3D &angular_velocity, const LioQuality &lidar_quality,
              std::vector<pointWithVar> &points, const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &planes,
              StatesGroup &state);
  void colorize(const PointCloudXYZI &world_cloud, double min_range, PointCloudXYZRGB &output) const;
  void appendLandmarks(pcl::PointCloud<pcl::PointNormal> &output) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace fast_livo
