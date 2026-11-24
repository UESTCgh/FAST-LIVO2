#pragma once

#include "utils/types.h"

#include <memory>
#include <vector>

namespace fast_livo
{
class BievrRegistration
{
public:
  BievrRegistration(double voxel_size, double pixel_size, int max_voxels, double fine_voxel_size,
                    int informed_voxels, bool intensity_enabled, bool intensity_sampling);
  ~BievrRegistration();

  void updateMap(const PointCloudXYZI& scan, const M3D& world_rotation, const V3D& world_position,
                 const M3D& ext_rotation, const V3D& ext_translation);
  void selectPoints(const PointCloudXYZI& scan, const M3D& world_rotation, const V3D& world_position,
                    const M3D& ext_rotation, const V3D& ext_translation);

  const std::vector<V3D>& imuPoints() const;
  const std::vector<V3D>& intensityPoints() const;

  // Read-only during the iterated update. The caller integrates scans between updates.
  bool sample(const V3D& world_point, V3D& gradient, double& residual) const;
  bool sampleIntensity(std::size_t index, const V3D& world_point, V3D& gradient, double& residual) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace fast_livo
