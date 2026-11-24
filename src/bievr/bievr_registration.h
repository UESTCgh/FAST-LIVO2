#pragma once

#include "bievr_adaptive_config.h"
#include "utils/types.h"

#include <memory>
#include <vector>

namespace fast_livo
{
class BievrRegistration
{
public:
  BievrRegistration(double voxel_size, double pixel_size, int max_voxels, double fine_voxel_size,
                    int informed_voxels, bool intensity_enabled, bool intensity_sampling,
                    const bievr::BievrAdaptiveParams& adaptive);
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
  bool sampleAdaptive(const V3D& world_point, V3D& gradient, double& residual) const;
  void prepareAdaptive(const M3D& world_rotation, const V3D& world_position, double huber_delta);
  const bievr::BievrAdaptiveDiagnostics& adaptiveDiagnostics() const;
  void discardAdaptive();
  void finishAdaptive(bool accepted);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace fast_livo
