#pragma once

#include "bievr_adaptive_config.h"

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace fast_livo::bievr
{
class BievrVoxelMap;
struct BievrMeasurement;

BievrAdaptiveDiagnostics analyzeBievrGeometry(const Eigen::Matrix<double, 6, 6> &information,
                                              std::size_t candidates, std::size_t matches,
                                              const BievrAdaptiveParams &params);

class BievrAdaptivePolicy
{
public:
  explicit BievrAdaptivePolicy(const BievrAdaptiveParams &params);
  ~BievrAdaptivePolicy();

  void prepare(const BievrVoxelMap &map, const std::vector<Eigen::Vector3d> &imu_points,
               const Eigen::Matrix3d &rotation, const Eigen::Vector3d &position, double huber_delta);

  // The prepared cache and voxel map are read-only during the iterated solve.
  bool sample(const BievrVoxelMap &map, const Eigen::Vector3d &point, BievrMeasurement &measurement) const;
  const BievrAdaptiveDiagnostics &diagnostics() const;

  void useBase();
  void finishFrame(bool accepted);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace fast_livo::bievr
