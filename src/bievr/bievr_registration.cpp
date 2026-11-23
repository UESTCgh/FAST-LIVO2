#include "bievr_registration.h"

#include "bievr_voxel_map.h"

#include <tbb/task_arena.h>

namespace fast_livo
{
namespace
{
constexpr double max_voxel_index = std::numeric_limits<std::int32_t>::max() - 2.0;

bool validSize(double size)
{
  return std::isfinite(size) && size > 0.0 && std::isfinite(1.0 / size);
}

bool canIndexPoint(const V3D& point, double inverse_voxel_size)
{
  return point.allFinite() && point.cwiseAbs().maxCoeff() * inverse_voxel_size <= max_voxel_index;
}

bievr::BievrVoxelMap::Parameters mapParameters(double voxel_size, double pixel_size, int max_voxels)
{
  if (!validSize(voxel_size) || !validSize(pixel_size) || max_voxels <= 0 || std::sqrt(3.0) * voxel_size / pixel_size > max_voxel_index)
    throw std::invalid_argument("Invalid BIEVR voxel size, pixel size, or map capacity");

  bievr::BievrVoxelMap::Parameters parameters;
  parameters.voxel_size_ = voxel_size;
  parameters.pixel_size_ = pixel_size;
  parameters.maximum_voxel_count_ = static_cast<std::size_t>(max_voxels);
  return parameters;
}
} // namespace

struct BievrRegistration::Impl
{
  Impl(double voxel_size, double pixel_size, int max_voxels, double fine_voxel_size, int informed_voxels)
      : map(mapParameters(voxel_size, pixel_size, max_voxels)), inverse_voxel_size(1.0 / voxel_size)
  {
    if (!validSize(fine_voxel_size) || informed_voxels < 0)
      throw std::invalid_argument("Invalid BIEVR scan sampling parameters");

    sampling.fine_voxel_size_ = fine_voxel_size;
    sampling.informed_voxel_count_ = static_cast<std::size_t>(informed_voxels);
  }

  bievr::BievrVoxelMap map;
  const double inverse_voxel_size;
  bievr::BievrVoxelMap::SamplingParameters sampling;
  tbb::task_arena arena{4};
  PointCloudXYZI world_scan;
  PointCloudXYZI finite_scan;
  PointCloudXYZI fine_points;
  PointCloudXYZI selected_points;
  std::vector<double> ranges;
  std::vector<V3D> imu_points;
};

BievrRegistration::BievrRegistration(double voxel_size, double pixel_size, int max_voxels, double fine_voxel_size, int informed_voxels)
    : impl_(std::make_unique<Impl>(voxel_size, pixel_size, max_voxels, fine_voxel_size, informed_voxels))
{
}

BievrRegistration::~BievrRegistration() = default;

void BievrRegistration::updateMap(const PointCloudXYZI& scan, const M3D& world_rotation, const V3D& world_position,
                                  const M3D& ext_rotation, const V3D& ext_translation)
{
  Impl& impl = *impl_;
  const M3D lidar_to_world_rotation = world_rotation * ext_rotation;
  const V3D lidar_to_world_translation = world_rotation * ext_translation + world_position;
  impl.world_scan.resize(scan.size());
  impl.ranges.resize(scan.size());

  impl.arena.execute([&] {
    tbb::parallel_for(tbb::blocked_range<std::size_t>(0, scan.size()), [&](const tbb::blocked_range<std::size_t>& indices) {
      for (std::size_t index = indices.begin(); index < indices.end(); ++index)
      {
        const V3D point_lidar = scan[index].getVector3fMap().cast<double>();
        const V3D point_world = lidar_to_world_rotation * point_lidar + lidar_to_world_translation;
        if (!canIndexPoint(point_world, impl.inverse_voxel_size) || point_world.cwiseAbs().maxCoeff() > std::numeric_limits<float>::max())
        {
          impl.ranges[index] = -1.0;
          continue;
        }

        const V3F point_world_float = point_world.cast<float>();
        if (!canIndexPoint(point_world_float.cast<double>(), impl.inverse_voxel_size))
        {
          impl.ranges[index] = -1.0;
          continue;
        }

        impl.world_scan[index].getVector3fMap() = point_world_float;
        impl.ranges[index] = point_lidar.norm();
      }
    });

    std::size_t point_count = 0;
    for (std::size_t index = 0; index < scan.size(); ++index)
    {
      if (impl.ranges[index] < 0.0)
        continue;
      if (point_count != index)
      {
        impl.world_scan[point_count] = impl.world_scan[index];
        impl.ranges[point_count] = impl.ranges[index];
      }
      ++point_count;
    }
    impl.world_scan.resize(point_count);
    impl.ranges.resize(point_count);
    impl.map.update(impl.world_scan, &impl.ranges);
  });
}

void BievrRegistration::selectPoints(const PointCloudXYZI& scan, const M3D& world_rotation, const V3D& world_position,
                                     const M3D& ext_rotation, const V3D& ext_translation)
{
  Impl& impl = *impl_;
  const M3D lidar_to_world_rotation = world_rotation * ext_rotation;
  const V3D lidar_to_world_translation = world_rotation * ext_translation + world_position;
  const double inverse_fine_voxel_size = 1.0 / impl.sampling.fine_voxel_size_;
  impl.finite_scan.clear();
  impl.finite_scan.reserve(scan.size());
  for (const PointType& point : scan.points)
  {
    const V3D point_lidar = point.getVector3fMap().cast<double>();
    if (canIndexPoint(point_lidar, inverse_fine_voxel_size) &&
        canIndexPoint(lidar_to_world_rotation * point_lidar + lidar_to_world_translation, impl.inverse_voxel_size))
      impl.finite_scan.push_back(point);
  }

  impl.arena.execute([&] {
    bievr::BievrVoxelMap::downsampleNearestToVoxelCenter(impl.finite_scan, impl.sampling.fine_voxel_size_, impl.fine_points);
    impl.map.selectMeasurementPoints(impl.fine_points, Eigen::Quaterniond(world_rotation), world_position,
                                     ext_rotation, ext_translation, impl.sampling, impl.selected_points);
  });

  impl.imu_points.resize(impl.selected_points.size());
  for (std::size_t index = 0; index < impl.selected_points.size(); ++index)
    impl.imu_points[index] = ext_rotation * impl.selected_points[index].getVector3fMap().cast<double>() + ext_translation;
}

const std::vector<V3D>& BievrRegistration::imuPoints() const
{
  return impl_->imu_points;
}

bool BievrRegistration::sample(const V3D& world_point, V3D& gradient, double& residual) const
{
  if (!canIndexPoint(world_point, impl_->inverse_voxel_size))
    return false;

  bievr::BievrMeasurement measurement;
  if (!impl_->map.sample(world_point, measurement))
    return false;

  gradient = measurement.position_jacobian_.transpose();
  residual = measurement.residual_;
  return gradient.allFinite() && std::isfinite(residual);
}
} // namespace fast_livo
