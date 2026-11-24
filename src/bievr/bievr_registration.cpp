#include "bievr_registration.h"

#include "bievr_adaptive.h"
#include "bievr_intensity_map.h"
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

bievr::BievrVoxelMap::Parameters mapParameters(double voxel_size, double pixel_size, int max_voxels,
                                             bool intensity_enabled)
{
  if (!validSize(voxel_size) || !validSize(pixel_size) || max_voxels <= 0 || std::sqrt(3.0) * voxel_size / pixel_size > max_voxel_index)
    throw std::invalid_argument("Invalid BIEVR voxel size, pixel size, or map capacity");

  bievr::BievrVoxelMap::Parameters parameters;
  parameters.voxel_size_ = voxel_size;
  parameters.pixel_size_ = pixel_size;
  parameters.maximum_voxel_count_ = static_cast<std::size_t>(max_voxels);
  parameters.intensity_enabled_ = intensity_enabled;
  return parameters;
}
} // namespace

struct BievrRegistration::Impl
{
  Impl(double voxel_size, double pixel_size, int max_voxels, double fine_voxel_size, int informed_voxels,
       bool use_intensity, bool use_intensity_sampling, const bievr::BievrAdaptiveParams& adaptive_parameters)
      : map(mapParameters(voxel_size, pixel_size, max_voxels, use_intensity)),
        inverse_voxel_size(1.0 / voxel_size), intensity_enabled(use_intensity),
        intensity_sampling(use_intensity_sampling)
  {
    if (!validSize(fine_voxel_size) || informed_voxels < 0)
      throw std::invalid_argument("Invalid BIEVR scan sampling parameters");
    if (adaptive_parameters.enabled)
    {
      if (!(adaptive_parameters.coarse_pixel_size_m > pixel_size))
        throw std::invalid_argument("BIEVR adaptive coarse pixels must be larger than base pixels");
      adaptive = std::make_unique<bievr::BievrAdaptivePolicy>(adaptive_parameters);
    }

    sampling.fine_voxel_size_ = fine_voxel_size;
    sampling.informed_voxel_count_ = static_cast<std::size_t>(informed_voxels);
  }

  bievr::BievrVoxelMap map;
  std::unique_ptr<bievr::BievrAdaptivePolicy> adaptive;
  const double inverse_voxel_size;
  const bool intensity_enabled;
  const bool intensity_sampling;
  const bievr::BievrIntensityParameters intensity_parameters;
  bievr::BievrVoxelMap::SamplingParameters sampling;
  tbb::task_arena arena{4};
  PointCloudXYZI world_scan;
  PointCloudXYZI finite_scan;
  PointCloudXYZI fine_points;
  PointCloudXYZI selected_points;
  std::vector<double> ranges;
  std::vector<V3D> imu_points;
  std::vector<V3D> intensity_points;
  std::vector<float> intensity_values;
};

BievrRegistration::BievrRegistration(double voxel_size, double pixel_size, int max_voxels, double fine_voxel_size,
                                     int informed_voxels, bool intensity_enabled, bool intensity_sampling,
                                     const bievr::BievrAdaptiveParams& adaptive)
    : impl_(std::make_unique<Impl>(voxel_size, pixel_size, max_voxels, fine_voxel_size, informed_voxels,
                                  intensity_enabled, intensity_sampling, adaptive))
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
  std::vector<float> intensities;
  if (impl.intensity_enabled) bievr::normalizeIntensities(scan, impl.intensity_parameters, intensities);

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
        if (impl.intensity_enabled) intensities[point_count] = intensities[index];
      }
      ++point_count;
    }
    impl.world_scan.resize(point_count);
    impl.ranges.resize(point_count);
    if (impl.intensity_enabled) intensities.resize(point_count);
    impl.map.update(impl.world_scan, &impl.ranges, impl.intensity_enabled ? &intensities : nullptr);
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
  std::vector<float> intensities;
  std::vector<float> finite_intensities;
  if (impl.intensity_enabled)
  {
    bievr::normalizeIntensities(scan, impl.intensity_parameters, intensities);
    finite_intensities.reserve(scan.size());
  }
  std::size_t input_index = 0;
  for (const PointType& point : scan.points)
  {
    const V3D point_lidar = point.getVector3fMap().cast<double>();
    if (canIndexPoint(point_lidar, inverse_fine_voxel_size) &&
        canIndexPoint(lidar_to_world_rotation * point_lidar + lidar_to_world_translation, impl.inverse_voxel_size))
    {
      impl.finite_scan.push_back(point);
      if (impl.intensity_enabled) finite_intensities.push_back(intensities[input_index]);
    }
    ++input_index;
  }

  std::vector<std::size_t> fine_indices;
  std::vector<std::size_t> geometric_indices;
  impl.arena.execute([&] {
    bievr::BievrVoxelMap::downsampleNearestToVoxelCenter(impl.finite_scan, impl.sampling.fine_voxel_size_, impl.fine_points,
                                                       impl.intensity_enabled ? &fine_indices : nullptr);
    impl.map.selectMeasurementPoints(impl.fine_points, Eigen::Quaterniond(world_rotation), world_position,
                                     ext_rotation, ext_translation, impl.sampling, impl.selected_points,
                                     impl.intensity_enabled && !impl.intensity_sampling ? &geometric_indices : nullptr);
  });

  impl.imu_points.resize(impl.selected_points.size());
  for (std::size_t index = 0; index < impl.selected_points.size(); ++index)
    impl.imu_points[index] = ext_rotation * impl.selected_points[index].getVector3fMap().cast<double>() + ext_translation;

  if (!impl.intensity_enabled) return;

  std::vector<float> fine_intensities;
  fine_intensities.reserve(fine_indices.size());
  for (const auto index : fine_indices) fine_intensities.push_back(finite_intensities[index]);
  std::vector<std::size_t> informed_indices;
  if (impl.intensity_sampling)
  {
    impl.arena.execute([&] {
      bievr::selectIntensityPoints(impl.map, impl.fine_points, fine_intensities, lidar_to_world_rotation,
                                   lidar_to_world_translation, impl.intensity_parameters, informed_indices);
    });
  }
  const auto &intensity_indices = impl.intensity_sampling ? informed_indices : geometric_indices;
  impl.intensity_points.clear();
  impl.intensity_values.clear();
  impl.intensity_points.reserve(intensity_indices.size());
  impl.intensity_values.reserve(intensity_indices.size());
  for (const auto index : intensity_indices)
  {
    if (!(fine_intensities[index] >= 0.0f)) continue;
    impl.intensity_points.push_back(ext_rotation * impl.fine_points[index].getVector3fMap().cast<double>() + ext_translation);
    impl.intensity_values.push_back(fine_intensities[index]);
  }
}

const std::vector<V3D>& BievrRegistration::imuPoints() const
{
  return impl_->imu_points;
}

const std::vector<V3D>& BievrRegistration::intensityPoints() const
{
  return impl_->intensity_points;
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

bool BievrRegistration::sampleIntensity(std::size_t index, const V3D& world_point, V3D& gradient, double& residual) const
{
  if (!canIndexPoint(world_point, impl_->inverse_voxel_size)) return false;
  bievr::BievrMeasurement measurement;
  if (!impl_->map.sampleIntensity(world_point, impl_->intensity_values[index], measurement)) return false;
  gradient = measurement.position_jacobian_.transpose();
  residual = measurement.residual_;
  return gradient.allFinite() && std::isfinite(residual);
}

bool BievrRegistration::sampleAdaptive(const V3D& world_point, V3D& gradient, double& residual) const
{
  bievr::BievrMeasurement measurement;
  if (!impl_->adaptive->sample(impl_->map, world_point, measurement)) return false;
  gradient = measurement.position_jacobian_.transpose();
  residual = measurement.residual_;
  return gradient.allFinite() && std::isfinite(residual);
}

void BievrRegistration::prepareAdaptive(const M3D& world_rotation, const V3D& world_position, double huber_delta)
{
  impl_->arena.execute([&] {
    impl_->adaptive->prepare(impl_->map, impl_->imu_points, world_rotation, world_position, huber_delta);
  });
}

const bievr::BievrAdaptiveDiagnostics& BievrRegistration::adaptiveDiagnostics() const
{
  return impl_->adaptive->diagnostics();
}

void BievrRegistration::discardAdaptive()
{
  impl_->adaptive->useBase();
}

void BievrRegistration::finishAdaptive(bool accepted)
{
  impl_->adaptive->finishFrame(accepted);
}
} // namespace fast_livo
