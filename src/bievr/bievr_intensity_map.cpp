// Adapted from the COIN-BIEVR reproduction, c560f3be7feebf5636fe4b2def021b77e26db52a.
// Copyright (c) 2026, Autonomous Systems Lab, ETH Zurich
// SPDX-License-Identifier: BSD-3-Clause
// See bievr_intensity_map.h for the redistribution conditions and disclaimer.

#include "bievr_intensity_map.h"
#include "bievr_voxel_map.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace fast_livo::bievr
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinValidRangeSquared = 1e-6;
using Image = Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using SumImage = Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

int wrapColumn(int column, int width)
{
  const int wrapped = column % width;
  return wrapped < 0 ? wrapped + width : wrapped;
}

Image computeBrightness(const Image &image, const Image &count, int window_width, int window_height)
{
  const int width = image.cols();
  const int height = image.rows();
  const int half_width = std::min(window_width / 2, (width - 1) / 2);
  const int half_height = std::min(window_height / 2, height - 1);
  const int padded_width = width + 2 * half_width;

  SumImage horizontal_sums(height, width), horizontal_counts(height, width);
  tbb::parallel_for(tbb::blocked_range<int>(0, height), [&](const tbb::blocked_range<int> &rows) {
    std::vector<double> sums(padded_width + 1, 0.0), counts(padded_width + 1, 0.0);
    for (int row = rows.begin(); row < rows.end(); ++row)
    {
      for (int index = 0; index < padded_width; ++index)
      {
        const int column = wrapColumn(index - half_width, width);
        const bool valid = count(row, column) > 0.0F;
        sums[index + 1] = sums[index] + (valid ? image(row, column) : 0.0F);
        counts[index + 1] = counts[index] + static_cast<double>(valid);
      }
      for (int column = 0; column < width; ++column)
      {
        const int end = column + 2 * half_width + 1;
        horizontal_sums(row, column) = sums[end] - sums[column];
        horizontal_counts(row, column) = counts[end] - counts[column];
      }
    }
  });

  // Vertical windows truncate at the bounds. Prefix rows are accumulated in order.
  SumImage sums = SumImage::Zero(height + 1, width), counts = SumImage::Zero(height + 1, width);
  for (int row = 0; row < height; ++row)
  {
    sums.row(row + 1) = sums.row(row) + horizontal_sums.row(row);
    counts.row(row + 1) = counts.row(row) + horizontal_counts.row(row);
  }
  Image brightness(height, width);
  tbb::parallel_for(tbb::blocked_range<int>(0, height), [&](const tbb::blocked_range<int> &rows) {
    for (int row = rows.begin(); row < rows.end(); ++row)
    {
      const int begin = std::max(0, row - half_height);
      const int end = std::min(height, row + half_height + 1);
      for (int column = 0; column < width; ++column)
      {
        const double count = counts(end, column) - counts(begin, column);
        brightness(row, column) = count > 0.0 ? (sums(end, column) - sums(begin, column)) / count : 0.0;
      }
    }
  });
  return brightness;
}
} // namespace

void normalizeIntensities(const PointCloudXYZI &scan, const BievrIntensityParameters &parameters,
                          std::vector<float> &intensities)
{
  const int width = parameters.image_width;
  const int height = parameters.image_height;
  const double fov_span = parameters.fov_up_deg - parameters.fov_down_deg;
  if (width <= 0 || height <= 0 || static_cast<long long>(width) * height > std::numeric_limits<int>::max() / 2 ||
      parameters.window_width <= 0 || parameters.window_height <= 0 ||
      !(parameters.fov_up_deg <= 90.0 && parameters.fov_down_deg >= -90.0 && fov_span > 0.0) ||
      !(parameters.scale > 0.0 && parameters.brightness_scale > 0.0) ||
      !(parameters.min_range >= 0.0 && parameters.max_range > parameters.min_range) ||
      !std::isfinite(parameters.scale + parameters.brightness_scale + parameters.max_range))
    throw std::invalid_argument("Invalid BIEVR intensity projection parameters");

  intensities.assign(scan.size(), kInvalidIntensity);
  if (scan.empty()) return;

  const double fov_up = parameters.fov_up_deg * kPi / 180.0;
  const double inverse_fov = 180.0 / (kPi * fov_span);
  std::vector<int> pixels(scan.size(), -1);
  tbb::parallel_for(tbb::blocked_range<std::size_t>(0, scan.size()), [&](const auto &range) {
    for (std::size_t index = range.begin(); index < range.end(); ++index)
    {
      const PointType &sample = scan[index];
      const V3D point = sample.getVector3fMap().cast<double>();
      const double squared_range = point.squaredNorm();
      if (!std::isfinite(squared_range) || squared_range <= kMinValidRangeSquared ||
          !std::isfinite(sample.intensity) || sample.intensity < 0.0F) continue;

      const double azimuth = std::atan2(point.y(), point.x());
      const double elevation = std::asin(std::clamp(point.z() / std::sqrt(squared_range), -1.0, 1.0));
      const double projected_row = (fov_up - elevation) * inverse_fov * height;
      if (!(projected_row >= 0.0 && projected_row < height)) continue;
      const int row = static_cast<int>(projected_row);
      const int column = wrapColumn(static_cast<int>(std::floor(0.5 * width - width * azimuth / (2.0 * kPi))), width);
      pixels[index] = row * width + column;
    }
  });

  Image image = Image::Zero(height, width), count = Image::Zero(height, width);
  for (std::size_t index = 0; index < scan.size(); ++index)
  {
    const int pixel = pixels[index];
    if (pixel < 0) continue;
    image.data()[pixel] += static_cast<float>(parameters.scale) * scan[index].intensity;
    count.data()[pixel] += 1.0F;
  }
  image = (count > 0.0F).select(image / count, 0.0F);
  const Image brightness = computeBrightness(image, count, parameters.window_width, parameters.window_height);
  const double min_squared_range = parameters.min_range * parameters.min_range;
  const double max_squared_range = parameters.max_range * parameters.max_range;

  tbb::parallel_for(tbb::blocked_range<std::size_t>(0, scan.size()), [&](const auto &range) {
    for (std::size_t index = range.begin(); index < range.end(); ++index)
    {
      const int pixel = pixels[index];
      if (pixel < 0) continue;
      const PointType &sample = scan[index];
      const double squared_range = sample.getVector3fMap().cast<double>().squaredNorm();
      if (squared_range < min_squared_range || squared_range > max_squared_range) continue;
      const double value = parameters.scale * sample.intensity;
      const double normalized = parameters.brightness_scale * value / (brightness.data()[pixel] + 1.0);
      if (std::isfinite(normalized)) intensities[index] = static_cast<float>(std::min(255.0, normalized));
    }
  });
}

void selectIntensityPoints(const BievrVoxelMap &map, const PointCloudXYZI &fine_points,
                           const std::vector<float> &fine_intensities, const M3D &world_from_lidar_rotation,
                           const V3D &world_from_lidar_translation, const BievrIntensityParameters &parameters,
                           std::vector<std::size_t> &selected_indices)
{
  selected_indices.clear();
  if (fine_points.size() != fine_intensities.size())
    throw std::invalid_argument("BIEVR intensity count must equal point count");
  if (!(parameters.degeneracy_ratio > 0.0) || !std::isfinite(parameters.degeneracy_ratio))
    throw std::invalid_argument("Invalid BIEVR intensity degeneracy ratio");
  if (fine_points.empty() || parameters.informed_voxels == 0) return;

  struct PointVoxel
  {
    BievrVoxelKey key;
    std::size_t index;
  };
  std::vector<PointVoxel> entries;
  entries.reserve(fine_points.size());
  for (std::size_t index = 0; index < fine_points.size(); ++index)
  {
    const V3D point = world_from_lidar_rotation * fine_points[index].getVector3fMap().cast<double>() + world_from_lidar_translation;
    if (!map.canIndex(point)) continue;
    const auto key = map.pointToKey(point);
    entries.push_back({key, index});
  }
  tbb::parallel_sort(entries.begin(), entries.end(), [](const PointVoxel &left, const PointVoxel &right) {
    return std::tie(left.key.x_, left.key.y_, left.key.z_, left.index) < std::tie(right.key.x_, right.key.y_, right.key.z_, right.index);
  });

  struct ObservedVoxel
  {
    const BievrVoxel *voxel;
    std::size_t begin;
    std::size_t end;
    double contribution = 0.0;
  };
  std::vector<ObservedVoxel> observed;
  M3D normals = M3D::Zero();
  for (std::size_t begin = 0; begin < entries.size();)
  {
    std::size_t end = begin + 1;
    while (end < entries.size() && entries[end].key == entries[begin].key) ++end;
    if (const BievrVoxel *voxel = map.findVoxel(entries[begin].key))
    {
      observed.push_back({voxel, begin, end});
      const V3D normal = voxel->cell_from_world_.linear().row(2).transpose();
      normals.noalias() += normal * normal.transpose();
    }
    begin = end;
  }
  if (observed.empty()) return;
  const Eigen::SelfAdjointEigenSolver<M3D> solver(normals);
  if (solver.info() != Eigen::Success) return;
  const int target_directions = parameters.degeneracy_ratio * solver.eigenvalues()[0] > solver.eigenvalues()[1] ? 2 : 1;
  for (ObservedVoxel &entry : observed)
  {
    if (!entry.voxel->intensity_) continue;
    for (int direction = 0; direction < target_directions; ++direction)
    {
      const V3D local_direction = entry.voxel->cell_from_world_.linear() * solver.eigenvectors().col(direction);
      entry.contribution += local_direction.head<2>().cwiseAbs().dot(entry.voxel->intensity_->gradient_strength);
    }
  }

  const std::size_t limit = std::min(parameters.informed_voxels, observed.size());
  std::partial_sort(observed.begin(), observed.begin() + limit, observed.end(),
    [](const ObservedVoxel &left, const ObservedVoxel &right) {
      if (left.contribution != right.contribution) return left.contribution > right.contribution;
      return left.begin < right.begin;
    });
  for (std::size_t i = 0; i < limit && observed[i].contribution > 0.0; ++i)
  {
    for (std::size_t j = observed[i].begin; j < observed[i].end; ++j)
    {
      const std::size_t index = entries[j].index;
      if (std::isfinite(fine_intensities[index]) && fine_intensities[index] >= 0.0F) selected_indices.push_back(index);
    }
  }
}
} // namespace fast_livo::bievr
