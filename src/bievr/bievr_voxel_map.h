// BIEVR voxel-image mapping and sampling are derived from BIEVR-LIO:
// https://github.com/ethz-asl/BIEVR-LIO
//
// BSD 3-Clause License
//
// Copyright (c) 2026, Autonomous Systems Lab, ETH Zurich
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// Adapted from BIEVR-LIO-ESIKF (23543322879c19a12f6151499a0074e4cd539d0f).
// Intensity images and scoring are adapted from COIN-BIEVR (c560f3be7feebf5636fe4b2def021b77e26db52a).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <list>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

namespace fast_livo::bievr
{
struct BievrVoxelKey
{
    std::int32_t x_ = 0;
    std::int32_t y_ = 0;
    std::int32_t z_ = 0;

    [[nodiscard]] bool operator==(const BievrVoxelKey &_other) const noexcept
    {
        return x_ == _other.x_ && y_ == _other.y_ && z_ == _other.z_;
    }
};

struct BievrVoxelKeyHash
{
    [[nodiscard]] std::size_t operator()(const BievrVoxelKey &_key) const noexcept
    {
        const std::uint64_t x = expandBits(_key.x_);
        const std::uint64_t y = expandBits(_key.y_);
        const std::uint64_t z = expandBits(_key.z_);
        return static_cast<std::size_t>(x | (y << 1U) | (z << 2U));
    }

private:
    [[nodiscard]] static std::uint64_t expandBits(const std::int32_t _value) noexcept
    {
        std::uint64_t expanded = static_cast<std::uint64_t>(static_cast<std::int64_t>(_value) + (1LL << 20)) &
                                 0x1fffffULL;
        expanded = (expanded | (expanded << 32U)) & 0x1f00000000ffffULL;
        expanded = (expanded | (expanded << 16U)) & 0x1f0000ff0000ffULL;
        expanded = (expanded | (expanded << 8U)) & 0x100f00f00f00f00fULL;
        expanded = (expanded | (expanded << 4U)) & 0x10c30c30c30c30c3ULL;
        expanded = (expanded | (expanded << 2U)) & 0x1249249249249249ULL;
        return expanded;
    }
};

struct BievrIntensityImage
{
    Eigen::MatrixXf image;
    Eigen::MatrixXf smoothed;
    Eigen::MatrixXf weights;
    Eigen::Vector2d gradient_strength = Eigen::Vector2d::Zero();
    std::vector<float> pending;
};

struct BievrVoxel
{
    bool observed_ = false;
    Eigen::Isometry3d cell_from_world_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d orientation_from_world_ = Eigen::Isometry3d::Identity();
    Eigen::MatrixXf bump_image_;
    Eigen::MatrixXf bump_image_smoothed_;
    Eigen::MatrixXf bump_weights_;
    Eigen::Matrix3d centered_outer_sum_ = Eigen::Matrix3d::Zero();
    Eigen::Vector3d mean_ = Eigen::Vector3d::Zero();
    std::size_t point_count_ = 0U;
    double mean_image_distance_ = 0.0;
    std::vector<Eigen::Vector4d> pending_points_;
    std::unique_ptr<BievrIntensityImage> intensity_;
};

struct BievrMeasurement
{
    Eigen::RowVector3d position_jacobian_ = Eigen::RowVector3d::Zero();
    double residual_ = 0.0;
};

// Derived from a voxel's unsmoothed Base observations. Its lifetime is one
// registration scan, during which the voxel frame and observations are fixed.
struct BievrBumpLevel
{
    double pixel_size = 0.0;
    Eigen::MatrixXf mean_height;
    Eigen::MatrixXf smoothed_height;
    Eigen::MatrixXd effective_weight;
    Eigen::MatrixXd height_variance;
    Eigen::MatrixXi support_count; // Number of contributing Base pixels, not LiDAR returns.
    std::size_t observed_pixels = 0;

    [[nodiscard]] double coverage() const noexcept
    {
        return effective_weight.size() ? static_cast<double>(observed_pixels) / effective_weight.size() : 0.0;
    }
};

class BievrVoxelMap
{
public:
    struct Parameters
    {
        std::size_t maximum_voxel_count_ = 1500000U;
        double voxel_size_ = 0.5;
        double pixel_size_ = 0.05;
        double normal_tolerance_degrees_ = 3.0;
        bool use_range_weight_ = true;
        bool smooth_image_ = true;
        bool intensity_enabled_ = false;
        bool smooth_intensity_ = true;
    };

    struct SamplingParameters
    {
        double fine_voxel_size_ = 0.1;
        std::size_t informed_voxel_count_ = 300U;
        bool informed_sampling_enabled_ = true;
    };

    BievrVoxelMap()
    {
        configure(Parameters{});
    }

    explicit BievrVoxelMap(const Parameters &_parameters)
    {
        configure(_parameters);
    }

    void configure(const Parameters &_parameters)
    {
        if (!(_parameters.voxel_size_ > 0.0) || !(_parameters.pixel_size_ > 0.0))
        {
            throw std::invalid_argument("BIEVR voxel and pixel sizes must be positive");
        }
        if (_parameters.maximum_voxel_count_ == 0U)
        {
            throw std::invalid_argument("BIEVR maximum voxel count must be positive");
        }

        parameters_ = _parameters;
        inverse_voxel_size_ = 1.0 / parameters_.voxel_size_;
        inverse_pixel_size_ = 1.0 / parameters_.pixel_size_;
        normal_tolerance_radians_ = parameters_.normal_tolerance_degrees_ * kPi / 180.0;

        corner_offsets_.resize(3, 8);
        int index = 0;
        for (int x : {0, 1})
        {
            for (int y : {0, 1})
            {
                for (int z : {0, 1})
                {
                    corner_offsets_.col(index++) = parameters_.voxel_size_ * Eigen::Vector3d(x, y, z);
                }
            }
        }

        const float sigma = 0.8F;
        gaussian_kernel_.resize(3, 3);
        float sum = 0.0F;
        for (int y = -1; y <= 1; ++y)
        {
            for (int x = -1; x <= 1; ++x)
            {
                const float value = std::exp(-static_cast<float>(x * x + y * y) / (2.0F * sigma * sigma));
                gaussian_kernel_(y + 1, x + 1) = value;
                sum += value;
            }
        }
        gaussian_kernel_ /= sum;
        clear();
    }

    void clear()
    {
        voxels_.clear();
        least_recently_used_keys_.clear();
    }

    [[nodiscard]] bool empty() const noexcept
    {
        return voxels_.empty();
    }

    [[nodiscard]] std::size_t voxelCount() const noexcept
    {
        return voxels_.size();
    }

    [[nodiscard]] BievrVoxelKey pointToKey(const Eigen::Vector3d &_point) const noexcept
    {
        return {static_cast<std::int32_t>(std::floor(_point.x() * inverse_voxel_size_)),
                static_cast<std::int32_t>(std::floor(_point.y() * inverse_voxel_size_)),
                static_cast<std::int32_t>(std::floor(_point.z() * inverse_voxel_size_))};
    }

    [[nodiscard]] bool canIndex(const Eigen::Vector3d &point) const noexcept
    {
        constexpr double maximum_index = std::numeric_limits<std::int32_t>::max() - 2.0;
        return point.allFinite() && point.cwiseAbs().maxCoeff() * inverse_voxel_size_ <= maximum_index;
    }

    [[nodiscard]] const BievrVoxel *findVoxel(const BievrVoxelKey &_key) const
    {
        const auto iterator = voxels_.find(_key);
        if (iterator == voxels_.end() || !iterator->second.voxel_.observed_)
        {
            return nullptr;
        }
        return &iterator->second.voxel_;
    }

    // The caller validates indexing bounds with canIndex before querying.
    [[nodiscard]] const BievrVoxel *queryVoxel(const Eigen::Vector3d &point_world, BievrVoxelKey *matched_key = nullptr) const
    {
        const BievrVoxelKey key = pointToKey(point_world);
        const BievrVoxel *voxel = findVoxel(key);
        if (!voxel) return findNearestVoxel(point_world, matched_key);
        if (matched_key) *matched_key = key;
        return voxel;
    }

    [[nodiscard]] bool sample(const Eigen::Vector3d &_point_world,
                              BievrMeasurement &_measurement) const
    {
        const BievrVoxel *voxel = queryVoxel(_point_world);
        return voxel && sampleBase(*voxel, _point_world, _measurement);
    }

    [[nodiscard]] bool sampleBase(const BievrVoxel &voxel, const Eigen::Vector3d &_point_world,
                                  BievrMeasurement &_measurement) const
    {
        const Eigen::Vector3d point_cell = voxel.cell_from_world_ * _point_world;
        const double x = point_cell.x() * inverse_pixel_size_;
        const double y = point_cell.y() * inverse_pixel_size_;
        double image_value = 0.0;
        double gradient_x = 0.0;
        double gradient_y = 0.0;
        if (!sampleValueAndGradient(voxel, x, y, image_value, gradient_x, gradient_y))
        {
            return false;
        }

        Eigen::RowVector3d local_jacobian;
        local_jacobian << -gradient_x * inverse_pixel_size_,
            -gradient_y * inverse_pixel_size_,
            1.0;
        _measurement.position_jacobian_.noalias() = local_jacobian * voxel.cell_from_world_.linear();
        _measurement.residual_ = point_cell.z() - image_value;
        return true;
    }

    [[nodiscard]] bool buildCoarse(const BievrVoxel &voxel, double pixel_size, BievrBumpLevel &level) const
    {
        level = BievrBumpLevel{};
        if (!voxel.observed_ || !(pixel_size > parameters_.pixel_size_) || !std::isfinite(pixel_size) ||
            voxel.bump_image_.size() == 0) return false;

        const double ratio = parameters_.pixel_size_ / pixel_size;
        const Eigen::Index height = static_cast<Eigen::Index>(std::ceil((voxel.bump_image_.rows() - 1) * ratio)) + 1;
        const Eigen::Index width = static_cast<Eigen::Index>(std::ceil((voxel.bump_image_.cols() - 1) * ratio)) + 1;
        if (width < 2 || height < 2) return false;

        level.pixel_size = pixel_size;
        level.effective_weight.setZero(height, width);
        level.height_variance.setZero(height, width);
        level.support_count.setZero(height, width);
        Eigen::MatrixXd mean = Eigen::MatrixXd::Zero(height, width);
        for (Eigen::Index column = 0; column < voxel.bump_image_.cols(); ++column)
        {
            // Both lattices begin at the same cell-frame origin. A measured Base
            // pixel contributes once to the nearest Coarse pixel in that frame.
            const Eigen::Index coarse_column = static_cast<Eigen::Index>(std::round(column * ratio));
            for (Eigen::Index row = 0; row < voxel.bump_image_.rows(); ++row)
            {
                const double base_weight = voxel.bump_weights_(row, column);
                const double base_height = voxel.bump_image_(row, column);
                if (!(base_weight > 0.0) || !std::isfinite(base_weight) || !std::isfinite(base_height)) continue;

                const Eigen::Index coarse_row = static_cast<Eigen::Index>(std::round(row * ratio));
                double &weight = level.effective_weight(coarse_row, coarse_column);
                double &height_mean = mean(coarse_row, coarse_column);
                int &count = level.support_count(coarse_row, coarse_column);
                if (count == 0) ++level.observed_pixels;
                ++count;
                weight += base_weight;
                const double difference = base_height - height_mean;
                height_mean += (base_weight / weight) * difference;
                level.height_variance(coarse_row, coarse_column) += base_weight * difference * (base_height - height_mean);
            }
        }
        if (level.observed_pixels == 0)
        {
            level = BievrBumpLevel{};
            return false;
        }
        for (Eigen::Index column = 0; column < width; ++column)
        {
            for (Eigen::Index row = 0; row < height; ++row)
            {
                const double weight = level.effective_weight(row, column);
                if (weight > 0.0)
                    level.height_variance(row, column) = std::max(0.0, level.height_variance(row, column) / weight);
            }
        }
        level.mean_height = mean.cast<float>();
        if (parameters_.smooth_image_)
            smoothCoarse(level);
        else
            level.smoothed_height = level.mean_height;
        return true;
    }

    [[nodiscard]] bool sampleCoarse(const BievrVoxel &voxel, const BievrBumpLevel &level,
                                    const Eigen::Vector3d &point_world, BievrMeasurement &measurement) const
    {
        if (!(level.pixel_size > 0.0) || !std::isfinite(level.pixel_size) || level.observed_pixels == 0 ||
            level.effective_weight.rows() != level.smoothed_height.rows() ||
            level.effective_weight.cols() != level.smoothed_height.cols()) return false;
        const Eigen::Vector3d point_cell = voxel.cell_from_world_ * point_world;
        const double inverse_scale = 1.0 / level.pixel_size;
        const double x = point_cell.x() * inverse_scale;
        const double y = point_cell.y() * inverse_scale;
        if (!point_cell.allFinite() || !(x >= 0.0 && x < level.smoothed_height.cols() - 1 &&
                                       y >= 0.0 && y < level.smoothed_height.rows() - 1) ||
            point_cell.x() > (voxel.bump_image_.cols() - 1) * parameters_.pixel_size_ ||
            point_cell.y() > (voxel.bump_image_.rows() - 1) * parameters_.pixel_size_) return false;

        double height, gradient_x, gradient_y;
        if (!sampleValueAndGradient(level.smoothed_height, level.effective_weight, x, y,
                                    height, gradient_x, gradient_y, true)) return false;

        const Eigen::RowVector3d local_jacobian(-gradient_x * inverse_scale, -gradient_y * inverse_scale, 1.0);
        measurement.position_jacobian_.noalias() = local_jacobian * voxel.cell_from_world_.linear();
        measurement.residual_ = point_cell.z() - height;
        return measurement.position_jacobian_.allFinite() && std::isfinite(measurement.residual_);
    }

    [[nodiscard]] bool sampleIntensity(const Eigen::Vector3d &point_world, float scan_intensity,
                                       BievrMeasurement &measurement) const
    {
        if (!parameters_.intensity_enabled_ || !(scan_intensity >= 0.0F && scan_intensity <= 255.0F) || !canIndex(point_world)) return false;
        const BievrVoxel *voxel = findVoxel(pointToKey(point_world));
        if (!voxel) voxel = findNearestVoxel(point_world);
        if (!voxel || !voxel->intensity_) return false;

        const auto &intensity = *voxel->intensity_;
        const Eigen::Vector3d point_cell = voxel->cell_from_world_ * point_world;
        const double x = point_cell.x() * inverse_pixel_size_;
        const double y = point_cell.y() * inverse_pixel_size_;
        double value, gradient_x, gradient_y;
        if (!sampleValueAndGradient(intensity.smoothed, intensity.weights, x, y,
                                    value, gradient_x, gradient_y, true)) return false;

        const Eigen::RowVector2d local_gradient(-gradient_x * inverse_pixel_size_, -gradient_y * inverse_pixel_size_);
        measurement.position_jacobian_.noalias() = local_gradient * voxel->cell_from_world_.linear().topRows<2>();
        measurement.residual_ = static_cast<double>(scan_intensity) - value;
        return true;
    }

    static void downsampleNearestToVoxelCenter(const pcl::PointCloud<pcl::PointXYZINormal> &_input,
                                               const double _voxel_size,
                                               pcl::PointCloud<pcl::PointXYZINormal> &_output,
                                               std::vector<std::size_t> *source_indices = nullptr)
    {
        if (!(_voxel_size > 0.0))
        {
            throw std::invalid_argument("BIEVR downsampling voxel size must be positive");
        }
        if (_input.empty())
        {
            _output.clear();
            if (source_indices) source_indices->clear();
            return;
        }

        const double inverse_voxel_size = 1.0 / _voxel_size;
        std::vector<DownsampleEntry> entries(_input.size());
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0U, _input.size()),
                          [&_input, _voxel_size, inverse_voxel_size, &entries](const tbb::blocked_range<std::size_t> &_range)
        {
            for (std::size_t index = _range.begin(); index != _range.end(); ++index)
            {
                const pcl::PointXYZINormal &point = _input.points[index];
                const Eigen::Vector3d position(point.x, point.y, point.z);
                const BievrVoxelKey key{static_cast<std::int32_t>(std::floor(position.x() * inverse_voxel_size)),
                                        static_cast<std::int32_t>(std::floor(position.y() * inverse_voxel_size)),
                                        static_cast<std::int32_t>(std::floor(position.z() * inverse_voxel_size))};
                const Eigen::Vector3d center = _voxel_size * Eigen::Vector3d(static_cast<double>(key.x_) + 0.5,
                                                                             static_cast<double>(key.y_) + 0.5,
                                                                             static_cast<double>(key.z_) + 0.5);
                entries[index] = {key, index, (position - center).squaredNorm()};
            }
        });

        tbb::parallel_sort(entries.begin(),
                           entries.end(),
                           [](const DownsampleEntry &_left, const DownsampleEntry &_right)
        {
            return std::tuple(_left.key_.x_,
                              _left.key_.y_,
                              _left.key_.z_,
                              _left.squared_distance_,
                              _left.point_index_) <
                   std::tuple(_right.key_.x_,
                              _right.key_.y_,
                              _right.key_.z_,
                              _right.squared_distance_,
                              _right.point_index_);
        });

        std::vector<std::size_t> selected_indices;
        selected_indices.reserve(entries.size());
        for (std::size_t index = 0U; index < entries.size(); ++index)
        {
            if (index == 0U || !(entries[index].key_ == entries[index - 1U].key_))
            {
                selected_indices.push_back(entries[index].point_index_);
            }
        }

        _output.resize(selected_indices.size());
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0U, selected_indices.size()),
                          [&_input, &_output, &selected_indices](const tbb::blocked_range<std::size_t> &_range)
        {
            for (std::size_t index = _range.begin(); index != _range.end(); ++index)
            {
                _output.points[index] = _input.points[selected_indices[index]];
            }
        });
        if (source_indices) *source_indices = std::move(selected_indices);
    }

    void update(const pcl::PointCloud<pcl::PointXYZINormal> &_points_world,
                const std::vector<double> *_ranges = nullptr,
                const std::vector<float> *intensities = nullptr)
    {
        if (_points_world.empty())
        {
            return;
        }
        if (_ranges != nullptr && _ranges->size() != _points_world.size())
        {
            throw std::invalid_argument("BIEVR range count must equal point count");
        }
        if (intensities && intensities->size() != _points_world.size())
            throw std::invalid_argument("BIEVR intensity count must equal point count");
        if (!parameters_.intensity_enabled_) intensities = nullptr;

        std::vector<HashedPoint> hashed_points(_points_world.size());
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0U, _points_world.size()),
                          [this, &_points_world, _ranges, intensities, &hashed_points](const tbb::blocked_range<std::size_t> &_range)
        {
            for (std::size_t index = _range.begin(); index != _range.end(); ++index)
            {
                const pcl::PointXYZINormal &point = _points_world.points[index];
                hashed_points[index].point_ << point.x,
                    point.y,
                    point.z,
                    _ranges == nullptr ? 1.0 : (*_ranges)[index];
                hashed_points[index].key_ = pointToKey(hashed_points[index].point_.head<3>());
                if (intensities)
                {
                    const float intensity = (*intensities)[index];
                    if (intensity >= 0.0F && intensity <= 255.0F) hashed_points[index].intensity = intensity;
                }
            }
        });

        tbb::parallel_sort(hashed_points.begin(),
                           hashed_points.end(),
                           [](const HashedPoint &_left, const HashedPoint &_right)
        {
            // Canonicalize all values consumed by map accumulation. Equal x alone
            // leaves distinct points equivalent to parallel_sort and makes the
            // floating-point accumulation depend on their arrival/sort order.
            const auto left_key = std::tuple(_left.key_.x_,
                                             _left.key_.y_,
                                             _left.key_.z_,
                                             _left.point_.x(),
                                             _left.point_.y(),
                                             _left.point_.z(),
                                             _left.point_.w(),
                                             _left.intensity);
            const auto right_key = std::tuple(_right.key_.x_,
                                              _right.key_.y_,
                                              _right.key_.z_,
                                              _right.point_.x(),
                                              _right.point_.y(),
                                              _right.point_.z(),
                                              _right.point_.w(),
                                              _right.intensity);
            return left_key < right_key;
        });

        std::vector<std::size_t> group_starts;
        group_starts.reserve(hashed_points.size());
        for (std::size_t index = 0U; index < hashed_points.size(); ++index)
        {
            if (index == 0U || !(hashed_points[index].key_ == hashed_points[index - 1U].key_))
            {
                group_starts.push_back(index);
                const BievrVoxelKey &key = hashed_points[index].key_;
                if (voxels_.find(key) == voxels_.end())
                {
                    least_recently_used_keys_.push_front(key);
                    auto result = voxels_.emplace(key, VoxelEntry{});
                    result.first->second.lru_iterator_ = least_recently_used_keys_.begin();
                    result.first->second.voxel_.pending_points_.reserve(8U);
                }
            }
        }

        std::vector<typename VoxelMap::iterator> voxel_iterators(group_starts.size());
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0U, group_starts.size()),
                          [this, &hashed_points, &group_starts, &voxel_iterators, intensities](const tbb::blocked_range<std::size_t> &_range)
        {
            std::vector<Eigen::Vector4d> voxel_points;
            std::vector<float> voxel_intensities;
            for (std::size_t group_index = _range.begin(); group_index != _range.end(); ++group_index)
            {
                const std::size_t begin = group_starts[group_index];
                const std::size_t end = group_index + 1U < group_starts.size() ? group_starts[group_index + 1U] : hashed_points.size();
                auto iterator = voxels_.find(hashed_points[begin].key_);
                voxel_iterators[group_index] = iterator;
                BievrVoxel &voxel = iterator->second.voxel_;

                voxel_points.clear();
                voxel_intensities.clear();
                voxel_points.reserve(end - begin);
                for (std::size_t index = begin; index < end; ++index)
                {
                    const Eigen::Vector3d point = hashed_points[index].point_.head<3>();
                    ++voxel.point_count_;
                    const Eigen::Vector3d mean_delta = point - voxel.mean_;
                    voxel.mean_ += mean_delta / static_cast<double>(voxel.point_count_);
                    voxel.centered_outer_sum_.noalias() += mean_delta * (point - voxel.mean_).transpose();
                    voxel_points.push_back(hashed_points[index].point_);
                }

                if (intensities && !voxel.intensity_ &&
                    std::any_of(hashed_points.begin() + begin, hashed_points.begin() + end,
                                [](const HashedPoint &point) { return point.intensity >= 0.0F; }))
                {
                    voxel.intensity_ = std::make_unique<BievrIntensityImage>();
                    voxel.intensity_->pending.resize(voxel.pending_points_.size(), -1.0F);
                }
                if (voxel.intensity_)
                {
                    voxel_intensities.reserve(end - begin);
                    for (std::size_t index = begin; index < end; ++index)
                        voxel_intensities.push_back(hashed_points[index].intensity);
                }

                const bool was_observed = voxel.observed_;
                const bool normal_changed = updateNormal(voxel);
                if (!was_observed && voxel.observed_)
                {
                    voxel_points.insert(voxel_points.end(), voxel.pending_points_.begin(), voxel.pending_points_.end());
                    if (voxel.intensity_)
                    {
                        auto &pending = voxel.intensity_->pending;
                        voxel_intensities.insert(voxel_intensities.end(), pending.begin(), pending.end());
                        std::vector<float>().swap(pending);
                    }
                    voxel.pending_points_.clear();
                    voxel.pending_points_.shrink_to_fit();
                }
                else if (!voxel.observed_)
                {
                    voxel.pending_points_.insert(voxel.pending_points_.end(), voxel_points.begin(), voxel_points.end());
                    if (voxel.intensity_)
                        voxel.intensity_->pending.insert(voxel.intensity_->pending.end(), voxel_intensities.begin(), voxel_intensities.end());
                }
                updateBumpImage(voxel_points, voxel, normal_changed, voxel_intensities);
            }
        });

        for (const auto &iterator : voxel_iterators)
        {
            least_recently_used_keys_.splice(least_recently_used_keys_.begin(),
                                             least_recently_used_keys_,
                                             iterator->second.lru_iterator_);
        }
        while (voxels_.size() > parameters_.maximum_voxel_count_ && !least_recently_used_keys_.empty())
        {
            voxels_.erase(least_recently_used_keys_.back());
            least_recently_used_keys_.pop_back();
        }
    }

    void selectMeasurementPoints(const pcl::PointCloud<pcl::PointXYZINormal> &_fine_points,
                                 const Eigen::Quaterniond &_rotation_world_from_imu,
                                 const Eigen::Vector3d &_position_world_from_imu,
                                 const Eigen::Matrix3d &_rotation_imu_from_lidar,
                                 const Eigen::Vector3d &_position_imu_from_lidar,
                                 const SamplingParameters &_sampling,
                                 pcl::PointCloud<pcl::PointXYZINormal> &_selected_points,
                                 std::vector<std::size_t> *source_indices = nullptr) const
    {
        if (!_sampling.informed_sampling_enabled_ || _fine_points.size() <= _sampling.informed_voxel_count_)
        {
            _selected_points = _fine_points;
            if (source_indices)
            {
                source_indices->resize(_fine_points.size());
                std::iota(source_indices->begin(), source_indices->end(), std::size_t{0});
            }
            return;
        }

        std::vector<SamplingEntry> entries(_fine_points.size());
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0U, _fine_points.size()),
                          [this, &_fine_points, &_rotation_world_from_imu, &_position_world_from_imu, &_rotation_imu_from_lidar, &_position_imu_from_lidar, &entries](const tbb::blocked_range<std::size_t> &_range)
        {
            for (std::size_t index = _range.begin(); index != _range.end(); ++index)
            {
                const pcl::PointXYZINormal &point = _fine_points.points[index];
                const Eigen::Vector3d point_lidar(point.x, point.y, point.z);
                const Eigen::Vector3d point_world = _rotation_world_from_imu * (_rotation_imu_from_lidar * point_lidar + _position_imu_from_lidar) + _position_world_from_imu;
                entries[index] = {pointToKey(point_world), index};
            }
        });

        tbb::parallel_sort(entries.begin(),
                           entries.end(),
                           [](const SamplingEntry &_left, const SamplingEntry &_right)
        {
            return std::tie(_left.key_.x_, _left.key_.y_, _left.key_.z_, _left.point_index_) <
                   std::tie(_right.key_.x_, _right.key_.y_, _right.key_.z_, _right.point_index_);
        });

        std::vector<VoxelScore> scores;
        scores.reserve(entries.size());
        for (std::size_t index = 0U; index < entries.size();)
        {
            const std::size_t begin = index;
            while (index < entries.size() && entries[index].key_ == entries[begin].key_)
            {
                ++index;
            }
            const BievrVoxel *voxel = findVoxel(entries[begin].key_);
            const double score = voxel == nullptr || index - begin < 2U ? 0.0 : voxel->mean_image_distance_;
            scores.push_back({score, entries[begin].key_, entries[begin].point_index_});
        }

        tbb::parallel_sort(scores.begin(),
                           scores.end(),
                           [](const VoxelScore &_left, const VoxelScore &_right)
        {
            if (_left.score_ != _right.score_)
            {
                return _left.score_ > _right.score_;
            }
            return std::tie(_left.key_.x_, _left.key_.y_, _left.key_.z_) <
                   std::tie(_right.key_.x_, _right.key_.y_, _right.key_.z_);
        });

        const std::size_t informed_count = std::min(_sampling.informed_voxel_count_, scores.size());
        std::unordered_set<BievrVoxelKey, BievrVoxelKeyHash> informed_keys;
        informed_keys.reserve(informed_count);
        for (std::size_t index = 0U; index < informed_count; ++index)
        {
            informed_keys.insert(scores[index].key_);
        }

        std::vector<std::size_t> selected_indices;
        selected_indices.reserve(entries.size());
        for (const SamplingEntry &entry : entries)
        {
            if (informed_keys.find(entry.key_) != informed_keys.end())
            {
                selected_indices.push_back(entry.point_index_);
            }
        }
        for (std::size_t index = informed_count; index < scores.size(); ++index)
        {
            selected_indices.push_back(scores[index].point_index_);
        }

        _selected_points.resize(selected_indices.size());
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0U, selected_indices.size()),
                          [&_fine_points, &_selected_points, &selected_indices](const tbb::blocked_range<std::size_t> &_range)
        {
            for (std::size_t index = _range.begin(); index != _range.end(); ++index)
            {
                _selected_points.points[index] = _fine_points.points[selected_indices[index]];
            }
        });
        if (source_indices) *source_indices = std::move(selected_indices);
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr std::size_t kMinimumValidPointCount = 4U;

    struct ImageBounds
    {
        int width_ = 0;
        int height_ = 0;
        double minimum_u_ = 0.0;
        double minimum_v_ = 0.0;
    };

    struct HashedPoint
    {
        BievrVoxelKey key_;
        float intensity = -1.0F;
        Eigen::Vector4d point_ = Eigen::Vector4d::Zero();
    };

    struct SamplingEntry
    {
        BievrVoxelKey key_;
        std::size_t point_index_ = 0U;
    };

    struct DownsampleEntry
    {
        BievrVoxelKey key_;
        std::size_t point_index_ = 0U;
        double squared_distance_ = 0.0;
    };

    struct VoxelScore
    {
        double score_ = 0.0;
        BievrVoxelKey key_;
        std::size_t point_index_ = 0U;
    };

    struct VoxelEntry
    {
        BievrVoxel voxel_;
        std::list<BievrVoxelKey>::iterator lru_iterator_;
    };

    using VoxelMap = std::unordered_map<BievrVoxelKey, VoxelEntry, BievrVoxelKeyHash>;

    [[nodiscard]] const BievrVoxel *findNearestVoxel(const Eigen::Vector3d &_point, BievrVoxelKey *matched_key = nullptr) const
    {
        const BievrVoxel *nearest_voxel = nullptr;
        double minimum_squared_distance = std::numeric_limits<double>::max();
        const BievrVoxelKey center_key = pointToKey(_point);
        constexpr std::array<std::array<int, 3>, 7> offsets{{{{0, 0, 0}},
                                                             {{1, 0, 0}},
                                                             {{-1, 0, 0}},
                                                             {{0, 1, 0}},
                                                             {{0, -1, 0}},
                                                             {{0, 0, 1}},
                                                             {{0, 0, -1}}}};
        for (const auto &offset : offsets)
        {
            const BievrVoxelKey key{center_key.x_ + offset[0],
                                    center_key.y_ + offset[1],
                                    center_key.z_ + offset[2]};
            const BievrVoxel *voxel = findVoxel(key);
            if (voxel == nullptr)
            {
                continue;
            }
            const double squared_distance = (_point - voxel->mean_).squaredNorm();
            if (squared_distance < minimum_squared_distance)
            {
                minimum_squared_distance = squared_distance;
                nearest_voxel = voxel;
                if (matched_key) *matched_key = key;
            }
        }
        return nearest_voxel;
    }

    bool updateNormal(BievrVoxel &_voxel)
    {
        if (_voxel.point_count_ < kMinimumValidPointCount)
        {
            return false;
        }

        const Eigen::Vector3d mean = _voxel.mean_;
        const Eigen::Matrix3d covariance = 0.5 * (_voxel.centered_outer_sum_ + _voxel.centered_outer_sum_.transpose()) /
                                           (static_cast<double>(_voxel.point_count_) - 1.0);
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success)
        {
            return false;
        }

        Eigen::Vector3d normal = solver.eigenvectors().col(0);
        bool normal_changed = !_voxel.observed_;
        if (_voxel.observed_)
        {
            const Eigen::Vector3d previous_normal = _voxel.orientation_from_world_.linear().row(2);
            const double cosine = std::clamp(std::abs(previous_normal.dot(normal)), 0.0, 1.0);
            normal_changed = std::acos(cosine) > normal_tolerance_radians_;
            if (previous_normal.dot(normal) < 0.0)
            {
                normal = -normal;
            }
        }

        if (normal_changed)
        {
            const Eigen::Vector3d x_axis = normal.unitOrthogonal();
            const Eigen::Vector3d y_axis = normal.cross(x_axis).normalized();
            Eigen::Matrix3d rotation_world_from_orientation;
            rotation_world_from_orientation.col(0) = x_axis;
            rotation_world_from_orientation.col(1) = y_axis;
            rotation_world_from_orientation.col(2) = normal;
            Eigen::Isometry3d world_from_orientation = Eigen::Isometry3d::Identity();
            world_from_orientation.linear() = rotation_world_from_orientation;
            world_from_orientation.translation() = mean;
            _voxel.orientation_from_world_ = world_from_orientation.inverse();
        }

        _voxel.observed_ = true;
        return normal_changed;
    }

    [[nodiscard]] ImageBounds computeImageBounds(const BievrVoxel &_voxel,
                                                 const Eigen::Vector3d &_reference_point) const
    {
        const BievrVoxelKey key = pointToKey(_reference_point);
        const Eigen::Vector3d voxel_origin(static_cast<double>(key.x_) * parameters_.voxel_size_,
                                           static_cast<double>(key.y_) * parameters_.voxel_size_,
                                           static_cast<double>(key.z_) * parameters_.voxel_size_);
        const Eigen::Matrix<double, 2, 8> projected = _voxel.orientation_from_world_.linear().topRows<2>() *
                                                          (corner_offsets_.colwise() + voxel_origin) +
                                                      _voxel.orientation_from_world_.translation().head<2>().replicate(1, 8);

        ImageBounds bounds;
        bounds.minimum_u_ = projected.row(0).minCoeff();
        bounds.minimum_v_ = projected.row(1).minCoeff();
        const double maximum_u = projected.row(0).maxCoeff();
        const double maximum_v = projected.row(1).maxCoeff();
        bounds.width_ = static_cast<int>(std::ceil((maximum_u - bounds.minimum_u_) * inverse_pixel_size_)) + 1;
        bounds.height_ = static_cast<int>(std::ceil((maximum_v - bounds.minimum_v_) * inverse_pixel_size_)) + 1;
        return bounds;
    }

    void reprojectImage(BievrVoxel &_voxel,
                        const ImageBounds &_bounds,
                        Eigen::MatrixXi &_changed)
    {
        const Eigen::MatrixXf old_image = _voxel.bump_image_;
        const Eigen::MatrixXf old_weights = _voxel.bump_weights_;
        const Eigen::Isometry3d world_from_old_cell = _voxel.cell_from_world_.inverse();
        Eigen::MatrixXf old_intensities, old_intensity_weights;
        if (_voxel.intensity_)
        {
            auto &intensity = *_voxel.intensity_;
            old_intensities.swap(intensity.image);
            old_intensity_weights.swap(intensity.weights);
            intensity.image.setZero(_bounds.height_, _bounds.width_);
            intensity.smoothed.setZero(_bounds.height_, _bounds.width_);
            intensity.weights.setZero(_bounds.height_, _bounds.width_);
        }

        _voxel.bump_image_.setZero(_bounds.height_, _bounds.width_);
        _voxel.bump_image_smoothed_.setZero(_bounds.height_, _bounds.width_);
        _voxel.bump_weights_.setZero(_bounds.height_, _bounds.width_);
        _changed.setZero(_bounds.height_, _bounds.width_);

        const Eigen::Vector3d planar_origin(_bounds.minimum_u_, _bounds.minimum_v_, 0.0);
        const Eigen::Vector3d world_origin = _voxel.orientation_from_world_.inverse() * planar_origin;
        Eigen::Isometry3d world_from_cell = Eigen::Isometry3d::Identity();
        world_from_cell.linear() = _voxel.orientation_from_world_.linear().transpose();
        world_from_cell.translation() = world_origin;
        _voxel.cell_from_world_ = world_from_cell.inverse();

        const Eigen::Isometry3d new_cell_from_old_cell = _voxel.cell_from_world_ * world_from_old_cell;
        for (int row = 0; row < old_image.rows(); ++row)
        {
            for (int column = 0; column < old_image.cols(); ++column)
            {
                if (old_weights(row, column) <= 0.0F)
                {
                    continue;
                }
                const Eigen::Vector3d old_point(static_cast<double>(column) * parameters_.pixel_size_,
                                                static_cast<double>(row) * parameters_.pixel_size_,
                                                old_image(row, column));
                const Eigen::Vector3d new_point = new_cell_from_old_cell * old_point;
                const int x = static_cast<int>(std::round(new_point.x() * inverse_pixel_size_));
                const int y = static_cast<int>(std::round(new_point.y() * inverse_pixel_size_));
                if (x < 0 || x >= _voxel.bump_image_.cols() || y < 0 || y >= _voxel.bump_image_.rows())
                {
                    continue;
                }
                const float target_weight = _voxel.bump_weights_(y, x);
                const float source_weight = old_weights(row, column);
                const float accumulated_weight = target_weight + source_weight;
                _voxel.bump_image_(y, x) = (_voxel.bump_image_(y, x) * target_weight +
                                            static_cast<float>(new_point.z()) * source_weight) /
                                           accumulated_weight;
                _voxel.bump_weights_(y, x) = accumulated_weight;
                _changed(y, x) = 1;
                if (old_intensity_weights.size() > 0 && old_intensity_weights(row, column) > 0.0F)
                {
                    auto &intensity = *_voxel.intensity_;
                    const float weight = old_intensity_weights(row, column);
                    const float previous_weight = intensity.weights(y, x);
                    intensity.weights(y, x) += weight;
                    intensity.image(y, x) = (intensity.image(y, x) * previous_weight + old_intensities(row, column) * weight) /
                                            intensity.weights(y, x);
                }
            }
        }
    }

    void integrateVoxelPoints(const std::vector<Eigen::Vector4d> &_points,
                              BievrVoxel &_voxel,
                              Eigen::MatrixXi &_changed,
                              const std::vector<float> &intensities)
    {
        for (std::size_t index = 0U; index < _points.size(); ++index)
        {
            const Eigen::Vector4d &point = _points[index];
            const Eigen::Vector3d point_cell = _voxel.cell_from_world_ * point.head<3>();
            const int x = static_cast<int>(std::round(point_cell.x() * inverse_pixel_size_));
            const int y = static_cast<int>(std::round(point_cell.y() * inverse_pixel_size_));
            if (x < 0 || x >= _voxel.bump_image_.cols() || y < 0 || y >= _voxel.bump_image_.rows())
            {
                continue;
            }

            const float previous_weight = _voxel.bump_weights_(y, x);
            const double safe_range = std::max(point.w(), 1.0e-6);
            const float new_weight = parameters_.use_range_weight_ ? static_cast<float>(std::min(0.5, 1.0 / safe_range)) : 1.0F;
            const float accumulated_weight = previous_weight + new_weight;
            _voxel.bump_image_(y, x) = (_voxel.bump_image_(y, x) * previous_weight + new_weight * static_cast<float>(point_cell.z())) /
                                       accumulated_weight;
            _voxel.bump_weights_(y, x) = accumulated_weight;
            _changed(y, x) = 1;
            if (_voxel.intensity_ && intensities[index] >= 0.0F)
            {
                auto &intensity = *_voxel.intensity_;
                const float previous_intensity_weight = intensity.weights(y, x);
                intensity.weights(y, x) += new_weight;
                intensity.image(y, x) = (intensity.image(y, x) * previous_intensity_weight + new_weight * intensities[index]) /
                                        intensity.weights(y, x);
            }
        }
    }

    void dilateChangedMask(const Eigen::MatrixXi &_changed,
                           const Eigen::MatrixXf &_weights,
                           Eigen::MatrixXi &_dilated) const
    {
        _dilated.setZero(_changed.rows(), _changed.cols());
        for (int row = 0; row < _changed.rows(); ++row)
        {
            for (int column = 0; column < _changed.cols(); ++column)
            {
                if (_changed(row, column) == 0)
                {
                    continue;
                }
                for (int row_offset = -1; row_offset <= 1; ++row_offset)
                {
                    const int neighbor_row = row + row_offset;
                    if (neighbor_row < 0 || neighbor_row >= _changed.rows())
                    {
                        continue;
                    }
                    for (int column_offset = -1; column_offset <= 1; ++column_offset)
                    {
                        const int neighbor_column = column + column_offset;
                        if (neighbor_column >= 0 && neighbor_column < _changed.cols() &&
                            _weights(neighbor_row, neighbor_column) > 0.0F)
                        {
                            _dilated(neighbor_row, neighbor_column) = 1;
                        }
                    }
                }
            }
        }
    }

    void smoothImage(const Eigen::MatrixXf &_image,
                     const Eigen::MatrixXf &_weights,
                     const Eigen::MatrixXi &_changed,
                     Eigen::MatrixXf &_smoothed) const
    {
        for (int row = 0; row < _image.rows(); ++row)
        {
            for (int column = 0; column < _image.cols(); ++column)
            {
                if (_changed(row, column) == 0)
                {
                    continue;
                }
                float weighted_sum = 0.0F;
                float kernel_sum = 0.0F;
                for (int row_offset = -1; row_offset <= 1; ++row_offset)
                {
                    const int neighbor_row = row + row_offset;
                    if (neighbor_row < 0 || neighbor_row >= _image.rows())
                    {
                        continue;
                    }
                    for (int column_offset = -1; column_offset <= 1; ++column_offset)
                    {
                        const int neighbor_column = column + column_offset;
                        if (neighbor_column < 0 || neighbor_column >= _image.cols() ||
                            _weights(neighbor_row, neighbor_column) <= 0.0F)
                        {
                            continue;
                        }
                        const float kernel_weight = gaussian_kernel_(row_offset + 1, column_offset + 1);
                        weighted_sum += _image(neighbor_row, neighbor_column) * kernel_weight;
                        kernel_sum += kernel_weight;
                    }
                }
                _smoothed(row, column) = kernel_sum > 0.0F ? weighted_sum / kernel_sum : 0.0F;
            }
        }
    }

    void smoothCoarse(BievrBumpLevel &level) const
    {
        const auto &weights = level.effective_weight;
        level.smoothed_height.setZero(weights.rows(), weights.cols());
        for (Eigen::Index column = 0; column < weights.cols(); ++column)
        {
            for (Eigen::Index row = 0; row < weights.rows(); ++row)
            {
                if (!(weights(row, column) > 0.0)) continue;
                double sum_height = 0.0, sum_weight = 0.0;
                for (auto source_column = std::max<Eigen::Index>(0, column - 1);
                     source_column < std::min(weights.cols(), column + 2); ++source_column)
                {
                    for (auto source_row = std::max<Eigen::Index>(0, row - 1);
                         source_row < std::min(weights.rows(), row + 2); ++source_row)
                    {
                        const double weight = weights(source_row, source_column) *
                            gaussian_kernel_(source_row - row + 1, source_column - column + 1);
                        sum_height += weight * level.mean_height(source_row, source_column);
                        sum_weight += weight;
                    }
                }
                level.smoothed_height(row, column) = static_cast<float>(sum_height / sum_weight);
            }
        }
    }

    void updateImageScore(BievrVoxel &_voxel) const
    {
        std::size_t observed_pixel_count = 0U;
        double absolute_height_sum = 0.0;
        for (int row = 0; row < _voxel.bump_weights_.rows(); ++row)
        {
            for (int column = 0; column < _voxel.bump_weights_.cols(); ++column)
            {
                if (_voxel.bump_weights_(row, column) > 0.0F)
                {
                    ++observed_pixel_count;
                    absolute_height_sum += std::abs(static_cast<double>(_voxel.bump_image_smoothed_(row, column)));
                }
            }
        }
        _voxel.mean_image_distance_ = observed_pixel_count < 5U ? 0.0 : absolute_height_sum / static_cast<double>(observed_pixel_count);
    }

    bool updateBumpImage(const std::vector<Eigen::Vector4d> &_points,
                         BievrVoxel &_voxel,
                         const bool _normal_changed,
                         const std::vector<float> &intensities)
    {
        if (!_voxel.observed_ || _points.empty())
        {
            return false;
        }

        Eigen::MatrixXi changed;
        if (_normal_changed)
        {
            reprojectImage(_voxel, computeImageBounds(_voxel, _points.front().head<3>()), changed);
        }
        else
        {
            changed.setZero(_voxel.bump_image_.rows(), _voxel.bump_image_.cols());
        }
        if (_voxel.intensity_ && _voxel.intensity_->weights.size() == 0)
        {
            auto &intensity = *_voxel.intensity_;
            intensity.image.setZero(changed.rows(), changed.cols());
            intensity.smoothed.setZero(changed.rows(), changed.cols());
            intensity.weights.setZero(changed.rows(), changed.cols());
        }
        integrateVoxelPoints(_points, _voxel, changed, intensities);

        Eigen::MatrixXi changed_for_smoothing;
        if (_normal_changed)
        {
            changed_for_smoothing = changed;
        }
        else
        {
            dilateChangedMask(changed, _voxel.bump_weights_, changed_for_smoothing);
        }
        if (parameters_.smooth_image_)
        {
            smoothImage(_voxel.bump_image_,
                        _voxel.bump_weights_,
                        changed_for_smoothing,
                        _voxel.bump_image_smoothed_);
        }
        else
        {
            _voxel.bump_image_smoothed_ = _voxel.bump_image_;
        }
        updateImageScore(_voxel);
        if (_voxel.intensity_)
        {
            auto &intensity = *_voxel.intensity_;
            if (parameters_.smooth_intensity_)
                smoothImage(intensity.image, intensity.weights, changed_for_smoothing, intensity.smoothed);
            else
                intensity.smoothed = intensity.image;

            double gradient_x = 0.0, gradient_y = 0.0;
            for (int column = 0; column < intensity.weights.cols(); ++column)
            {
                for (int row = 0; row < intensity.weights.rows(); ++row)
                {
                    if (column > 0 && column + 1 < intensity.weights.cols() &&
                        intensity.weights(row, column - 1) > 0.0F && intensity.weights(row, column + 1) > 0.0F)
                        gradient_x += std::abs(intensity.smoothed(row, column + 1) - intensity.smoothed(row, column - 1));
                    if (row > 0 && row + 1 < intensity.weights.rows() &&
                        intensity.weights(row - 1, column) > 0.0F && intensity.weights(row + 1, column) > 0.0F)
                        gradient_y += std::abs(intensity.smoothed(row + 1, column) - intensity.smoothed(row - 1, column));
                }
            }
            intensity.gradient_strength << 0.5 * gradient_x, 0.5 * gradient_y;
        }
        return true;
    }

    template <typename WeightMatrix>
    [[nodiscard]] static bool interpolateValue(const Eigen::MatrixXf &image,
                                               const WeightMatrix &image_weights,
                                               const double _x,
                                               const double _y,
                                               double &_value)
    {
        const int x0 = static_cast<int>(std::floor(_x));
        const int y0 = static_cast<int>(std::floor(_y));
        const int x1 = x0 + 1;
        const int y1 = y0 + 1;
        if (x0 < 0 || y0 < 0 || x1 >= image.cols() || y1 >= image.rows())
        {
            return false;
        }

        const double dx = _x - static_cast<double>(x0);
        const double dy = _y - static_cast<double>(y0);
        const std::array<double, 4> weights{{(1.0 - dx) * (1.0 - dy) * (image_weights(y0, x0) > 0.0F),
                                             dx * (1.0 - dy) * (image_weights(y0, x1) > 0.0F),
                                             (1.0 - dx) * dy * (image_weights(y1, x0) > 0.0F),
                                             dx * dy * (image_weights(y1, x1) > 0.0F)}};
        const double weight_sum = weights[0] + weights[1] + weights[2] + weights[3];
        if (!(weight_sum > 0.0))
        {
            return false;
        }
        _value = (weights[0] * image(y0, x0) +
                  weights[1] * image(y0, x1) +
                  weights[2] * image(y1, x0) +
                  weights[3] * image(y1, x1)) /
                 weight_sum;
        return true;
    }

    [[nodiscard]] static bool sampleValueAndGradient(const BievrVoxel &_voxel,
                                                     const double _x,
                                                     const double _y,
                                                     double &_value,
                                                     double &_gradient_x,
                                                     double &_gradient_y)
    {
        return sampleValueAndGradient(_voxel.bump_image_smoothed_, _voxel.bump_weights_,
                                      _x, _y, _value, _gradient_x, _gradient_y);
    }

    template <typename WeightMatrix>
    [[nodiscard]] static bool sampleValueAndGradient(const Eigen::MatrixXf &image,
                                                     const WeightMatrix &image_weights,
                                                     double _x, double _y, double &_value,
                                                     double &_gradient_x, double &_gradient_y,
                                                     bool require_gradient_support = false)
    {
        if (!interpolateValue(image, image_weights, _x, _y, _value))
        {
            return false;
        }
        double negative_value = 0.0;
        double positive_value = 0.0;
        const bool horizontal = interpolateValue(image, image_weights, _x - 1.0, _y, negative_value) &&
                                interpolateValue(image, image_weights, _x + 1.0, _y, positive_value);
        _gradient_x = horizontal ? 0.5 * (positive_value - negative_value) : 0.0;
        const bool vertical = interpolateValue(image, image_weights, _x, _y - 1.0, negative_value) &&
                              interpolateValue(image, image_weights, _x, _y + 1.0, positive_value);
        _gradient_y = vertical ? 0.5 * (positive_value - negative_value) : 0.0;
        return !require_gradient_support || horizontal || vertical;
    }

    Parameters parameters_;
    double inverse_voxel_size_ = 2.0;
    double inverse_pixel_size_ = 20.0;
    double normal_tolerance_radians_ = 0.05235987755982989;
    Eigen::Matrix<double, 3, 8> corner_offsets_;
    Eigen::Matrix3f gaussian_kernel_ = Eigen::Matrix3f::Zero();
    VoxelMap voxels_;
    std::list<BievrVoxelKey> least_recently_used_keys_;
};
} // namespace fast_livo::bievr
