#include "bievr_adaptive.h"
#include "bievr_voxel_map.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace fast_livo::bievr
{
namespace
{
using PoseMatrix = Eigen::Matrix<double, 6, 6>;
using PoseVector = Eigen::Matrix<double, 6, 1>;
using VoxelOrder = std::tuple<std::int32_t, std::int32_t, std::int32_t>;

constexpr double kMeasurementVariance = 0.001;
constexpr double kSparseBaseSupport = 0.60;
constexpr double kMinimumRetainedBaseSupport = 0.70;

VoxelOrder voxelOrder(const BievrVoxelKey &key)
{
  return {key.x_, key.y_, key.z_};
}

bool validMeasurement(const BievrMeasurement &measurement)
{
  return measurement.position_jacobian_.allFinite() && std::isfinite(measurement.residual_);
}

PoseVector poseJacobian(const Eigen::Vector3d &point_imu, const Eigen::Matrix3d &world_to_imu,
                        const BievrMeasurement &measurement)
{
  PoseVector jacobian;
  jacobian.tail<3>() = measurement.position_jacobian_.transpose();
  jacobian.head<3>() = point_imu.cross(world_to_imu * jacobian.tail<3>());
  return jacobian;
}

double precision(double residual, double huber_delta)
{
  const double absolute = std::abs(residual);
  return (absolute > huber_delta ? huber_delta / absolute : 1.0) / kMeasurementVariance;
}

struct GeometrySpectrum
{
  BievrAdaptiveDiagnostics diagnostics;
  PoseMatrix weak_projector = PoseMatrix::Zero();
};

GeometrySpectrum analyzeGeometry(const PoseMatrix &information, std::size_t candidates, std::size_t matches,
                                 const BievrAdaptiveParams &params)
{
  GeometrySpectrum result;
  auto &stats = result.diagnostics;
  stats.candidates = candidates;
  stats.base_valid = matches;
  stats.predicted_valid = matches;
  if (matches < static_cast<std::size_t>(params.min_effective_points) || !information.allFinite()) return result;

  PoseVector scaling = PoseVector::Ones();
  scaling.head<3>().setConstant(1.0 / params.rotation_reference_m);
  const PoseMatrix normalized = scaling.asDiagonal() * information * scaling.asDiagonal();
  const Eigen::SelfAdjointEigenSolver<PoseMatrix> spectrum(normalized);
  if (spectrum.info() != Eigen::Success) return result;
  const auto values = spectrum.eigenvalues().cwiseMax(0.0);
  for (std::size_t i = 0; i < stats.base_eigenvalues.size(); ++i) stats.base_eigenvalues[i] = values(i);
  stats.base_min_eigenvalue = values(0);
  if (!(values(5) > 0.0)) return result;
  stats.base_eigen_ratio = values(0) / values(5);

  const double cutoff = params.degeneracy_ratio * values(5);
  while (stats.weak_rank < 5 && values(stats.weak_rank) < cutoff)
  {
    stats.base_weak_information += values(stats.weak_rank);
    ++stats.weak_rank;
  }
  stats.selected_weak_information = stats.base_weak_information;
  if (stats.weak_rank > 0)
  {
    const auto weak = spectrum.eigenvectors().leftCols(stats.weak_rank);
    result.weak_projector.noalias() = scaling.asDiagonal() * weak * weak.transpose() * scaling.asDiagonal();
  }
  return result;
}

struct MatchedPoint
{
  BievrVoxelKey key;
  const BievrVoxel *voxel;
  Eigen::Vector3d world;
  std::size_t imu_index;
  PoseVector base_jacobian = PoseVector::Zero();
  double base_residual = 0.0;
  bool base_valid = false;
};

struct ScaleStats
{
  std::size_t valid = 0;
  double squared_residual = 0.0;
  double weak_information = 0.0;

  void add(const PoseVector &jacobian, double residual, const PoseMatrix &weak_projector, double huber_delta)
  {
    ++valid;
    squared_residual += residual * residual;
    const double projected = jacobian.dot(weak_projector * jacobian);
    weak_information += precision(residual, huber_delta) * std::max(0.0, projected);
  }

  double rmse() const
  {
    return valid ? std::sqrt(squared_residual / static_cast<double>(valid)) : 0.0;
  }
};

struct VoxelGroup
{
  std::size_t begin;
  std::size_t end;
  ScaleStats base;

  std::size_t size() const { return end - begin; }
  double baseSupport() const { return static_cast<double>(base.valid) / static_cast<double>(size()); }
};

struct Hold
{
  bool coarse = false;
  int frames = 0;
};

struct ScaleCandidate
{
  BievrVoxelKey key;
  std::size_t group;
  ScaleStats stats;
  double merit;
  bool held;
  BievrBumpLevel level;
};

std::size_t coarseBytes(const BievrBumpLevel &level)
{
  return sizeof(BievrBumpLevel) + static_cast<std::size_t>(level.mean_height.size()) * sizeof(float) +
         static_cast<std::size_t>(level.smoothed_height.size()) * sizeof(float) +
         static_cast<std::size_t>(level.effective_weight.size()) * sizeof(double) +
         static_cast<std::size_t>(level.height_variance.size()) * sizeof(double) +
         static_cast<std::size_t>(level.support_count.size()) * sizeof(int);
}
} // namespace

void BievrAdaptiveParams::validate() const
{
  const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
  if (!positive(coarse_pixel_size_m) || !positive(1.0 / coarse_pixel_size_m) ||
      !positive(rotation_reference_m) || !positive(1.0 / rotation_reference_m) ||
      min_effective_points < 1 || !positive(degeneracy_ratio) || degeneracy_ratio >= 1.0 ||
      !positive(min_valid_ratio) || min_valid_ratio > 1.0 || !std::isfinite(switch_margin) || switch_margin < 0.0 ||
      switch_margin >= 1.0 || min_hold_frames < 0 || max_coarse_voxels < 1 || min_voxel_points < 1 ||
      !positive(max_residual_m) || !std::isfinite(max_rmse_ratio) || max_rmse_ratio < 1.0 ||
      !positive(coarse_cost_ratio))
    throw std::invalid_argument("Invalid BIEVR adaptive multiscale parameters");
}

BievrAdaptiveDiagnostics analyzeBievrGeometry(const PoseMatrix &information, std::size_t candidates, std::size_t matches,
                                              const BievrAdaptiveParams &params)
{
  return analyzeGeometry(information, candidates, matches, params).diagnostics;
}

struct BievrAdaptivePolicy::Impl
{
  explicit Impl(const BievrAdaptiveParams &config) : params(config) {}

  const BievrAdaptiveParams params;
  BievrAdaptiveDiagnostics stats;
  std::unordered_map<BievrVoxelKey, BievrBumpLevel, BievrVoxelKeyHash> coarse;
  std::unordered_map<BievrVoxelKey, Hold, BievrVoxelKeyHash> history;
  bool prepared = false;
};

BievrAdaptivePolicy::BievrAdaptivePolicy(const BievrAdaptiveParams &params) : impl_(std::make_unique<Impl>(params))
{
  params.validate();
}

BievrAdaptivePolicy::~BievrAdaptivePolicy() = default;

void BievrAdaptivePolicy::prepare(const BievrVoxelMap &map, const std::vector<Eigen::Vector3d> &imu_points,
                                  const Eigen::Matrix3d &rotation, const Eigen::Vector3d &position, double huber_delta)
{
  auto &state = *impl_;
  const auto &params = state.params;
  state.coarse.clear();
  state.stats = {};
  state.prepared = params.enabled;
  if (!state.prepared) return;

  std::vector<MatchedPoint> matches;
  matches.reserve(imu_points.size());
  PoseMatrix information = PoseMatrix::Zero();
  const Eigen::Matrix3d world_to_imu = rotation.transpose();
  std::size_t base_valid = 0;
  for (std::size_t i = 0; i < imu_points.size(); ++i)
  {
    const Eigen::Vector3d world = rotation * imu_points[i] + position;
    if (!map.canIndex(world)) continue;
    BievrVoxelKey key;
    const auto *voxel = map.queryVoxel(world, &key);
    if (voxel == nullptr) continue;
    auto &match = matches.emplace_back(MatchedPoint{key, voxel, world, i});
    BievrMeasurement measurement;
    if (!map.sampleBase(*voxel, world, measurement) || !validMeasurement(measurement)) continue;
    match.base_jacobian = poseJacobian(imu_points[i], world_to_imu, measurement);
    if (!match.base_jacobian.allFinite()) continue;
    match.base_residual = measurement.residual_;
    match.base_valid = true;
    information.noalias() += precision(match.base_residual, huber_delta) *
                             match.base_jacobian * match.base_jacobian.transpose();
    ++base_valid;
  }

  const auto spectrum = analyzeGeometry(information, imu_points.size(), base_valid, params);
  auto &stats = state.stats;
  stats = spectrum.diagnostics;
  if (base_valid < static_cast<std::size_t>(params.min_effective_points) || !information.allFinite()) return;
  std::sort(matches.begin(), matches.end(), [](const MatchedPoint &a, const MatchedPoint &b) {
    return std::make_tuple(voxelOrder(a.key), a.imu_index) < std::make_tuple(voxelOrder(b.key), b.imu_index);
  });
  std::vector<VoxelGroup> groups;
  for (std::size_t begin = 0; begin < matches.size();)
  {
    const auto key = matches[begin].key;
    std::size_t end = begin;
    ScaleStats base;
    do
    {
      const auto &match = matches[end++];
      if (match.base_valid)
        base.add(match.base_jacobian, match.base_residual, spectrum.weak_projector, huber_delta);
    } while (end < matches.size() && matches[end].key == key);

    const VoxelGroup group{begin, end, base};
    if (group.baseSupport() < kSparseBaseSupport)
    {
      stats.sparse_candidates += group.size();
      stats.sparse_base_valid += base.valid;
    }
    if (group.size() >= static_cast<std::size_t>(params.min_voxel_points))
      groups.push_back(group);
    begin = end;
  }
  stats.sparse_selected_valid = stats.sparse_base_valid;
  if (!params.density_only && stats.weak_rank == 0) return;

  const auto heldCoarse = [&](const BievrVoxelKey &key) {
    const auto prev = state.history.find(key);
    return prev != state.history.end() && prev->second.coarse && prev->second.frames < params.min_hold_frames;
  };
  std::sort(groups.begin(), groups.end(), [&](const VoxelGroup &a, const VoxelGroup &b) {
    const auto &ka = matches[a.begin].key;
    const auto &kb = matches[b.begin].key;
    return std::make_tuple(!heldCoarse(ka), a.baseSupport(), std::numeric_limits<std::size_t>::max() - a.size(), voxelOrder(ka)) <
           std::make_tuple(!heldCoarse(kb), b.baseSupport(), std::numeric_limits<std::size_t>::max() - b.size(), voxelOrder(kb));
  });

  const std::size_t cache_limit = static_cast<std::size_t>(params.max_coarse_voxels);
  const std::size_t evaluation_limit = 2 * cache_limit;
  const double information_floor = 64.0 * std::numeric_limits<double>::epsilon() * information.cwiseAbs().maxCoeff();
  std::vector<ScaleCandidate> selected;
  selected.reserve(std::min(groups.size(), cache_limit));
  const auto better = [](const ScaleCandidate &a, const ScaleCandidate &b) {
    if (a.held != b.held) return a.held;
    if (a.merit != b.merit) return a.merit > b.merit;
    return voxelOrder(a.key) < voxelOrder(b.key);
  };
  for (std::size_t j = 0; j < groups.size() && stats.evaluated_voxels < evaluation_limit; ++j)
  {
    const auto &group = groups[j];
    const auto &first = matches[group.begin];
    const auto prior = state.history.find(first.key);
    if (prior != state.history.end() && !prior->second.coarse && prior->second.frames < params.min_hold_frames)
      continue;
    const bool previously_coarse = prior != state.history.end() && prior->second.coarse;
    const bool held = heldCoarse(first.key);
    ++stats.evaluated_voxels;
    BievrBumpLevel coarse;
    if (!map.buildCoarse(*first.voxel, params.coarse_pixel_size_m, coarse)) continue;

    ScaleStats score;
    std::size_t shared = 0;
    double base_shared_squared = 0.0;
    double coarse_shared_squared = 0.0;
    for (std::size_t i = group.begin; i < group.end; ++i)
    {
      const auto &match = matches[i];
      BievrMeasurement measurement;
      if (!map.sampleCoarse(*match.voxel, coarse, match.world, measurement) || !validMeasurement(measurement)) continue;
      const PoseVector jacobian = poseJacobian(imu_points[match.imu_index], world_to_imu, measurement);
      if (!jacobian.allFinite()) continue;
      score.add(jacobian, measurement.residual_, spectrum.weak_projector, huber_delta);
      if (match.base_valid)
      {
        ++shared;
        base_shared_squared += match.base_residual * match.base_residual;
        coarse_shared_squared += measurement.residual_ * measurement.residual_;
      }
    }

    const double base_ratio = group.baseSupport();
    const double coarse_ratio = static_cast<double>(score.valid) / static_cast<double>(group.size());
    const double base_rmse = group.base.rmse();
    const double coarse_rmse = score.rmse();
    const double rmse_slack = params.max_residual_m * 0.05;
    if (score.valid < group.base.valid || coarse_ratio < params.min_valid_ratio ||
        coarse_rmse > params.max_residual_m ||
        shared < kMinimumRetainedBaseSupport * group.base.valid ||
        (shared > 0 && std::sqrt(coarse_shared_squared / shared) >
                           params.max_rmse_ratio * std::sqrt(base_shared_squared / shared) + rmse_slack))
      continue;

    const double advantage = previously_coarse ? 1.0 : 1.0 + params.switch_margin;
    double merit = 0.0;
    if (params.density_only)
    {
      if (!held && (coarse_ratio < advantage * base_ratio || (!previously_coarse && score.valid <= group.base.valid)))
        continue;
      merit = coarse_ratio - base_ratio;
    }
    else
    {
      const double base_quality = base_ratio / (1.0 + std::pow(base_rmse / params.max_residual_m, 2));
      const double coarse_quality = coarse_ratio / (1.0 + std::pow(coarse_rmse / params.max_residual_m, 2));
      const double base_score = base_quality * group.base.weak_information;
      const double coarse_score = coarse_quality * score.weak_information / params.coarse_cost_ratio;
      const double added_information = score.weak_information - group.base.weak_information;
      if (!std::isfinite(coarse_score) || added_information < 0.0 ||
          (!held && (added_information <= information_floor || coarse_score <= advantage * base_score)))
        continue;
      merit = coarse_score - base_score;
    }
    ScaleCandidate candidate{first.key, j, score, merit, held, std::move(coarse)};
    if (selected.size() < cache_limit)
      selected.push_back(std::move(candidate));
    else
    {
      auto worst = std::max_element(selected.begin(), selected.end(), better);
      if (better(candidate, *worst)) *worst = std::move(candidate);
    }
  }

  for (auto &candidate : selected)
  {
    const auto &group = groups[candidate.group];
    const auto extra_valid = candidate.stats.valid - group.base.valid;
    stats.predicted_valid += extra_valid;
    if (group.baseSupport() < kSparseBaseSupport) stats.sparse_selected_valid += extra_valid;
    stats.weak_information_gain += candidate.stats.weak_information - group.base.weak_information;
    stats.coarse_payload_bytes += coarseBytes(candidate.level);
    state.coarse.emplace(candidate.key, std::move(candidate.level));
  }
  stats.selected_weak_information += stats.weak_information_gain;
  stats.coarse_voxels = state.coarse.size();
  stats.proposed_coarse_voxels = stats.coarse_voxels;
  stats.proposed_coarse_payload_bytes = stats.coarse_payload_bytes;
}

bool BievrAdaptivePolicy::sample(const BievrVoxelMap &map, const Eigen::Vector3d &point, BievrMeasurement &measurement) const
{
  if (!map.canIndex(point)) return false;
  const auto &coarse = impl_->coarse;
  if (coarse.empty()) return map.sample(point, measurement);

  BievrVoxelKey key;
  const auto *voxel = map.queryVoxel(point, &key);
  if (voxel == nullptr) return false;
  const auto match = coarse.find(key);
  return match == coarse.end() ? map.sampleBase(*voxel, point, measurement) :
                                 map.sampleCoarse(*voxel, match->second, point, measurement);
}

const BievrAdaptiveDiagnostics &BievrAdaptivePolicy::diagnostics() const
{
  return impl_->stats;
}

void BievrAdaptivePolicy::useBase()
{
  impl_->coarse.clear();
  auto &stats = impl_->stats;
  stats.predicted_valid = stats.base_valid;
  stats.sparse_selected_valid = stats.sparse_base_valid;
  stats.selected_weak_information = stats.base_weak_information;
  stats.weak_information_gain = 0.0;
  stats.coarse_voxels = 0;
  stats.coarse_payload_bytes = 0;
}

void BievrAdaptivePolicy::finishFrame(bool accepted)
{
  auto &state = *impl_;
  if (!state.prepared) return;
  state.prepared = false;
  if (!accepted)
  {
    state.coarse.clear();
    return;
  }

  const int hold_limit = state.params.min_hold_frames;
  const auto advance = [hold_limit](int frames) { return frames < hold_limit ? frames + 1 : frames; };
  decltype(state.history) next;
  for (const auto &entry : state.coarse)
  {
    const auto prior = state.history.find(entry.first);
    const bool previous_coarse = prior != state.history.end() && prior->second.coarse;
    if (!previous_coarse) ++state.stats.switched_voxels;
    next.emplace(entry.first, Hold{true, previous_coarse ? advance(prior->second.frames) : 1});
  }

  // Recent Base switches retain a bounded cooldown and expired entries are discarded.
  std::vector<std::pair<BievrVoxelKey, int>> base_holds;
  for (const auto &entry : state.history)
  {
    if (state.coarse.find(entry.first) != state.coarse.end()) continue;
    const int frames = entry.second.coarse ? 1 : advance(entry.second.frames);
    if (entry.second.coarse) ++state.stats.switched_voxels;
    if (frames < hold_limit) base_holds.emplace_back(entry.first, frames);
  }
  std::sort(base_holds.begin(), base_holds.end(), [](const auto &a, const auto &b) {
    return std::make_tuple(a.second, voxelOrder(a.first)) < std::make_tuple(b.second, voxelOrder(b.first));
  });
  const auto base_limit = std::min(base_holds.size(), static_cast<std::size_t>(state.params.max_coarse_voxels));
  for (std::size_t i = 0; i < base_limit; ++i)
    next.emplace(base_holds[i].first, Hold{false, base_holds[i].second});
  state.history = std::move(next);
  state.coarse.clear();
}
} // namespace fast_livo::bievr
