#include "bievr_registration.h"
#include "voxel_map.h"

#include <Eigen/Cholesky>
#include <cmath>

namespace
{
using PoseMatrix = Eigen::Matrix<double, 6, 6>;
using PoseVector = Eigen::Matrix<double, 6, 1>;
using StateMatrix = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;
using StateVector = Eigen::Matrix<double, DIM_STATE, 1>;

constexpr double kMeasurementVariance = 0.001;  // m², BIEVR-LIO-ESIKF

M3D rightJacobian(const V3D &rotation)
{
  const double angle_squared = rotation.squaredNorm();
  M3D skew;
  skew << SKEW_SYM_MATRX(rotation);
  if (angle_squared < 1e-8)
    return M3D::Identity() - 0.5 * skew + skew * skew / 6.0;

  const double angle = std::sqrt(angle_squared);
  return M3D::Identity() - ((1.0 - std::cos(angle)) / angle_squared) * skew +
         ((angle - std::sin(angle)) / (angle_squared * angle)) * (skew * skew);
}

struct BievrLinearization
{
  PoseVector jacobian;
  double residual = 0.0;
  bool valid = false;
};
}  // namespace

VoxelMapManager::VoxelMapManager(const VoxelMapConfig &config, const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
    : config_setting_(config), voxel_map_(voxel_map), feats_undistort_(new PointCloudXYZI),
      feats_down_body_(new PointCloudXYZI), feats_down_world_(new PointCloudXYZI)
{
  if (config.use_bievr_)
  {
    if (!std::isfinite(config.bievr_huber_delta_) || config.bievr_huber_delta_ <= 0.0 || config.max_iterations_ <= 0)
      throw std::invalid_argument("BIEVR requires a positive finite Huber delta and a positive iteration limit");
    bievr_registration_ = std::make_unique<fast_livo::BievrRegistration>(
        config.bievr_voxel_size_, config.bievr_pixel_size_, config.bievr_max_voxels_,
        config.bievr_fine_voxel_size_, config.bievr_informed_voxels_);
  }
}

VoxelMapManager::~VoxelMapManager() = default;

void VoxelMapManager::UpdateBievrMap(const StatesGroup &state)
{
  bievr_registration_->updateMap(*feats_undistort_, state.rot_end, state.pos_end, extR_, extT_);
}

bool VoxelMapManager::UpdateBievrState(const StatesGroup &prediction, int min_measurements)
{
  lio_quality_ = LioQuality{};
  bievr_effective_points_.clear();
  if (!prediction.cov.allFinite() || !prediction.rot_end.allFinite() || !prediction.pos_end.allFinite()) return false;

  bievr_registration_->selectPoints(*feats_undistort_, prediction.rot_end, prediction.pos_end, extR_, extT_);
  const auto &imu_points = bievr_registration_->imuPoints();
  std::vector<BievrLinearization> rows(imu_points.size());
  StatesGroup estimate = prediction;
  Eigen::LDLT<StateMatrix> posterior_factor;
  V3D last_rotation_step = V3D::Zero();
  int convergence_count = 0;

  for (int iteration = 0; iteration < config_setting_.max_iterations_; ++iteration)
  {
    const M3D world_to_imu = estimate.rot_end.transpose();
#ifdef MP_EN
#pragma omp parallel for num_threads(MP_PROC_NUM)
#endif
    for (std::size_t i = 0; i < imu_points.size(); ++i)
    {
      const V3D &point = imu_points[i];
      const V3D world_point = estimate.rot_end * point + estimate.pos_end;
      V3D gradient;
      auto &row = rows[i];
      row.valid = bievr_registration_->sample(world_point, gradient, row.residual);
      if (row.valid)
      {
        row.jacobian.head<3>() = point.cross(world_to_imu * gradient);
        row.jacobian.tail<3>() = gradient;
        row.valid = row.jacobian.allFinite() && std::isfinite(row.residual);
      }
    }

    PoseMatrix info = PoseMatrix::Zero();
    PoseVector rhs = PoseVector::Zero();
    double sum_abs = 0.0, sum_sq = 0.0, max_abs = 0.0;
    int count = 0;
    for (const auto &row : rows)
    {
      if (!row.valid) continue;
      const double abs_residual = std::abs(row.residual);
      const double weight = abs_residual > config_setting_.bievr_huber_delta_ ?
                            config_setting_.bievr_huber_delta_ / abs_residual : 1.0;
      const double precision = weight / kMeasurementVariance;
      info.noalias() += precision * row.jacobian * row.jacobian.transpose();
      rhs.noalias() -= precision * row.jacobian * row.residual;
      sum_abs += abs_residual;
      sum_sq += row.residual * row.residual;
      max_abs = std::max(max_abs, abs_residual);
      ++count;
    }

    auto &q = lio_quality_;
    q.eff = count;
    q.iters = iteration + 1;
    q.mean_abs_res = count ? sum_abs / count : 0.0;
    q.rmse_res = count ? std::sqrt(sum_sq / count) : 0.0;
    q.max_abs_res = max_abs;
    if (count < std::max(1, min_measurements) || !info.allFinite() || !rhs.allFinite()) return false;

    const Eigen::SelfAdjointEigenSolver<PoseMatrix> eigensolver(info);
    if (eigensolver.info() != Eigen::Success) return false;
    q.min_eig = eigensolver.eigenvalues()(0);
    q.max_eig = eigensolver.eigenvalues()(5);
    q.cond = q.min_eig > 1e-12 ? q.max_eig / q.min_eig : 1e18;
    q.degenerate = q.min_eig < 1e-6 || q.cond > 1e10;
    for (int i = 0; i < 6; ++i)
    {
      q.eigs[i] = eigensolver.eigenvalues()(i);
      q.min_evec[i] = eigensolver.eigenvectors()(i, 0);
    }

    // Move the IMU prior into the right SO(3) tangent space at this iterate.
    StateVector delta = estimate - prediction;
    StateMatrix transport = StateMatrix::Identity();
    transport.topLeftCorner<3, 3>() = rightJacobian(delta.head<3>());
    const StateMatrix prior = transport * prediction.cov * transport.transpose();
    const Eigen::LDLT<StateMatrix> prior_factor(prior);
    if (prior_factor.info() != Eigen::Success || (prior_factor.vectorD().array() <= 0.0).any()) return false;

    StateMatrix posterior_information = prior_factor.solve(StateMatrix::Identity());
    posterior_information.topLeftCorner<6, 6>() += info;
    posterior_factor.compute(posterior_information);
    if (posterior_factor.info() != Eigen::Success || (posterior_factor.vectorD().array() <= 0.0).any()) return false;

    delta.head<3>() = (transport.topLeftCorner<3, 3>() * delta.head<3>()).eval();
    StateVector state_rhs = -prior_factor.solve(delta);
    state_rhs.head<6>() += rhs;
    const StateVector step = posterior_factor.solve(state_rhs);
    if (!step.allFinite()) return false;
    estimate += step;
    last_rotation_step = step.head<3>();

    if (last_rotation_step.norm() < DEG2RAD(0.01) && step.segment<3>(3).norm() < 0.00015 && ++convergence_count == 2) break;
  }

  StateMatrix transport = StateMatrix::Identity();
  transport.topLeftCorner<3, 3>() = rightJacobian(last_rotation_step);
  const StateMatrix covariance = transport * posterior_factor.solve(StateMatrix::Identity()) * transport.transpose();
  if (!covariance.allFinite() || !estimate.rot_end.allFinite() || !estimate.pos_end.allFinite()) return false;
  estimate.cov = 0.5 * (covariance + covariance.transpose());
  state_ = estimate;

  bievr_effective_points_.reserve(lio_quality_.eff);
  for (std::size_t i = 0; i < imu_points.size(); ++i)
  {
    if (rows[i].valid) bievr_effective_points_.push_back(estimate.rot_end * imu_points[i] + estimate.pos_end);
  }
  return true;
}

PointCloudXYZI VoxelMapManager::EffectiveWorldPoints() const
{
  PointCloudXYZI cloud;
  cloud.resize(config_setting_.use_bievr_ ? bievr_effective_points_.size() : ptpl_list_.size());
  for (std::size_t i = 0; i < cloud.size(); ++i)
  {
    const V3D &point = config_setting_.use_bievr_ ? bievr_effective_points_[i] : ptpl_list_[i].point_w_;
    cloud[i].x = point.x();
    cloud[i].y = point.y();
    cloud[i].z = point.z();
  }
  return cloud;
}
