#include "bievr_registration.h"
#include "bievr_adaptive.h"
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
constexpr double kIntensityHuberMeters = 0.05;

double huberCost(double residual, double delta)
{
  const double magnitude = std::abs(residual);
  return magnitude <= delta ? 0.5 * magnitude * magnitude : delta * (magnitude - 0.5 * delta);
}

M3D rotationInformationInverse(const PoseMatrix &information)
{
  const Eigen::SelfAdjointEigenSolver<M3D> solver(information.topLeftCorner<3, 3>());
  if (solver.info() != Eigen::Success) return M3D::Zero();
  const double tolerance = std::max(1e-9, 1e-9 * solver.eigenvalues().maxCoeff());
  const V3D inverse = solver.eigenvalues().unaryExpr([tolerance](double value) {
    return value > tolerance ? 1.0 / value : 0.0;
  });
  return solver.eigenvectors() * inverse.asDiagonal() * solver.eigenvectors().transpose();
}

M3D translationInformation(const PoseMatrix &information, const M3D &rotation_inverse)
{
  const M3D translation = information.bottomRightCorner<3, 3>() -
      information.bottomLeftCorner<3, 3>() * rotation_inverse * information.topRightCorner<3, 3>();
  return 0.5 * (translation + translation.transpose());
}

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

bool improvesIntensity(const fast_livo::BievrRegistration &registration,
                       const std::vector<BievrLinearization> &rows, double scale,
                       const StatesGroup &geometry, const StatesGroup &joint, int min_matches)
{
  const auto &points = registration.intensityPoints();
  double before = 0.0, after = 0.0;
  int shared = 0, support = 0;
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    if (!rows[i].valid) continue;
    ++support;
    V3D gradient;
    double geometry_residual, joint_residual;
    if (!registration.sampleIntensity(i, geometry.rot_end * points[i] + geometry.pos_end, gradient, geometry_residual))
      continue;
    const double geometry_cost = huberCost(scale * geometry_residual, kIntensityHuberMeters);
    before += geometry_cost;
    if (registration.sampleIntensity(i, joint.rot_end * points[i] + joint.pos_end, gradient, joint_residual))
    {
      ++shared;
      after += huberCost(scale * joint_residual, kIntensityHuberMeters);
    }
    else
    {
      after += std::max(geometry_cost, huberCost(0.15, kIntensityHuberMeters));
    }
  }
  return shared >= min_matches && shared >= 0.8 * support && after < before;
}
}  // namespace

VoxelMapManager::VoxelMapManager(const VoxelMapConfig &config, const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
    : config_setting_(config), voxel_map_(voxel_map), feats_undistort_(new PointCloudXYZI),
      feats_down_body_(new PointCloudXYZI), feats_down_world_(new PointCloudXYZI)
{
  if (config.use_bievr_)
  {
    if (!std::isfinite(config.bievr_huber_delta_) || config.bievr_huber_delta_ <= 0.0 || config.max_iterations_ <= 0)
      throw std::invalid_argument("BIEVR requires a positive finite Huber delta and a positive iteration limit");
    if (config.bievr_intensity_en_ &&
        (!(config.bievr_intensity_scale_ > 0.0) || !std::isfinite(config.bievr_intensity_scale_) ||
         !(config.bievr_intensity_weight_ > 0.0 && config.bievr_intensity_weight_ <= 1.0) ||
         !(config.bievr_intensity_weak_ratio_ > 0.0 && config.bievr_intensity_weak_ratio_ <= 1.0) ||
         config.bievr_intensity_min_matches_ <= 0))
      throw std::invalid_argument("Invalid BIEVR intensity scale, weight, weak-axis threshold or support");
    bievr_registration_ = std::make_unique<fast_livo::BievrRegistration>(
        config.bievr_voxel_size_, config.bievr_pixel_size_, config.bievr_max_voxels_,
        config.bievr_fine_voxel_size_, config.bievr_informed_voxels_, config.bievr_intensity_en_,
        config.bievr_intensity_sampling_, config.bievr_adaptive_);
    if (config.bievr_adaptive_.log_diagnostics) config.bievr_adaptive_.validate();
  }
}

VoxelMapManager::~VoxelMapManager() = default;

void VoxelMapManager::UpdateBievrMap(const StatesGroup &state)
{
  bievr_registration_->updateMap(*feats_undistort_, state.rot_end, state.pos_end, extR_, extT_);
}

bool VoxelMapManager::UpdateBievrState(const StatesGroup &prediction, int min_measurements,
                                       double mean_abs_limit, double rmse_limit)
{
  lio_quality_ = LioQuality{};
  bievr_effective_points_.clear();
  if (!prediction.cov.allFinite() || !prediction.rot_end.allFinite() || !prediction.pos_end.allFinite()) return false;

  bievr_registration_->selectPoints(*feats_undistort_, prediction.rot_end, prediction.pos_end, extR_, extT_);
  const auto &params = config_setting_.bievr_adaptive_;
  if (!params.enabled) return SolveBievrState(prediction, min_measurements, false, mean_abs_limit, rmse_limit);

  bievr_registration_->prepareAdaptive(prediction.rot_end, prediction.pos_end, config_setting_.bievr_huber_delta_);
  const auto &adaptive = bievr_registration_->adaptiveDiagnostics();
  lio_quality_.adaptive = adaptive;
  const bool use_coarse = adaptive.coarse_voxels > 0;
  bool accepted = SolveBievrState(prediction, min_measurements, use_coarse, mean_abs_limit, rmse_limit);
  if (use_coarse && !accepted)
  {
    // A different scale requires a new solve from the original prediction.
    bievr_registration_->discardAdaptive();
    lio_quality_ = LioQuality{};
    lio_quality_.adaptive = bievr_registration_->adaptiveDiagnostics();
    lio_quality_.adaptive_fallback = true;
    accepted = SolveBievrState(prediction, min_measurements, false, mean_abs_limit, rmse_limit);
  }
  bievr_registration_->finishAdaptive(accepted && lio_quality_.mean_abs_res <= mean_abs_limit &&
                                      lio_quality_.rmse_res <= rmse_limit);
  lio_quality_.adaptive = adaptive;
  return accepted;
}

bool VoxelMapManager::SolveBievrState(const StatesGroup &prediction, int min_measurements, bool use_adaptive,
                                      double mean_abs_limit, double rmse_limit)
{
  const auto &imu_points = bievr_registration_->imuPoints();
  const auto &intensity_points = bievr_registration_->intensityPoints();
  std::vector<BievrLinearization> rows(imu_points.size());
  std::vector<BievrLinearization> intensity_rows(intensity_points.size());
  lio_quality_.intensity_candidates = intensity_points.size();
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
      row.valid = use_adaptive ? bievr_registration_->sampleAdaptive(world_point, gradient, row.residual)
                               : bievr_registration_->sample(world_point, gradient, row.residual);
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
    if (iteration == 0 && config_setting_.bievr_adaptive_.log_diagnostics && !config_setting_.bievr_adaptive_.enabled)
      q.adaptive = fast_livo::bievr::analyzeBievrGeometry(info, imu_points.size(), count, config_setting_.bievr_adaptive_);
    if (count < std::max(1, min_measurements) || !info.allFinite() || !rhs.allFinite()) return false;
    if (use_adaptive && count < 0.8 * std::min(q.adaptive.base_valid, q.adaptive.predicted_valid)) return false;

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
    StateVector step = posterior_factor.solve(state_rhs);
    if (!step.allFinite()) return false;

    if (config_setting_.bievr_intensity_en_ &&
        intensity_points.size() >= static_cast<std::size_t>(config_setting_.bievr_intensity_min_matches_))
    {
      const M3D geometry_translation = translationInformation(info, rotationInformationInverse(info));
      const Eigen::SelfAdjointEigenSolver<M3D> translation_spectrum(geometry_translation);
      if (translation_spectrum.info() == Eigen::Success && translation_spectrum.eigenvalues().maxCoeff() > 0.0)
      {
        const double weak_information = std::max(0.0, translation_spectrum.eigenvalues()[0]);
        q.translation_eigen_ratio = weak_information / translation_spectrum.eigenvalues()[2];
        if (q.translation_eigen_ratio < config_setting_.bievr_intensity_weak_ratio_)
        {
          const V3D axis = translation_spectrum.eigenvectors().col(0);
          const double scale = config_setting_.bievr_intensity_scale_;
#ifdef MP_EN
#pragma omp parallel for num_threads(MP_PROC_NUM)
#endif
          for (std::size_t i = 0; i < intensity_points.size(); ++i)
          {
            auto &row = intensity_rows[i];
            const V3D &p = intensity_points[i];
            V3D gradient;
            row.valid = bievr_registration_->sampleIntensity(i, estimate.rot_end * p + estimate.pos_end,
                                                             gradient, row.residual);
            if (!row.valid) continue;
            gradient *= scale;
            row.residual *= scale;
            row.jacobian.head<3>() = p.cross(world_to_imu * gradient);
            row.jacobian.tail<3>() = gradient;
            row.valid = row.jacobian.allFinite() && std::isfinite(row.residual);
          }

          PoseMatrix intensity_information = PoseMatrix::Zero();
          PoseVector intensity_rhs = PoseVector::Zero();
          int matches = 0;
          double squared_error = 0.0;
          for (const auto &row : intensity_rows)
          {
            if (!row.valid) continue;
            const double weight = kIntensityHuberMeters / std::max(kIntensityHuberMeters, std::abs(row.residual));
            const double precision = weight / kMeasurementVariance;
            intensity_information.noalias() += precision * row.jacobian * row.jacobian.transpose();
            intensity_rhs.noalias() -= precision * row.jacobian * row.residual;
            squared_error += row.residual * row.residual;
            ++matches;
          }

          q.intensity_matches = matches;
          q.intensity_photo_rmse = matches ? std::sqrt(squared_error / matches) / scale : 0.0;
          if (matches >= config_setting_.bievr_intensity_min_matches_ && intensity_information.allFinite() &&
              intensity_rhs.allFinite())
          {
            // Marginalize rotation before reducing the intensity factor to the weak translation mode.
            const M3D rotation_inverse = rotationInformationInverse(intensity_information);
            const M3D intensity_translation = translationInformation(intensity_information, rotation_inverse);
            const V3D translation_rhs = intensity_rhs.tail<3>() -
                intensity_information.bottomLeftCorner<3, 3>() * rotation_inverse * intensity_rhs.head<3>();
            const double axis_information = axis.dot(intensity_translation * axis);
            if (axis_information > std::max(1e-6, 0.1 * weak_information))
            {
              const double weight = config_setting_.bievr_intensity_weight_ *
                  std::min(1.0, 4.0 * std::max(1e-6, weak_information) / axis_information);
              StateVector projection = StateVector::Zero();
              projection.segment<3>(3) = axis;
              StateMatrix joint_information = posterior_information;
              joint_information.noalias() += (weight * axis_information) * projection * projection.transpose();
              Eigen::LDLT<StateMatrix> joint_factor(joint_information);
              if (joint_factor.info() == Eigen::Success && (joint_factor.vectorD().array() > 0.0).all())
              {
                const StateVector joint_step = joint_factor.solve(state_rhs + weight * axis.dot(translation_rhs) * projection);
                if (joint_step.allFinite())
                {
                  StatesGroup geometry_state = estimate;
                  geometry_state += step;
                  StatesGroup joint_state = estimate;
                  joint_state += joint_step;
                  if (improvesIntensity(*bievr_registration_, intensity_rows, scale, geometry_state, joint_state,
                                        config_setting_.bievr_intensity_min_matches_))
                  {
                    step = joint_step;
                    posterior_factor = joint_factor;
                    q.intensity_axis_information = axis_information;
                    q.intensity_applied_weight = weight;
                    ++q.intensity_steps;
                  }
                }
              }
            }
          }
        }
      }
    }

    estimate += step;
    last_rotation_step = step.head<3>();

    if (last_rotation_step.norm() < DEG2RAD(0.01) && step.segment<3>(3).norm() < 0.00015 && ++convergence_count == 2) break;
  }

  if (use_adaptive && !(lio_quality_.mean_abs_res <= mean_abs_limit && lio_quality_.rmse_res <= rmse_limit)) return false;
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
