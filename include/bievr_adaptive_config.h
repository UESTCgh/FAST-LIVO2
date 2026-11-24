#pragma once

#include <array>
#include <cstddef>

namespace fast_livo::bievr
{
struct BievrAdaptiveParams
{
  bool enabled = false;
  bool density_only = false;
  bool log_diagnostics = false;
  double coarse_pixel_size_m = 0.10;
  double rotation_reference_m = 5.0;
  int min_effective_points = 100;
  double degeneracy_ratio = 0.02;
  double min_valid_ratio = 0.30;
  double switch_margin = 0.15;
  int min_hold_frames = 3;
  int max_coarse_voxels = 64;
  int min_voxel_points = 6;
  double max_residual_m = 0.12;
  double max_rmse_ratio = 1.10;
  double coarse_cost_ratio = 1.0;

  void validate() const;
};

struct BievrAdaptiveDiagnostics
{
  std::size_t candidates = 0;
  std::size_t base_valid = 0;
  std::size_t predicted_valid = 0;
  std::size_t sparse_candidates = 0;
  std::size_t sparse_base_valid = 0;
  std::size_t sparse_selected_valid = 0;
  std::size_t evaluated_voxels = 0;
  std::size_t coarse_voxels = 0;
  std::size_t proposed_coarse_voxels = 0;
  std::size_t switched_voxels = 0;
  std::size_t coarse_payload_bytes = 0;
  std::size_t proposed_coarse_payload_bytes = 0;
  int weak_rank = 0;
  double base_min_eigenvalue = 0.0;
  double base_eigen_ratio = 0.0;
  double base_weak_information = 0.0;
  double selected_weak_information = 0.0;
  double weak_information_gain = 0.0;
  std::array<double, 6> base_eigenvalues{};
};
} // namespace fast_livo::bievr
