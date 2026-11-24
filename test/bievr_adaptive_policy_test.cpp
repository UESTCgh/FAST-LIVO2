#include "../src/bievr/bievr_adaptive.h"
#include "../src/bievr/bievr_voxel_map.h"

#include <algorithm>
#include <gtest/gtest.h>

namespace fast_livo::bievr
{
namespace
{
using PoseMatrix = Eigen::Matrix<double, 6, 6>;
using Points = std::vector<Eigen::Vector3d>;

class BievrAdaptivePolicyTest : public testing::Test
{
protected:
  void SetUp() override
  {
    BievrVoxelMap::Parameters map_params;
    map_params.voxel_size_ = 1.0;
    map.configure(map_params);
    pcl::PointCloud<pcl::PointXYZINormal> training;
    for (int cell = 0; cell < 4; ++cell)
    {
      for (int row = 0; row < 7; ++row)
        for (int col = 0; col < 7; ++col)
        {
          pcl::PointXYZINormal point;
          point.x = cell + 0.05 + col * 0.15;
          point.y = 0.05 + row * 0.15;
          point.z = 0.25;
          training.push_back(point);
        }
      for (int row = 0; row < 15; ++row)
        for (int col = 0; col < 15; ++col)
          scan.emplace_back(cell + 0.12 + col * 0.05, 0.12 + row * 0.05, 0.25);
    }
    map.update(training, nullptr);

    params.enabled = true;
    params.max_coarse_voxels = 1;
  }

  void prepare(BievrAdaptivePolicy &policy, const Points &points) const
  {
    policy.prepare(map, points, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), 0.1);
  }

  BievrAdaptiveParams params;
  BievrVoxelMap map;
  Points scan;
};

TEST(BievrAdaptiveSpectrum, NormalizesRotationAndNeedsMinimumSupport)
{
  BievrAdaptiveParams params;
  params.rotation_reference_m = 5.0;
  PoseMatrix information = PoseMatrix::Identity();
  information.topLeftCorner<3, 3>() *= 25.0;
  information(3, 3) = 0.01;
  const auto supported = analyzeBievrGeometry(information, 120, 110, params);
  EXPECT_EQ(supported.candidates, 120u);
  EXPECT_EQ(supported.base_valid, 110u);
  EXPECT_EQ(supported.weak_rank, 1);
  EXPECT_NEAR(supported.base_min_eigenvalue, 0.01, 1e-12);
  EXPECT_NEAR(supported.base_eigen_ratio, 0.01, 1e-12);
  EXPECT_NEAR(supported.base_weak_information, 0.01, 1e-12);

  const auto unsupported = analyzeBievrGeometry(information, 120, 55, params);
  EXPECT_EQ(unsupported.base_valid, 55u);
  EXPECT_EQ(unsupported.weak_rank, 0);

  params.rotation_reference_m = 0.0;
  EXPECT_THROW(params.validate(), std::invalid_argument);
}

TEST_F(BievrAdaptivePolicyTest, DensitySelectionIsDeterministicBoundedAndRevertible)
{
  params.density_only = true;
  BievrAdaptivePolicy policy(params);
  BievrAdaptivePolicy reversed_policy(params);
  prepare(policy, scan);
  prepare(reversed_policy, Points(scan.rbegin(), scan.rend()));
  const auto planned = policy.diagnostics();
  ASSERT_EQ(planned.coarse_voxels, 1u);
  EXPECT_LE(planned.evaluated_voxels, 2u);
  EXPECT_GT(planned.coarse_payload_bytes, 0u);
  EXPECT_GT(planned.predicted_valid, planned.base_valid);
  EXPECT_GT(planned.sparse_selected_valid, planned.sparse_base_valid);

  std::size_t valid = 0;
  for (const auto &point : scan)
  {
    BievrMeasurement forward;
    BievrMeasurement reversed;
    const bool sampled = policy.sample(map, point, forward);
    ASSERT_EQ(sampled, reversed_policy.sample(map, point, reversed));
    if (sampled)
    {
      ++valid;
      EXPECT_DOUBLE_EQ(forward.residual_, reversed.residual_);
      EXPECT_TRUE(forward.position_jacobian_.isApprox(reversed.position_jacobian_, 1e-12));
    }
  }
  EXPECT_EQ(valid, planned.predicted_valid);

  policy.useBase();
  EXPECT_EQ(policy.diagnostics().coarse_voxels, 0u);
  EXPECT_EQ(policy.diagnostics().proposed_coarse_voxels, 1u);
  EXPECT_EQ(policy.diagnostics().predicted_valid, planned.base_valid);
  policy.finishFrame(true);
  EXPECT_EQ(policy.diagnostics().switched_voxels, 0u);

  prepare(policy, scan);
  ASSERT_EQ(policy.diagnostics().coarse_voxels, 1u);
  policy.finishFrame(true);
  EXPECT_EQ(policy.diagnostics().switched_voxels, 1u);
  prepare(policy, scan);
  ASSERT_EQ(policy.diagnostics().coarse_voxels, 1u);
  policy.finishFrame(true);
  EXPECT_EQ(policy.diagnostics().switched_voxels, 0u);
}

TEST_F(BievrAdaptivePolicyTest, MorePlaneMatchesDoNotCreateWeakDirectionInformation)
{
  // Isolate the exact nullspace from observable tilt directions.
  params.degeneracy_ratio = 1e-8;
  BievrAdaptivePolicy policy(params);
  prepare(policy, scan);
  const auto &result = policy.diagnostics();
  ASSERT_EQ(result.weak_rank, 3);
  ASSERT_GT(result.base_valid, 0u);
  ASSERT_GT(result.evaluated_voxels, 0u);
  EXPECT_EQ(result.coarse_voxels, 0u);
  EXPECT_EQ(result.predicted_valid, result.base_valid);
  EXPECT_DOUBLE_EQ(result.weak_information_gain, 0.0);
}
} // namespace
} // namespace fast_livo::bievr

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
