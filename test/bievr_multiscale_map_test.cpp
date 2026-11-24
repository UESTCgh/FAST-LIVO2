#include "../src/bievr/bievr_voxel_map.h"

#include <gtest/gtest.h>

#include <limits>

namespace fast_livo::bievr
{
namespace
{
BievrVoxelMap makeMap(bool smooth)
{
  BievrVoxelMap::Parameters parameters;
  parameters.voxel_size_ = 1.0;
  parameters.pixel_size_ = 0.05;
  parameters.smooth_image_ = smooth;
  return BievrVoxelMap(parameters);
}

BievrVoxel makeSparseVoxel()
{
  BievrVoxel voxel;
  voxel.observed_ = true;
  voxel.bump_image_.setZero(13, 13);
  voxel.bump_weights_.setZero(13, 13);
  for (int row = 1; row < 13; row += 2)
  {
    for (int column = 1; column < 13; column += 2)
    {
      voxel.bump_weights_(row, column) = 1.0F;
      voxel.bump_image_(row, column) = 0.2F * (0.05F * column) + 0.1F * (0.05F * row) + 0.015F;
    }
  }
  voxel.bump_image_smoothed_ = voxel.bump_image_;
  return voxel;
}

TEST(BievrMultiscaleMap, AggregatesWeightedObservationsWithoutFillingHoles)
{
  const auto map = makeMap(true);
  auto voxel = makeSparseVoxel();
  voxel.bump_weights_(8, 8) = 3.0F;
  voxel.bump_image_(8, 8) = 0.22F;
  BievrBumpLevel level;

  EXPECT_FALSE(map.buildCoarse(voxel, 0.05, level));
  ASSERT_TRUE(map.buildCoarse(voxel, 0.10, level));
  EXPECT_EQ(level.observed_pixels, 36U);
  EXPECT_NEAR(level.coverage(), 36.0 / 49.0, 1e-12);
  EXPECT_EQ(level.support_count.sum(), 37);
  EXPECT_DOUBLE_EQ(level.effective_weight.sum(), 39.0);
  EXPECT_EQ(level.support_count(4, 4), 2);
  EXPECT_DOUBLE_EQ(level.effective_weight(4, 4), 4.0);
  EXPECT_NEAR(level.mean_height(4, 4), (0.12 + 3.0 * 0.22) / 4.0, 1e-6);
  EXPECT_NEAR(level.height_variance(4, 4), 0.001875, 1e-8);
  EXPECT_GT(std::abs(level.smoothed_height(4, 4) - level.mean_height(4, 4)), 1e-3);
  EXPECT_TRUE(level.smoothed_height.row(0).isZero());
  EXPECT_TRUE(level.smoothed_height.col(0).isZero());

  voxel.bump_weights_.setZero();
  EXPECT_FALSE(map.buildCoarse(voxel, 0.10, level));
  EXPECT_EQ(level.observed_pixels, 0U);
  EXPECT_DOUBLE_EQ(level.coverage(), 0.0);
}

TEST(BievrMultiscaleMap, RecoversSparseSupportAndTransformsJacobian)
{
  const auto map = makeMap(false);
  auto voxel = makeSparseVoxel();
  const Eigen::Vector3d query(0.4, 0.4, 0.125);
  BievrBumpLevel level;
  BievrMeasurement measurement;

  ASSERT_TRUE(map.buildCoarse(voxel, 0.10, level));
  EXPECT_FALSE(map.sampleBase(voxel, query, measurement));
  EXPECT_TRUE(map.sampleCoarse(voxel, level, query, measurement));
  EXPECT_FALSE(map.sampleCoarse(voxel, level, Eigen::Vector3d(-0.001, 0.4, 0.125), measurement));
  EXPECT_FALSE(map.sampleCoarse(voxel, level, Eigen::Vector3d(0.65, 0.4, 0.125), measurement));
  EXPECT_FALSE(map.sampleCoarse(voxel, level,
                                Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.4, 0.125), measurement));

  voxel.cell_from_world_.linear() =
      (Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  voxel.cell_from_world_.translation() = Eigen::Vector3d(0.3, -0.25, 0.5);
  ASSERT_TRUE(map.buildCoarse(voxel, 0.10, level));
  const Eigen::Vector3d inside(0.405, 0.405, 0.125);
  ASSERT_TRUE(map.sampleCoarse(voxel, level, voxel.cell_from_world_.inverse() * inside, measurement));
  EXPECT_NEAR(measurement.residual_, 0.125 - 0.405 * 0.3, 1e-6);
  const Eigen::RowVector3d expected = Eigen::RowVector3d(-0.2, -0.1, 1.0) * voxel.cell_from_world_.linear();
  EXPECT_TRUE(measurement.position_jacobian_.isApprox(expected, 1e-6));
}

TEST(BievrMultiscaleMap, PassesThroughLegacyBaseAndReportsFallbackKey)
{
  auto map = makeMap(false);
  pcl::PointCloud<pcl::PointXYZINormal> points;
  for (double x : {0.85, 0.90, 0.95})
  {
    for (double y : {0.2, 0.25, 0.3})
    {
      pcl::PointXYZINormal point;
      point.x = x;
      point.y = y;
      point.z = 0.3F;
      points.push_back(point);
    }
  }
  map.update(points);

  BievrVoxelKey key;
  const auto *fallback = map.queryVoxel(Eigen::Vector3d(1.001, 0.23, 0.3), &key);
  ASSERT_NE(fallback, nullptr);
  EXPECT_EQ(key, (BievrVoxelKey{0, 0, 0}));
  EXPECT_EQ(map.findVoxel(key), fallback);

  const Eigen::Vector3d center(0.90, 0.23, 0.3);
  BievrMeasurement old_api, new_api;
  const auto *direct = map.queryVoxel(center);
  ASSERT_NE(direct, nullptr);
  ASSERT_TRUE(map.sample(center, old_api));
  ASSERT_TRUE(map.sampleBase(*direct, center, new_api));
  EXPECT_EQ(old_api.residual_, new_api.residual_);
  EXPECT_EQ(old_api.position_jacobian_, new_api.position_jacobian_);
}

TEST(BievrMultiscaleMap, RebuildsFromReprojectedBaseAfterNormalChange)
{
  auto map = makeMap(false);
  pcl::PointCloud<pcl::PointXYZINormal> plane, tilted;
  for (int row = 1; row < 19; ++row)
  {
    for (int column = 1; column < 19; ++column)
    {
      pcl::PointXYZINormal point;
      point.x = 0.05F * column;
      point.y = 0.05F * row;
      point.z = 0.4F;
      plane.push_back(point);
      point.z += 0.3F * (point.x - 0.5F);
      tilted.push_back(point);
    }
  }
  const BievrVoxelKey key{0, 0, 0};
  map.update(plane);
  const auto *voxel = map.findVoxel(key);
  ASSERT_NE(voxel, nullptr);
  const Eigen::Matrix3d old_rotation = voxel->cell_from_world_.linear();
  BievrBumpLevel level;
  ASSERT_TRUE(map.buildCoarse(*voxel, 0.10, level));
  EXPECT_NEAR(level.effective_weight.sum(), voxel->bump_weights_.cast<double>().sum(), 1e-9);

  for (int scan = 0; scan < 4; ++scan) map.update(tilted);
  voxel = map.findVoxel(key);
  ASSERT_NE(voxel, nullptr);
  EXPECT_FALSE(voxel->cell_from_world_.linear().isApprox(old_rotation, 1e-4));
  ASSERT_TRUE(map.buildCoarse(*voxel, 0.10, level));
  EXPECT_NEAR(level.effective_weight.sum(), voxel->bump_weights_.cast<double>().sum(), 1e-9);
  EXPECT_TRUE(level.mean_height.allFinite());
  EXPECT_TRUE(level.height_variance.allFinite());
  BievrMeasurement measurement;
  EXPECT_TRUE(map.sampleCoarse(*voxel, level, Eigen::Vector3d(0.5, 0.5, 0.4), measurement));
}
} // namespace
} // namespace fast_livo::bievr

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
