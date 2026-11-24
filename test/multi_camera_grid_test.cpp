#include "vio.h"

#include <gtest/gtest.h>

TEST(MultiCameraGrid, IncludesPixelsInPartialCells)
{
  vk::PinholeCamera camera(1224, 1024, 1.0, 1174.4, 1174.5, 602.1, 506.2);
  VIOManager manager;
  manager.cam = &camera;
  std::vector<double> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::vector<double> translation{0, 0, 0};
  manager.setImuToLidarExtrinsic(V3D::Zero(), M3D::Identity());
  manager.setLidarToCameraExtrinsic(rotation, translation);
  manager.grid_size = 5;
  manager.grid_n_height = 3;
  manager.raycast_en = false;
  manager.patch_size = 8;
  manager.patch_pyrimid_level = 4;
  manager.initializeVIO(true);
  manager.resetGrid();

  const int last_col = (manager.width - 1) / manager.grid_size;
  const int last_row = (manager.height - 1) / manager.grid_size;
  EXPECT_LT(last_col, manager.grid_n_width);
  EXPECT_LT(last_row, manager.grid_n_height);
  EXPECT_LT(static_cast<std::size_t>(last_row * manager.grid_n_width + last_col), manager.append_voxel_points.size());
}

TEST(MultiCameraProjection, RobustJacobianMatchesDistortedProjection)
{
  vk::PinholeCamera camera(1224, 1024, 1.0, 1174.4, 1174.5, 602.1, 506.2,
                           -0.05808, 0.13450, 0.0000131, 0.0006217, -0.06735);
  VIOManager manager;
  manager.cam = &camera;
  manager.fx = camera.fx();
  manager.fy = camera.fy();
  const V3D point(1.64, -1.28, 4.0);

  MD(2, 3) legacy_jacobian;
  manager.computeProjectionJacobian(point, legacy_jacobian);
  EXPECT_DOUBLE_EQ(legacy_jacobian(0, 0), camera.fx() / point.z());
  EXPECT_DOUBLE_EQ(legacy_jacobian(0, 1), 0.0);

  manager.setRobustVisualOptions(8.0);
  MD(2, 3) robust_jacobian;
  manager.computeProjectionJacobian(point, robust_jacobian);
  EXPECT_GT((robust_jacobian - legacy_jacobian).norm(), 1.0);
  for (int axis = 0; axis < 3; ++axis)
  {
    const V3D displacement = 1e-4 * V3D::Unit(axis);
    const V3D before = point - displacement;
    const V3D after = point + displacement;
    const V2D observed = (camera.world2cam(after) - camera.world2cam(before)) / 2.0;
    EXPECT_LT((robust_jacobian * displacement - observed).norm(), 1e-8);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
