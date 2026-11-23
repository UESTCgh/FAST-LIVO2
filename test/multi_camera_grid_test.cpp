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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
