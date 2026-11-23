#include "multi_camera/camera_observation_queue.h"

#include <gtest/gtest.h>

namespace fast_livo
{
namespace
{
cv::Mat image(uint8_t value)
{
  return cv::Mat(2, 2, CV_8UC1, cv::Scalar(value));
}

TEST(CameraObservationQueue, UsesTheClosestHeaderIndependentlyForEachCamera)
{
  CameraObservationQueue queue(3, 0.008, 0.04);
  ASSERT_TRUE(queue.push(0, 1.006, image(6), 5));
  ASSERT_TRUE(queue.push(0, 0.993, image(3), 2));
  ASSERT_TRUE(queue.push(0, 1.001, image(1), 4));
  ASSERT_TRUE(queue.push(0, 1.001, image(2), 2));
  ASSERT_TRUE(queue.push(1, 0.998, image(8), 3));
  ASSERT_TRUE(queue.push(1, 1.025, image(9), 4));

  std::vector<CameraObservation> frames;
  EXPECT_FALSE(queue.takeForInterval(0.9, 1.0, 1.02, frames));
  ASSERT_TRUE(queue.takeForInterval(0.9, 1.0, 1.05, frames));
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].camera_id, 1u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 0.998);
  EXPECT_EQ(frames[1].camera_id, 0u);
  EXPECT_DOUBLE_EQ(frames[1].stamp, 1.001);
  EXPECT_EQ(frames[1].image.at<uint8_t>(0, 0), 2);
  EXPECT_FALSE(queue.push(1, 1.002, image(7)));

  ASSERT_TRUE(queue.takeForInterval(1.0, 1.025, 1.07, frames));
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].camera_id, 1u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 1.025);
}

TEST(CameraObservationQueue, MissingAndOutOfWindowImagesDoNotBlockLidar)
{
  CameraObservationQueue queue(2, 0.008, 0.04, 2);
  ASSERT_TRUE(queue.push(0, 3.979, image(1)));
  ASSERT_TRUE(queue.push(0, 3.981, image(2)));
  ASSERT_TRUE(queue.push(0, 4.012, image(3)));
  EXPECT_FALSE(queue.push(0, 3.980, image(4)));

  std::vector<CameraObservation> frames;
  ASSERT_TRUE(queue.takeForInterval(3.9, 4.0, 4.045, frames));
  EXPECT_TRUE(frames.empty());
  ASSERT_TRUE(queue.takeForInterval(4.0, 4.013, 4.06, frames));
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 4.012);
}
} // namespace
} // namespace fast_livo

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
