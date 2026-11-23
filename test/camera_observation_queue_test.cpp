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

TEST(CameraObservationQueue, ExtractsAllEventsInSensorOrderAndRetainsFutureImages)
{
  CameraObservationQueue queue(3, 0.008, 0.04, 8, 0.002);
  ASSERT_TRUE(queue.push(0, 1.020, image(5)));
  ASSERT_TRUE(queue.push(0, 1.000, image(4)));
  ASSERT_TRUE(queue.push(1, 0.980, image(2)));
  ASSERT_TRUE(queue.push(0, 0.980, image(1)));
  ASSERT_TRUE(queue.push(0, 0.930, image(3)));
  ASSERT_TRUE(queue.push(1, 0.900, image(6)));

  std::vector<CameraObservation> frames;
  EXPECT_FALSE(queue.takeEvents(0.900, 1.000, 1.025, frames));
  EXPECT_TRUE(frames.empty());
  ASSERT_TRUE(queue.takeEvents(0.900, 1.000, 1.045, frames));
  ASSERT_EQ(frames.size(), 4u);
  EXPECT_EQ(frames[0].camera_id, 0u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 0.930);
  EXPECT_EQ(frames[1].camera_id, 0u);
  EXPECT_DOUBLE_EQ(frames[1].stamp, 0.980);
  EXPECT_EQ(frames[2].camera_id, 1u);
  EXPECT_DOUBLE_EQ(frames[2].stamp, 0.980);
  EXPECT_EQ(frames[3].camera_id, 0u);
  EXPECT_DOUBLE_EQ(frames[3].stamp, 1.000);

  EXPECT_FALSE(queue.push(2, 0.990, image(7)));
  ASSERT_TRUE(queue.takeEvents(1.000, 1.100, 1.145, frames));
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].camera_id, 0u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 1.020);

  ASSERT_TRUE(queue.takeEvents(1.100, 1.200, 1.245, frames));
  EXPECT_TRUE(frames.empty());
}

TEST(CameraObservationQueue, ResolvesExactEventHeaderTiesWithoutArrivalOrder)
{
  CameraObservationQueue later_sequence_first(1, 0.008, 0.04, 8, 0.002);
  CameraObservationQueue earlier_sequence_first(1, 0.008, 0.04, 8, 0.002);
  ASSERT_TRUE(later_sequence_first.push(0, 2.000, image(1), 30));
  ASSERT_TRUE(later_sequence_first.push(0, 2.000, image(9), 20));
  ASSERT_TRUE(later_sequence_first.push(0, 2.000, image(4), 20));

  ASSERT_TRUE(earlier_sequence_first.push(0, 2.000, image(4), 20));
  EXPECT_FALSE(earlier_sequence_first.push(0, 2.000, image(9), 20));
  EXPECT_FALSE(earlier_sequence_first.push(0, 2.000, image(1), 30));

  for (auto *queue : {&later_sequence_first, &earlier_sequence_first})
  {
    std::vector<CameraObservation> frames;
    ASSERT_TRUE(queue->takeEvents(1.900, 2.000, 2.045, frames));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_DOUBLE_EQ(frames[0].stamp, 2.000);
    EXPECT_EQ(frames[0].image.at<uint8_t>(0, 0), 4);
  }
}

TEST(CameraObservationQueue, SuppressesNearDuplicatesAcrossScanBoundariesPerCamera)
{
  CameraObservationQueue queue(2, 0.008, 0.04, 8, 0.002);
  ASSERT_TRUE(queue.push(0, 3.0010, image(8)));
  ASSERT_TRUE(queue.push(0, 2.9995, image(3)));
  ASSERT_TRUE(queue.push(1, 3.0010, image(6)));
  ASSERT_TRUE(queue.push(0, 3.0025, image(5)));

  std::vector<CameraObservation> frames;
  ASSERT_TRUE(queue.takeEvents(2.900, 3.000, 3.045, frames));
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].camera_id, 0u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 2.9995);

  ASSERT_TRUE(queue.takeEvents(3.000, 3.100, 3.145, frames));
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].camera_id, 1u);
  EXPECT_DOUBLE_EQ(frames[0].stamp, 3.0010);
  EXPECT_EQ(frames[1].camera_id, 0u);
  EXPECT_DOUBLE_EQ(frames[1].stamp, 3.0025);
}
} // namespace
} // namespace fast_livo

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
