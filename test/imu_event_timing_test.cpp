#include "IMU_Processing.h"

#include <gtest/gtest.h>

namespace
{
constexpr double kScanStart = 1.0;
constexpr double kScanEnd = 1.1;
constexpr double kFirstEvent = 1.03;
constexpr double kSecondEvent = 1.04;
constexpr double kYawRate = 0.8;
const Eigen::Vector3d kLandmark(6.0, 2.0, -1.0);

Eigen::Matrix3d yawRotation(double angle)
{
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

sensor_msgs::ImuConstPtr imuSample(double stamp, double yaw_rate = 0.0)
{
  sensor_msgs::ImuPtr imu(new sensor_msgs::Imu);
  imu->header.stamp.fromSec(stamp);
  imu->angular_velocity.z = yaw_rate;
  imu->linear_acceleration.z = G_m_s2;
  return imu;
}

LidarMeasureGroup lidarScan()
{
  LidarMeasureGroup scan;
  scan.lio_vio_flg = LIO;
  scan.lidar_frame_beg_time = kScanStart;
  scan.lidar_frame_end_time = kScanEnd;
  auto &measurement = scan.measures.emplace_back();
  measurement.lio_time = kScanEnd;
  measurement.imu = {imuSample(kScanStart, kYawRate), imuSample(1.025, kYawRate), imuSample(1.075, kYawRate)};

  for (double elapsed : {0.02, 0.075, 0.10})
  {
    const Eigen::Vector3d raw = yawRotation(kYawRate * elapsed).transpose() * (kLandmark - Eigen::Vector3d(elapsed, 0, 0));
    PointType point{};
    point.getVector3fMap() = raw.cast<float>();
    point.curvature = static_cast<float>(1000 * elapsed);
    scan.pcl_proc_cur->push_back(point);
  }
  return scan;
}

struct ScanResult
{
  StatesGroup state;
  PointCloudXYZI::Ptr cloud{new PointCloudXYZI};
  double scan_end = 0.0;
};

ScanResult processScan(const std::vector<ImuProcess::TimedObservation> &events = {})
{
  ImuProcess imu;
  imu.first_lidar_time = kScanStart;
  imu.lidar_type = AVIA;
  imu.enableBievrDeskew();
  imu.set_imu_init_frame_num(1);
  imu.disable_exposure_est();
  // Keep Process2's debug output out of the mapper's workspace log.
  imu.fout_imu.open("/dev/null");
  ScanResult result;
  LidarMeasureGroup initial_scan;
  initial_scan.lio_vio_flg = LIO;
  initial_scan.measures.emplace_back().lio_time = kScanStart;
  initial_scan.measures.back().imu = {imuSample(kScanStart)};
  imu.Process2(initial_scan, result.state, result.cloud);
  EXPECT_FALSE(imu.imu_need_init);
  result.state.vel_end = Eigen::Vector3d::UnitX();

  LidarMeasureGroup scan = lidarScan();
  imu.Process2(scan, result.state, result.cloud, events);
  result.scan_end = scan.last_lio_update_time;
  return result;
}

void expectSameCloud(const PointCloudXYZI &actual, const PointCloudXYZI &reference)
{
  ASSERT_EQ(actual.size(), reference.size());
  for (std::size_t index = 0; index < actual.size(); ++index)
  {
    EXPECT_EQ(actual[index].getVector3fMap(), reference[index].getVector3fMap());
    EXPECT_EQ(actual[index].curvature, reference[index].curvature);
  }
}

TEST(ImuEventTiming, PropagatesBothAcceptedCorrectionsWhileKeepingNativeDeskew)
{
  const ScanResult native = processScan();
  std::vector<double> invoked;
  const auto first = [&](double stamp, StatesGroup &state) {
    invoked.push_back(stamp);
    EXPECT_NEAR(state.pos_end.x(), 0.03, 1e-12);
    EXPECT_TRUE(state.rot_end.isApprox(yawRotation(0.024), 1e-12));
    state.pos_end.y() += 0.2;
    state.vel_end.x() = 2.0;
    state.rot_end *= yawRotation(0.12);
    state.cov(6, 6) = 0.003;
    return true;
  };
  const auto second = [&](double stamp, StatesGroup &state) {
    invoked.push_back(stamp);
    EXPECT_TRUE(state.pos_end.isApprox(Eigen::Vector3d(0.05, 0.2, 0), 1e-12));
    EXPECT_TRUE(state.rot_end.isApprox(yawRotation(0.152), 1e-12));
    EXPECT_DOUBLE_EQ(state.cov(6, 6), 0.003);
    state.pos_end.x() += 0.3;
    state.vel_end.x() = 3.0;
    state.rot_end *= yawRotation(-0.04);
    return true;
  };
  const ScanResult corrected = processScan({{kFirstEvent, first}, {kSecondEvent, second}});

  EXPECT_EQ(invoked, (std::vector<double>{kFirstEvent, kSecondEvent}));
  EXPECT_DOUBLE_EQ(corrected.scan_end, kScanEnd);
  EXPECT_TRUE(corrected.state.pos_end.isApprox(Eigen::Vector3d(0.53, 0.2, 0), 1e-12));
  EXPECT_TRUE(corrected.state.vel_end.isApprox(Eigen::Vector3d(3, 0, 0), 1e-12));
  EXPECT_TRUE(corrected.state.rot_end.isApprox(yawRotation(0.16), 1e-12));
  EXPECT_DOUBLE_EQ(corrected.state.cov(6, 6), 0.003);

  const Eigen::Vector3d expected = yawRotation(0.08).transpose() * (kLandmark - Eigen::Vector3d(0.10, 0, 0));
  ASSERT_EQ(native.cloud->size(), 3u);
  for (const PointType &point : *native.cloud)
    EXPECT_LT((point.getVector3fMap().cast<double>() - expected).norm(), 1e-6);
  expectSameCloud(*corrected.cloud, *native.cloud);
}

TEST(ImuEventTiming, RejectedCorrectionsKeepNativeStateCovarianceAndDeskew)
{
  const ScanResult native = processScan();
  std::vector<double> invoked;
  const auto rejected = [&](double stamp, StatesGroup &state) {
    invoked.push_back(stamp);
    EXPECT_NEAR(state.pos_end.x(), stamp - kScanStart, 1e-12);
    EXPECT_TRUE(state.rot_end.isApprox(yawRotation(kYawRate * (stamp - kScanStart)), 1e-12));
    state.pos_end.setConstant(100.0);
    state.vel_end.setConstant(100.0);
    state.rot_end = yawRotation(1.0);
    state.cov.setZero();
    return false;
  };
  ScanResult result = processScan({{kFirstEvent, rejected}, {kSecondEvent, rejected}});

  EXPECT_EQ(invoked, (std::vector<double>{kFirstEvent, kSecondEvent}));
  EXPECT_DOUBLE_EQ(result.scan_end, native.scan_end);
  EXPECT_EQ(result.state - native.state, (Eigen::Matrix<double, DIM_STATE, 1>::Zero()));
  EXPECT_EQ(result.state.cov, native.state.cov);
  expectSameCloud(*result.cloud, *native.cloud);
}
} // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
