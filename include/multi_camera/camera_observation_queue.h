#pragma once

#include <opencv2/core/mat.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <vector>

namespace fast_livo
{
struct CameraObservation
{
  std::size_t camera_id;
  double stamp;
  cv::Mat image;
};

class CameraObservationQueue
{
public:
  CameraObservationQueue(std::size_t camera_count, double association_window_s = 0.008,
                         double camera_wait_s = 0.05, std::size_t per_camera_limit = 8, double duplicate_window_s = 0.0);

  // stamp is the corrected sensor time in seconds. The image shares ownership with the caller.
  bool push(std::size_t camera_id, double stamp, cv::Mat image, std::uint32_t sequence = 0);
  bool push(std::size_t camera_id, const sensor_msgs::ImageConstPtr &message, double time_offset = 0.0);
  bool push(std::size_t camera_id, const sensor_msgs::CompressedImageConstPtr &message, double time_offset = 0.0);

  // A false result means the sensor watermark has yet to cover the image arrival window.
  // A true result can have no observations. Each camera contributes at most one image.
  bool takeForInterval(double previous_scan_end, double scan_end, double sensor_watermark,
                       std::vector<CameraObservation> &observations);

  // Take every distinct observation in (previous_scan_end, scan_end].
  bool takeEvents(double previous_scan_end, double scan_end, double sensor_watermark,
                  std::vector<CameraObservation> &observations);

private:
  struct QueuedObservation
  {
    double stamp;
    std::uint32_t sequence;
    cv::Mat image;
  };

  const double association_window_s_;
  const double camera_wait_s_;
  const std::size_t per_camera_limit_;
  const double duplicate_window_s_;
  std::vector<double> last_kept_stamp_;
  std::vector<std::deque<QueuedObservation>> cameras_;
  double processed_through_ = -std::numeric_limits<double>::infinity();
  std::mutex mutex_;
};
} // namespace fast_livo
