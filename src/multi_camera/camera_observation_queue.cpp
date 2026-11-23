#include "multi_camera/camera_observation_queue.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace fast_livo
{
CameraObservationQueue::CameraObservationQueue(std::size_t camera_count, double association_window_s,
                                               double camera_wait_s, std::size_t per_camera_limit, double duplicate_window_s)
    : association_window_s_(association_window_s), camera_wait_s_(camera_wait_s),
      per_camera_limit_(per_camera_limit), duplicate_window_s_(duplicate_window_s),
      last_kept_stamp_(camera_count, -std::numeric_limits<double>::infinity()), cameras_(camera_count)
{
  if (!std::isfinite(association_window_s_) || association_window_s_ < 0.0 ||
      !std::isfinite(camera_wait_s_) || camera_wait_s_ < association_window_s_ || per_camera_limit_ == 0 ||
      !std::isfinite(duplicate_window_s_) || duplicate_window_s_ < 0.0)
    throw std::invalid_argument("Invalid camera association window, wait, or queue capacity");
}

bool CameraObservationQueue::push(std::size_t camera_id, double stamp, cv::Mat image, std::uint32_t sequence)
{
  if (camera_id >= cameras_.size() || !std::isfinite(stamp) || stamp < 0.0 || image.empty()) return false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (stamp <= processed_through_) return false;

  auto &queue = cameras_[camera_id];
  const auto position = std::lower_bound(queue.begin(), queue.end(), stamp,
                                         [](const QueuedObservation &entry, double timestamp) {
    return entry.stamp < timestamp;
  });
  if (position != queue.end() && position->stamp == stamp)
  {
    if (position->sequence < sequence) return false;
    if (position->sequence == sequence)
    {
      if (duplicate_window_s_ == 0.0) return false;
      // Some publishers reuse sequence numbers. Break exact header ties by
      // decoded content so reversed callback order keeps the same image.
      const auto &existing = position->image;
      const auto shape = std::make_tuple(image.rows, image.cols, image.type());
      const auto existing_shape = std::make_tuple(existing.rows, existing.cols, existing.type());
      if (shape > existing_shape) return false;
      if (shape == existing_shape)
      {
        int order = 0;
        for (int row = 0; row < image.rows && order == 0; ++row)
          order = std::memcmp(image.ptr(row), existing.ptr(row), image.cols * image.elemSize());
        if (order >= 0) return false;
      }
    }
    *position = {stamp, sequence, std::move(image)};
    return true;
  }
  if (queue.size() == per_camera_limit_ && position == queue.begin()) return false;

  queue.insert(position, {stamp, sequence, std::move(image)});
  if (queue.size() > per_camera_limit_) queue.pop_front();
  return true;
}

bool CameraObservationQueue::push(std::size_t camera_id, const sensor_msgs::ImageConstPtr &message, double time_offset)
{
  if (!message || camera_id >= cameras_.size()) return false;
  const double stamp = message->header.stamp.toSec() + time_offset;
  try
  {
    return push(camera_id, stamp, cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8)->image,
                message->header.seq);
  }
  catch (const cv_bridge::Exception &)
  {
    return false;
  }
  catch (const cv::Exception &)
  {
    return false;
  }
}

bool CameraObservationQueue::push(std::size_t camera_id, const sensor_msgs::CompressedImageConstPtr &message,
                                  double time_offset)
{
  if (!message || message->data.empty() || camera_id >= cameras_.size()) return false;
  const double stamp = message->header.stamp.toSec() + time_offset;
  try
  {
    return push(camera_id, stamp, cv::imdecode(message->data, cv::IMREAD_COLOR), message->header.seq);
  }
  catch (const cv::Exception &)
  {
    return false;
  }
}

bool CameraObservationQueue::takeForInterval(double previous_scan_end, double scan_end, double sensor_watermark,
                                             std::vector<CameraObservation> &observations)
{
  observations.clear();
  if (!std::isfinite(scan_end) || !std::isfinite(sensor_watermark) || sensor_watermark < scan_end + camera_wait_s_)
    return false;

  const double window_begin = scan_end - association_window_s_;
  const double window_end = scan_end + association_window_s_;
  std::lock_guard<std::mutex> lock(mutex_);
  if (window_end <= processed_through_) return true;

  for (std::size_t camera_id = 0; camera_id < cameras_.size(); ++camera_id)
  {
    auto &queue = cameras_[camera_id];
    auto best = queue.end();
    double best_distance = std::numeric_limits<double>::infinity();
    auto entry = queue.begin();
    for (; entry != queue.end() && entry->stamp <= window_end; ++entry)
    {
      if (entry->stamp < window_begin || entry->stamp <= previous_scan_end) continue;
      const double distance = std::abs(entry->stamp - scan_end);
      // Earlier timestamps win ties. push keeps the lowest sequence at each timestamp.
      if (distance < best_distance)
      {
        best = entry;
        best_distance = distance;
      }
    }
    if (best != queue.end()) observations.push_back({camera_id, best->stamp, std::move(best->image)});
    queue.erase(queue.begin(), entry);
  }
  processed_through_ = window_end;

  std::sort(observations.begin(), observations.end(), [](const CameraObservation &left, const CameraObservation &right) {
    return std::tie(left.stamp, left.camera_id) < std::tie(right.stamp, right.camera_id);
  });
  return true;
}
bool CameraObservationQueue::takeEvents(double previous_scan_end, double scan_end, double sensor_watermark,
                                       std::vector<CameraObservation> &observations)
{
  observations.clear();
  if (!std::isfinite(scan_end) || !std::isfinite(sensor_watermark) || sensor_watermark < scan_end + camera_wait_s_)
    return false;

  std::lock_guard<std::mutex> lock(mutex_);
  if (scan_end <= processed_through_) return true;
  for (std::size_t camera_id = 0; camera_id < cameras_.size(); ++camera_id)
  {
    auto &queue = cameras_[camera_id];
    auto &last_kept = last_kept_stamp_[camera_id];
    while (!queue.empty() && queue.front().stamp <= scan_end)
    {
      auto entry = std::move(queue.front());
      queue.pop_front();
      if (entry.stamp <= previous_scan_end || entry.stamp - last_kept <= duplicate_window_s_) continue;
      last_kept = entry.stamp;
      observations.push_back({camera_id, entry.stamp, std::move(entry.image)});
    }
  }
  processed_through_ = scan_end;
  std::sort(observations.begin(), observations.end(), [](const CameraObservation &left, const CameraObservation &right) {
    return std::tie(left.stamp, left.camera_id) < std::tie(right.stamp, right.camera_id);
  });
  return true;
}
} // namespace fast_livo
