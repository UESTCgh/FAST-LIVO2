#include "multi_camera/multi_camera_vio.h"

#include "multi_camera/camera_observation_queue.h"
#include "vio.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace fast_livo
{
namespace
{
template <typename T>
T parameter(ros::NodeHandle &nh, const char *name, const T &default_value)
{
  T value;
  nh.param(name, value, default_value);
  return value;
}

std::unique_ptr<vk::AbstractCamera> loadCamera(ros::NodeHandle &nh, MultiCameraVio::CameraLoader loader)
{
  if (parameter<std::string>(nh, "cam_model", "") == "Pinhole")
  {
    const int width = parameter(nh, "cam_width", 0), height = parameter(nh, "cam_height", 0);
    const double scale = parameter(nh, "scale", 1.0);
    const double fx = parameter(nh, "cam_fx", 0.0), fy = parameter(nh, "cam_fy", 0.0);
    const double cx = parameter(nh, "cam_cx", 0.0), cy = parameter(nh, "cam_cy", 0.0);
    if (width <= 0 || height <= 0 || !(scale > 0 && fx > 0 && fy > 0) ||
        !std::isfinite(scale + fx + fy + cx + cy))
      throw std::invalid_argument("Invalid pinhole calibration in " + nh.getNamespace());
    return std::make_unique<vk::PinholeCamera>(width, height, scale, fx, fy, cx, cy,
        parameter(nh, "cam_d0", 0.0), parameter(nh, "cam_d1", 0.0), parameter(nh, "cam_d2", 0.0),
        parameter(nh, "cam_d3", 0.0), parameter(nh, "cam_d4", 0.0));
  }

  vk::AbstractCamera *camera = nullptr;
  if (!loader(nh.getNamespace(), camera) || !camera)
    throw std::invalid_argument("Unsupported camera model in " + nh.getNamespace());
  return std::unique_ptr<vk::AbstractCamera>(camera);
}

class Camera
{
public:
  Camera(ros::NodeHandle &root, ros::NodeHandle config, const std::string &camera_name,
         const M3D &rotation, const V3D &translation, MultiCameraVio::CameraLoader loader, double default_covariance)
      : name(camera_name), topic(parameter<std::string>(config, "topic", "")),
        transport(parameter<std::string>(config, "transport", "raw")), time_offset(parameter(config, "time_offset", 0.0)),
        camera(loadCamera(config, loader))
  {
    if (topic.empty() || (transport != "raw" && transport != "compressed") || !std::isfinite(time_offset))
      throw std::invalid_argument("Invalid camera input in " + config.getNamespace());
    std::vector<double> r, t;
    if (!config.getParam("Rcl", r) || r.size() != 9 || !config.getParam("Pcl", t) || t.size() != 3)
      throw std::invalid_argument("Rcl and Pcl are required in " + config.getNamespace());
    vio.setLidarToCameraExtrinsic(r, t);
    if (!vio.Rcl.allFinite() || !vio.Pcl.allFinite() || vio.Rcl.determinant() < 0.0 ||
        !(vio.Rcl.transpose() * vio.Rcl).isIdentity(1e-3))
      throw std::invalid_argument("Invalid LiDAR-to-camera transform in " + config.getNamespace());

    vio.cam = camera.get();
    vio.setImuToLidarExtrinsic(translation, rotation);
    vio.state = &candidate;
    vio.state_propagat = &prior;
    vio.grid_size = parameter(root, "vio/grid_size", 5);
    vio.grid_n_width = parameter(root, "vio/grid_n_width", 17);
    vio.grid_n_height = parameter(root, "vio/grid_n_height", 17);
    vio.patch_size = parameter(root, "vio/patch_size", 8);
    vio.patch_pyrimid_level = parameter(root, "vio/patch_pyrimid_level", 4);
    vio.outlier_threshold = parameter(root, "vio/outlier_threshold", 1000.0);
    vio.max_iterations = parameter(root, "vio/max_iterations", 5);
    vio.img_point_cov = parameter(config, "img_point_cov", default_covariance);
    vio.normal_en = parameter(root, "vio/normal_en", true);
    vio.inverse_composition_en = parameter(root, "vio/inverse_composition_en", false);
    vio.raycast_en = parameter(root, "vio/raycast_en", false);
    vio.exposure_estimate_en = parameter(root, "vio/exposure_estimate_en", true);
    vio.plot_flag = false;
    if (!(vio.img_point_cov > 0.0) || !std::isfinite(vio.img_point_cov))
      throw std::invalid_argument("Camera measurement covariance must be positive and finite");
    vio.setVIOQualityThresholds(
        parameter(config, "tracked_min", parameter(root, "vio_quality/tracked_min", 60)),
        parameter(config, "inlier_min", parameter(root, "vio_quality/inlier_min", 40)),
        parameter(config, "inlier_ratio_min", parameter(root, "vio_quality/inlier_ratio_min", 0.5)),
        parameter(config, "mean_err_max", parameter(root, "vio_quality/mean_err_max", 10.0)),
        parameter(root, "vio_quality/max_iters", 10));
    vio.initializeVIO(true);
  }

  const std::string name, topic, transport;
  const double time_offset;
  std::unique_ptr<vk::AbstractCamera> camera;
  VIOManager vio;
  StatesGroup prior, candidate;
  M3D rotation_offset = M3D::Identity();
  V3D position_offset = V3D::Zero();
  double exposure = 1.0;
  double exposure_variance = 0.001;
  ros::Subscriber subscriber;
};

struct VisualCorrection
{
  CameraObservation *observation;
  Camera *camera;
  V3D rotation = V3D::Zero();
  V3D translation = V3D::Zero();
  double weight = 0.0;
};
}  // namespace

struct MultiCameraVio::Impl
{
  std::unique_ptr<CameraObservationQueue> queue;
  std::vector<std::unique_ptr<Camera>> cameras;
  std::vector<CameraObservation> observations;
  std::vector<Camera *> color_views;
  bool visual_feedback;
  double gain;
  double rotation_weight;
  double translation_weight;
  double axial_weight;
  double max_translation;
  double max_rotation;
};

MultiCameraVio::MultiCameraVio(ros::NodeHandle &nh, const M3D &rotation, const V3D &translation, CameraLoader loader)
    : impl_(std::make_unique<Impl>())
{
  auto &impl = *impl_;
  ros::NodeHandle config(nh, "multi_camera");
  std::vector<std::string> names;
  if (!config.getParam("cameras", names) || names.empty())
    throw std::invalid_argument("multi_camera/cameras must contain at least one camera");
  const int queue_size = parameter(config, "queue_size", 8);
  if (queue_size <= 0) throw std::invalid_argument("multi_camera/queue_size must be positive");
  impl.queue = std::make_unique<CameraObservationQueue>(names.size(), parameter(config, "association_window_s", 0.008),
                                                        parameter(config, "camera_wait_s", 0.04), queue_size);
  impl.visual_feedback = parameter(config, "visual_feedback", true);
  impl.gain = parameter(config, "feedback_gain", 0.35);
  impl.rotation_weight = parameter(config, "rotation_weight", 1.0);
  impl.translation_weight = parameter(config, "translation_weight", 1.0);
  impl.axial_weight = parameter(config, "axial_weight", 0.15);
  impl.max_translation = parameter(config, "max_translation_correction", 0.03);
  impl.max_rotation = DEG2RAD(parameter(config, "max_rotation_correction_deg", 0.4));
  for (double weight : {impl.gain, impl.rotation_weight, impl.translation_weight, impl.axial_weight})
  {
    if (!(weight >= 0 && weight <= 1)) throw std::invalid_argument("Multi-camera feedback weights must be in [0, 1]");
  }
  if (!(impl.max_rotation > 0 && impl.max_translation > 0) || !std::isfinite(impl.max_rotation + impl.max_translation))
    throw std::invalid_argument("Multi-camera correction limits must be positive and finite");

  std::unordered_set<std::string> unique_names, unique_topics;
  for (const auto &name : names)
  {
    if (name.empty() || name.find('/') != std::string::npos || !unique_names.insert(name).second)
      throw std::invalid_argument("Camera names must be unique ROS namespace components");
    auto view = std::make_unique<Camera>(nh, ros::NodeHandle(config, name), name, rotation, translation, loader,
                                         parameter(config, "img_point_cov", 1000.0));
    if (!unique_topics.insert(nh.resolveName(view->topic)).second)
      throw std::invalid_argument("Multi-camera input topics must be distinct");
    impl.cameras.push_back(std::move(view));
  }
}

MultiCameraVio::~MultiCameraVio() = default;

void MultiCameraVio::subscribe(ros::NodeHandle &nh)
{
  for (std::size_t id = 0; id < impl_->cameras.size(); ++id)
  {
    auto &view = *impl_->cameras[id];
    const double offset = view.time_offset;
    if (view.transport == "compressed")
      view.subscriber = nh.subscribe<sensor_msgs::CompressedImage>(view.topic, 32,
          [this, id, offset](const sensor_msgs::CompressedImageConstPtr &msg) { impl_->queue->push(id, msg, offset); });
    else
      view.subscriber = nh.subscribe<sensor_msgs::Image>(view.topic, 32,
          [this, id, offset](const sensor_msgs::ImageConstPtr &msg) { impl_->queue->push(id, msg, offset); });
  }
}

bool MultiCameraVio::takeObservations(double previous_scan_end, double scan_end, double imu_watermark)
{
  impl_->color_views.clear();
  return impl_->queue->takeForInterval(previous_scan_end, scan_end, imu_watermark, impl_->observations);
}

void MultiCameraVio::update(double scan_end, const V3D &angular_velocity, const LioQuality &lidar_quality,
                            std::vector<pointWithVar> &points, const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &planes,
                            StatesGroup &state)
{
  auto &impl = *impl_;
  const StatesGroup lidar_pose = state;
  std::vector<VisualCorrection> corrections;
  corrections.reserve(impl.observations.size());

  for (auto &observation : impl.observations)
  {
    auto &view = *impl.cameras[observation.camera_id];
    const double dt = observation.stamp - scan_end;
    view.rotation_offset = Exp(angular_velocity, dt);
    view.position_offset = state.vel_end * dt;
    view.prior = state;
    view.prior.rot_end *= view.rotation_offset;
    view.prior.pos_end += view.position_offset;
    view.prior.inv_expo_time = view.exposure;
    view.prior.cov.row(6).setZero();
    view.prior.cov.col(6).setZero();
    view.prior.cov(6, 6) = view.exposure_variance;
    view.candidate = view.prior;

    if (!view.vio.prepareFrame(observation.image, points, planes, true, true)) continue;
    auto &correction = corrections.emplace_back(VisualCorrection{&observation, &view});
    const M3D corrected_rotation = view.candidate.rot_end * view.rotation_offset.transpose();
    const M3D rotation_delta = state.rot_end.transpose() * corrected_rotation;
    correction.rotation = Log(rotation_delta);
    correction.translation = view.candidate.pos_end - view.prior.pos_end;
    if (view.vio.vio_last_ok_ && correction.rotation.allFinite() && correction.translation.allFinite() &&
        correction.rotation.norm() <= impl.max_rotation && correction.translation.norm() <= impl.max_translation)
    {
      const auto &quality = view.vio.vio_quality_;
      correction.weight = std::sqrt(quality.inliers) * quality.inlier_ratio / std::max(1.0, quality.mean_err);
      view.exposure = view.candidate.inv_expo_time;
      view.exposure_variance = std::clamp(view.candidate.cov(6, 6) + 1e-4, 1e-6, 0.01);
    }
  }

  // View estimates share a LiDAR prior and often see the same surface. Average their
  // corrections and suppress inconsistent views instead of adding their information.
  V3D rotation = V3D::Zero(), translation = V3D::Zero();
  double total_weight = 0.0;
  bool agree = true;
  const VisualCorrection *first = nullptr;
  for (const auto &correction : corrections)
  {
    if (correction.weight == 0.0) continue;
    if (!first) first = &correction;
    agree &= (correction.translation - first->translation).norm() <= impl.max_translation &&
             (correction.rotation - first->rotation).norm() <= impl.max_rotation;
    rotation += correction.weight * correction.rotation;
    translation += correction.weight * correction.translation;
    total_weight += correction.weight;
  }

  const bool use_visual = impl.visual_feedback && impl.gain > 0.0 && agree && total_weight > 0.0;
  if (use_visual)
  {
    V3D weak_direction(lidar_quality.min_evec[3], lidar_quality.min_evec[4], lidar_quality.min_evec[5]);
    if (weak_direction.squaredNorm() > 0.5 && lidar_quality.max_eig > 1e4 * lidar_quality.min_eig)
    {
      weak_direction.normalize();
      translation -= (1.0 - impl.axial_weight) * weak_direction.dot(translation) * weak_direction;
    }
    // This deliberately preserves the LiDAR covariance and inertial states. The
    // visual maps are LiDAR-seeded, so their errors are correlated with that prior.
    const V3D rotation_step = (impl.gain * impl.rotation_weight / total_weight) * rotation;
    state.rot_end *= Exp(rotation_step, 1.0);
    state.pos_end += (impl.gain * impl.translation_weight / total_weight) * translation;
  }

  for (auto &correction : corrections)
  {
    auto &view = *correction.camera;
    const bool selected = use_visual && correction.weight > 0.0;
    view.candidate = state;
    view.candidate.rot_end *= view.rotation_offset;
    view.candidate.pos_end += view.position_offset;
    view.candidate.inv_expo_time = view.exposure;
    // Both camera maps are anchored to the same final pose. The scan geometry
    // before the feedback is the reference for any newly inserted landmarks.
    StatesGroup geometry_pose = lidar_pose;
    geometry_pose.rot_end *= view.rotation_offset;
    geometry_pose.pos_end += view.position_offset;
    view.vio.commitFrame(correction.observation->image, points, planes, geometry_pose, correction.weight > 0.0, true);
    impl.color_views.push_back(&view);
    const auto &q = view.vio.vio_quality_;
    ROS_INFO("[MULTI_CAMERA] scan=%.6f camera=%s image=%.6f tracked=%d inliers=%d rmse=%.4f candidate=%d selected=%d",
             scan_end, view.name.c_str(), correction.observation->stamp, q.tracked, q.inliers, q.rmse,
             correction.weight > 0, selected);
  }
  impl.observations.clear();
}

void MultiCameraVio::colorize(const PointCloudXYZI &world_cloud, double min_range, PointCloudXYZRGB &output) const
{
  output.clear();
  if (impl_->color_views.empty()) return;
  output.reserve(world_cloud.size());
  for (const auto &point : world_cloud)
  {
    Camera *best = nullptr;
    V2D best_pixel;
    double best_score = 0.0;
    const V3D world = point.getVector3fMap().cast<double>();
    for (auto *view : impl_->color_views)
    {
      const V3D ray = view->vio.new_frame_->w2f(world);
      if (!ray.allFinite() || ray.z() <= 0.0 || ray.squaredNorm() <= min_range * min_range) continue;
      const V2D pixel = view->camera->world2cam(ray);
      if (!pixel.allFinite() || pixel.x() < 3 || pixel.x() >= view->vio.width - 3 ||
          pixel.y() < 3 || pixel.y() >= view->vio.height - 3) continue;
      const double score = ray.z() / ray.norm();
      if (score > best_score)
      {
        best = view;
        best_pixel = pixel;
        best_score = score;
      }
    }
    if (!best) continue;
    const V3F bgr = best->vio.getInterpolatedPixel(best->vio.img_rgb, best_pixel);
    PointTypeRGB rgb;
    rgb.getVector3fMap() = point.getVector3fMap();
    rgb.r = bgr.z();
    rgb.g = bgr.y();
    rgb.b = bgr.x();
    output.push_back(rgb);
  }
}

void MultiCameraVio::appendLandmarks(pcl::PointCloud<pcl::PointNormal> &output) const
{
  for (const auto &view : impl_->cameras)
  {
    for (const auto &voxel : view->vio.feat_map)
    {
      for (const auto *point : voxel.second->voxel_points)
      {
        if (!point || !point->pos_.allFinite() || !point->normal_.allFinite()) continue;
        pcl::PointNormal result;
        result.getVector3fMap() = point->pos_.cast<float>();
        result.getNormalVector3fMap() = point->normal_.cast<float>();
        result.curvature = 0.0f;
        output.push_back(result);
      }
    }
  }
}
} // namespace fast_livo
