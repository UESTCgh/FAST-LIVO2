/*
 * intensity_map.h — Voxel intensity map for LIO photometric residuals
 *
 * Implements COIN-BIEVR-style voxel intensity images for direct photometric
 * alignment in the ESIKF. See design_lio_intensity_degeneracy.md §5.
 *
 * Part of FAST-LIVO2.
 */

#ifndef INTENSITY_MAP_H_
#define INTENSITY_MAP_H_

#include "common_lib.h"
#include "voxel_map.h"
#include <Eigen/Dense>
#include <list>
#include <unordered_map>
#include <vector>

// ---------- Configuration ----------

struct IntensityConfig
{
  bool enable = false;
  bool height_en = false;          // BIEVR 高度图几何残差（L3）开关

  // Voxel intensity map
  double voxel_size = 0.5;          // intensity voxel edge length [m]
  double pixel_size = 0.05;         // pixel resolution within voxel [m]
  double point_leaf = 0.1;          // intensity point downsampling [m]
  int    top_k_voxels = 100;        // map-informed selection: top-K
  double max_range = 30.0;          // discard points beyond [m]
  double min_range = 1.5;           // discard close-range artifacts [m]
  double min_range_weight = 0.5;    // w = min(min_range_weight, 1/‖p‖)
  double sigma_pho = 8.0;           // photometric noise (intensity units)
  double lambda_pho = 1.0;          // relative weight vs. geometric
  bool   line_filter_en = true;     // per-ring horizontal destripe
  int    brightness_window = 21;    // brightness normalization kernel
  int    max_voxels = 200000;       // LRU eviction cap

  // Derived
  int    grid_n = 10;               // pixels per side = voxel_size / pixel_size

  void update_derived() { grid_n = std::max(2, (int)std::round(voxel_size / pixel_size)); }
};

// ---------- Single voxel's intensity image ----------

struct IntensityVoxel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::MatrixXf I_P;   // intensity image  [grid_n × grid_n]
  Eigen::MatrixXf W;     // accumulated weight
  Eigen::MatrixXf H_P;   // height image [grid_n × grid_n]: avg radial distance to dominant plane
  Eigen::MatrixXf HW;    // height weight
  Eigen::Vector3d center; // voxel center (world)
  Eigen::Matrix3d T_CG;  // columns: x_normal, y_normal, normal (from VoxelPlane)
  bool has_plane = false;
  double info_score = 0.0; // gradient magnitude sum for map-informed sampling

  void init(int n, const Eigen::Vector3d& c)
  {
    I_P = Eigen::MatrixXf::Zero(n, n);
    W   = Eigen::MatrixXf::Zero(n, n);
    H_P = Eigen::MatrixXf::Zero(n, n);
    HW  = Eigen::MatrixXf::Zero(n, n);
    center = c;
    has_plane = false;
    info_score = 0.0;
  }
};

// ---------- Intensity point for residual computation ----------

struct IntensityPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d point_b;   // body frame
  Eigen::Vector3d point_w;   // world frame
  float intensity;            // normalized intensity
  VOXEL_LOCATION voxel_loc;   // which intensity voxel
  int u, v;                   // pixel coords in voxel image
};

// ---------- Photometric residual for ESIKF ----------

struct PhotoResidual
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d point_b;     // body (after extrinsic)
  double residual;              // I_measured - I_P(u,v)
  Eigen::Matrix<double, 1, 6> J;  // Jacobian w.r.t. [rot, pos] (1×6)
  double R_inv;                 // inverse variance
};

// ---------- Main class ----------

class VoxelIntensityMap
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VoxelIntensityMap() = default;
  explicit VoxelIntensityMap(const IntensityConfig& cfg);

  // --- Preprocessing ---
  // Takes raw point cloud with intensity, returns normalized intensities.
  // Operates on Ouster-style scan: ring stored in point.normal_z (or ring field).
  void preprocessIntensity(const PointCloudXYZI::Ptr& cloud_in,
                           std::vector<float>& intensities_out);

  // --- Map update ---
  // Accumulate intensity from current frame's world-frame points.
  // planes_map: the geometric voxel map (for extracting plane normals).
  void updateMap(const PointCloudXYZI::Ptr& cloud_body,
                 const std::vector<float>& intensities,
                 const Eigen::Vector3d& eta,  // 退化方向（轴向），用于定向 T_CG 的 u 轴
                 const M3D& R_wl, const V3D& t_wl,
                 const M3D& extR, const V3D& extT,
                 const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*>& planes_map);

  // --- Map-informed sampling ---
  // Given degeneracy direction eta, select top-K voxels whose intensity
  // gradient aligns with eta, then sample points from those voxels.
  // Returns points suitable for residual computation.
  void samplePoints(const Eigen::Vector3d& eta,
                    int n_dirs,
                    const Eigen::Vector3d& eta2,
                    const M3D& R_wl,
                    std::vector<IntensityPoint>& sampled_points);

  // --- Map-informed scan point selection (NEW, §5.4 fix) ---
  // Instead of sampling MAP pixels (self-referential), bucket the CURRENT SCAN
  // points by voxel, score each voxel by l_c (Eq.8), keep top-K voxels, and
  // return those scan points.  Residual = I_scan − I_map(π(T·p)).
  void selectScanPoints(const PointCloudXYZI::Ptr& cloud_body,
                        const std::vector<float>& intensities,
                        const Eigen::Vector3d& eta, int n_dirs, const Eigen::Vector3d& eta2,
                        const M3D& R_wl, const V3D& t_wl,
                        const M3D& extR, const V3D& extT,
                        std::vector<IntensityPoint>& points);

  // --- Height-map point selection (L3) ---
  // Return ALL current-scan points that project into existing height-map
  // voxels (no top-K). Height residuals need as many points as geometric
  // residuals to constrain the degenerate axis.
  void selectHeightPoints(const PointCloudXYZI::Ptr& cloud_body,
                          const M3D& R_wl, const V3D& t_wl,
                          const M3D& extR, const V3D& extT,
                          std::vector<IntensityPoint>& points);

  // --- Residual computation ---
  // Given current state estimate, compute photometric residuals + Jacobians
  // for a set of sampled intensity points.
  void computeResiduals(const std::vector<IntensityPoint>& points,
                        const M3D& R_w, const V3D& t_w,
                        const M3D& extR, const V3D& extT,
                        std::vector<PhotoResidual>& residuals);

  // --- BIEVR height-map geometric residual (L3) ---
  // r = z − H(u,v): 当前点到主导平面的径向距离 − 高度图存储的该像素高度。
  // 高度图保留墙面精细起伏（管片环缝），使几何残差对轴向错位敏感。
  void computeHeightResiduals(const std::vector<IntensityPoint>& points,
                              const Eigen::Vector3d& eta,
                              const M3D& R_w, const V3D& t_w,
                              const M3D& extR, const V3D& extT,
                              std::vector<PhotoResidual>& residuals);

  // --- Access ---
  int numVoxels() const { return (int)voxels_.size(); }
  const IntensityConfig& config() const { return cfg_; }

  // --- 冻结位姿累积（解决自我引用）---
  // 用退化开始时的"相对准确"位姿冻结强度图，之后 updateMap 用冻结位姿投影，
  // 使强度图不随当前(偏快)位姿漂移，光度残差才对速度偏差敏感。
  void setFrozenPose(const M3D& R, const V3D& t)
  {
    frozen_R_ = R; frozen_t_ = t; pose_frozen_ = true;
  }
  bool poseFrozen() const { return pose_frozen_; }
  void clearMap()
  {
    voxels_.clear(); lru_list_.clear(); lru_map_.clear();
  }

private:
  IntensityConfig cfg_;

  // 冻结位姿（退化开始时记录，之后强度图用冻结位姿累积）
  bool pose_frozen_ = false;
  M3D frozen_R_ = M3D::Identity();
  V3D frozen_t_ = V3D::Zero();

  // Intensity voxel map with LRU eviction
  std::unordered_map<VOXEL_LOCATION, IntensityVoxel> voxels_;
  std::list<VOXEL_LOCATION> lru_list_;
  std::unordered_map<VOXEL_LOCATION, std::list<VOXEL_LOCATION>::iterator> lru_map_;

  void touchLRU(const VOXEL_LOCATION& loc);
  void evictLRU();

  // Project a world point into a voxel's image coordinate
  bool projectToVoxelImage(const IntensityVoxel& vox,
                           const Eigen::Vector3d& p_w,
                           int& u, int& v) const;

  // Continuous sub-pixel coordinates
  bool imageCoords(const IntensityVoxel& vox, const Eigen::Vector3d& p_w,
                   double& pu, double& pv) const;

  // Compute per-voxel gradient magnitude (info score)
  void computeInfoScores();

  // Bilinear interpolation + gradient at sub-pixel coords
  bool bilinearSample(const IntensityVoxel& vox,
                      double u, double v,
                      double& value, double& du, double& dv) const;

  // Line destriping filter for Ouster
  void destripeRingwise(const PointCloudXYZI::Ptr& cloud,
                        std::vector<float>& intensities);

  // Brightness normalization (Eq.2 from paper)
  void brightnessNormalize(const PointCloudXYZI::Ptr& cloud,
                           std::vector<float>& intensities);
};

void loadIntensityConfig(ros::NodeHandle& nh, IntensityConfig& cfg);

#endif // INTENSITY_MAP_H_
