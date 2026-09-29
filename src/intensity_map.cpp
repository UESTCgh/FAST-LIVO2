/*
 * intensity_map.cpp — Voxel intensity map for LIO photometric residuals
 *
 * COIN-BIEVR-style implementation: accumulate per-point LiDAR intensity into
 * voxel-level images, then compute direct photometric residuals in the ESIKF.
 * See design_lio_intensity_degeneracy.md §5.
 *
 * KEY FIX (v2): residuals are scan-to-map, NOT map-to-map.
 *   r_i = I_scan(p_i) − I_map(π(T·p_i))
 * p_i is a scan point (fixed in body frame).  Under the current state T,
 * we project p_i into the map's voxel image and read I_map via bilinear
 * interpolation.  The Jacobian propagates through π(T·p_i).  Because p_i is
 * measured (not from the map), the residual is genuinely state-dependent and
 * the iEKF converges.
 *
 * The previous implementation sampled map pixels and re-projected them: that
 * gives p_w = const, so r = I_map_pixel − I_map(π_inv(π(p_w))) ≈ 0 for any T,
 * while the Jacobian is nonzero → the iEKF chase produces unbounded updates.
 *
 * Part of FAST-LIVO2.
 */

#include "intensity_map.h"
#include <algorithm>
#include <numeric>
#include <cmath>

// ======================================================================
//  Config loading
// ======================================================================

void loadIntensityConfig(ros::NodeHandle& nh, IntensityConfig& cfg)
{
  nh.param<bool>  ("lio/intensity_en",         cfg.enable,           false);
  nh.param<bool>  ("lio/height_en",             cfg.height_en,        false);
  nh.param<double>("lio/intensity/voxel_size",  cfg.voxel_size,       0.5);
  nh.param<double>("lio/intensity/pixel_size",  cfg.pixel_size,       0.05);
  nh.param<double>("lio/intensity/point_leaf",  cfg.point_leaf,       0.1);
  nh.param<int>   ("lio/intensity/top_k_voxels",cfg.top_k_voxels,    100);
  nh.param<double>("lio/intensity/max_range",   cfg.max_range,        30.0);
  nh.param<double>("lio/intensity/min_range",   cfg.min_range,        1.5);
  nh.param<double>("lio/intensity/min_range_weight", cfg.min_range_weight, 0.5);
  nh.param<double>("lio/intensity/sigma_pho",   cfg.sigma_pho,        8.0);
  nh.param<double>("lio/intensity/lambda_pho",  cfg.lambda_pho,       1.0);
  nh.param<bool>  ("lio/intensity/line_filter_en", cfg.line_filter_en, true);
  nh.param<int>   ("lio/intensity/brightness_window", cfg.brightness_window, 21);
  nh.param<int>   ("lio/intensity/max_voxels",  cfg.max_voxels,       200000);
  cfg.update_derived();
}

// ======================================================================
//  Constructor
// ======================================================================

VoxelIntensityMap::VoxelIntensityMap(const IntensityConfig& cfg) : cfg_(cfg)
{
  cfg_.update_derived();
}

// ======================================================================
//  LRU management
// ======================================================================

void VoxelIntensityMap::touchLRU(const VOXEL_LOCATION& loc)
{
  auto it = lru_map_.find(loc);
  if (it != lru_map_.end())
    lru_list_.erase(it->second);
  lru_list_.push_front(loc);
  lru_map_[loc] = lru_list_.begin();
}

void VoxelIntensityMap::evictLRU()
{
  while ((int)voxels_.size() > cfg_.max_voxels && !lru_list_.empty())
  {
    auto loc = lru_list_.back();
    lru_list_.pop_back();
    lru_map_.erase(loc);
    voxels_.erase(loc);
  }
}

// ======================================================================
//  Preprocessing: log-compress + optional destripe + brightness normalize
// ======================================================================

void VoxelIntensityMap::destripeRingwise(const PointCloudXYZI::Ptr& cloud,
                                         std::vector<float>& intensities)
{
  if (cloud->empty()) return;
  const int n = (int)cloud->size();
  std::vector<std::vector<int>> ring_indices(64);
  for (int i = 0; i < n; i++)
  {
    const auto& p = cloud->points[i];
    double range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (range < 0.1) continue;
    double elev = std::asin(p.z / range);
    int ring = std::max(0, std::min(63, (int)((elev + 0.3927) / 0.01227)));
    ring_indices[ring].push_back(i);
  }
  for (int r = 0; r < 64; r++)
  {
    if (ring_indices[r].size() < 5) continue;
    std::vector<float> vals;
    vals.reserve(ring_indices[r].size());
    for (int idx : ring_indices[r])
      vals.push_back(intensities[idx]);
    std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
    float median = vals[vals.size() / 2];
    for (int idx : ring_indices[r])
      intensities[idx] -= median;
  }
  for (int i = 0; i < n; i++)
    intensities[i] += 128.0f;
}

void VoxelIntensityMap::brightnessNormalize(const PointCloudXYZI::Ptr& cloud,
                                            std::vector<float>& intensities)
{
  const int n = (int)intensities.size();
  if (n < 3) return;
  const int hw = cfg_.brightness_window / 2;
  std::vector<double> prefix(n + 1, 0.0);
  for (int i = 0; i < n; i++)
    prefix[i + 1] = prefix[i] + intensities[i];
  float s = (n > 0) ? (float)(prefix[n] / n) : 128.0f;
  for (int i = 0; i < n; i++)
  {
    int lo = std::max(0, i - hw);
    int hi = std::min(n - 1, i + hw);
    float I_B = (float)((prefix[hi + 1] - prefix[lo]) / (hi - lo + 1));
    intensities[i] = s * intensities[i] / (I_B + 1.0f);
  }
}

void VoxelIntensityMap::preprocessIntensity(const PointCloudXYZI::Ptr& cloud_in,
                                            std::vector<float>& intensities_out)
{
  const int n = (int)cloud_in->size();
  intensities_out.resize(n);

  // reflectivity 是 Ouster 出厂标定的反射率（range 独立），无需 range 归一化。
  // 预处理：clip 去镜面反射离群点（reflectivity p99≈650，p100 到 4800+）+ destripe（去逐线条带）。
  // 不用 log 压缩——log 会把值压到 ~5、梯度压到 ~0.5，导致光度 Jacobian 过小。
  for (int i = 0; i < n; i++)
  {
    float v = cloud_in->points[i].intensity;
    if (v > 700.0f) v = 700.0f;
    intensities_out[i] = v;
  }

  if (cfg_.line_filter_en)
    destripeRingwise(cloud_in, intensities_out);
}

// ======================================================================
//  Continuous sub-pixel image coordinates
// ======================================================================

bool VoxelIntensityMap::imageCoords(const IntensityVoxel& vox,
                                    const Eigen::Vector3d& p_w,
                                    double& pu, double& pv) const
{
  if (!vox.has_plane) return false;
  Eigen::Vector3d dp = p_w - vox.center;
  double lu = vox.T_CG.col(0).dot(dp);
  double lv = vox.T_CG.col(1).dot(dp);
  double half = cfg_.voxel_size * 0.5;
  pu = (lu + half) / cfg_.pixel_size;
  pv = (lv + half) / cfg_.pixel_size;
  return (pu >= 0.5 && pu < cfg_.grid_n - 0.5 &&
          pv >= 0.5 && pv < cfg_.grid_n - 0.5);
}

bool VoxelIntensityMap::projectToVoxelImage(const IntensityVoxel& vox,
                                            const Eigen::Vector3d& p_w,
                                            int& u, int& v) const
{
  double pu, pv;
  if (!imageCoords(vox, p_w, pu, pv)) return false;
  u = (int)std::floor(pu);
  v = (int)std::floor(pv);
  return (u >= 0 && u < cfg_.grid_n && v >= 0 && v < cfg_.grid_n);
}

// ======================================================================
//  Bilinear interpolation + gradient
// ======================================================================

bool VoxelIntensityMap::bilinearSample(const IntensityVoxel& vox,
                                       double pu, double pv,
                                       double& value, double& du, double& dv) const
{
  int n = cfg_.grid_n;
  int u0 = (int)std::floor(pu);
  int v0 = (int)std::floor(pv);
  int u1 = u0 + 1;
  int v1 = v0 + 1;

  if (u0 < 0 || u1 >= n || v0 < 0 || v1 >= n) return false;

  if (vox.W(u0, v0) < 0.1f || vox.W(u1, v0) < 0.1f ||
      vox.W(u0, v1) < 0.1f || vox.W(u1, v1) < 0.1f)
    return false;

  double fu = pu - u0;
  double fv = pv - v0;

  double I00 = vox.I_P(u0, v0);
  double I10 = vox.I_P(u1, v0);
  double I01 = vox.I_P(u0, v1);
  double I11 = vox.I_P(u1, v1);

  value = (1 - fu) * (1 - fv) * I00 +
               fu  * (1 - fv) * I10 +
          (1 - fu) *      fv  * I01 +
               fu  *      fv  * I11;

  du = (1 - fv) * (I10 - I00) + fv * (I11 - I01);
  dv = (1 - fu) * (I01 - I00) + fu * (I11 - I10);

  return true;
}

// ======================================================================
//  Map update: accumulate intensity into voxel images
// ======================================================================

void VoxelIntensityMap::updateMap(const PointCloudXYZI::Ptr& cloud_body,
                                 const std::vector<float>& intensities,
                                 const Eigen::Vector3d& eta,
                                 const M3D& R_wl, const V3D& t_wl,
                                 const M3D& extR, const V3D& extT,
                                 const std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*>& planes_map)
{
  // 冻结位姿：退化开始后强度图用冻结位姿累积，避免自我引用。
  const M3D& R_use = pose_frozen_ ? frozen_R_ : R_wl;
  const V3D& t_use = pose_frozen_ ? frozen_t_ : t_wl;
  const int n = (int)cloud_body->size();
  const double vs = cfg_.voxel_size;
  const double max_r2 = cfg_.max_range * cfg_.max_range;
  const double min_r2 = cfg_.min_range * cfg_.min_range;

  int created = 0, skipped_noplane = 0, skipped_range = 0, updated = 0;

  for (int i = 0; i < n; i++)
  {
    const auto& pb = cloud_body->points[i];
    V3D p_body(pb.x, pb.y, pb.z);
    double range2 = p_body.squaredNorm();
    if (range2 < min_r2 || range2 > max_r2) { skipped_range++; continue; }

    V3D p_imu = extR * p_body + extT;
    V3D p_w = R_use * p_imu + t_use;

    // Intensity voxel location
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_w[j] / vs;
      if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0f;
    }
    VOXEL_LOCATION vloc((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    auto it = voxels_.find(vloc);
    if (it == voxels_.end())
    {
      // Find plane from geometric map (try multiple hash sizes)
      VoxelPlane* found_plane = nullptr;
      static const double try_sizes[] = {0.25, 0.5, 1.0, 0.125};
      for (double gs : try_sizes)
      {
        float g_xyz[3];
        for (int j = 0; j < 3; j++)
        {
          g_xyz[j] = p_w[j] / gs;
          if (g_xyz[j] < 0) g_xyz[j] -= 1.0f;
        }
        VOXEL_LOCATION geo_loc((int64_t)g_xyz[0], (int64_t)g_xyz[1], (int64_t)g_xyz[2]);
        auto geo_it = planes_map.find(geo_loc);
        if (geo_it != planes_map.end())
        {
          VoxelOctoTree* leaf = geo_it->second->find_correspond(p_w);
          if (leaf && leaf->plane_ptr_ && leaf->plane_ptr_->is_plane_)
          {
            found_plane = leaf->plane_ptr_;
            break;
          }
        }
      }
      if (!found_plane) { skipped_noplane++; continue; }

      IntensityVoxel vox;
      vox.init(cfg_.grid_n, V3D((0.5 + vloc.x) * vs,
                                 (0.5 + vloc.y) * vs,
                                 (0.5 + vloc.z) * vs));
      // 稳定 T_CG：u 轴沿退化方向 η（轴向），保证高度图梯度 dH_du 是纯轴向梯度。
      // 这样高度残差专约束轴向，不再受体素 PCA 主方向随机的影响。
      V3D n_ax = found_plane->normal_;
      V3D u_ax = eta - n_ax * (eta.dot(n_ax));
      if (u_ax.norm() < 0.1)
        u_ax = found_plane->x_normal_;  // η 几乎平行法线（退化），回退到 PCA
      u_ax.normalize();
      V3D v_ax = n_ax.cross(u_ax);
      v_ax.normalize();
      vox.T_CG.col(0) = u_ax;
      vox.T_CG.col(1) = v_ax;
      vox.T_CG.col(2) = n_ax;
      vox.has_plane = true;
      voxels_[vloc] = vox;
      it = voxels_.find(vloc);
      created++;
    }

    IntensityVoxel& vox = it->second;
    if (!vox.has_plane) continue;

    int u, v;
    if (!projectToVoxelImage(vox, p_w, u, v)) continue;

    double range = std::sqrt(range2);
    double w = std::min(cfg_.min_range_weight, 1.0 / range);
    float I = (i < (int)intensities.size()) ? intensities[i] : pb.intensity;

    float old_W = vox.W(u, v);
    float new_W = old_W + (float)w;
    if (new_W > 1e-6f)
    {
      vox.I_P(u, v) = (old_W * vox.I_P(u, v) + (float)(w * I)) / new_W;
      vox.W(u, v) = new_W;
    }

    // 高度图：点到主导平面的径向距离 z = n·(p_w − center)
    // 逐像素累积（保留墙面精细起伏，用于 BIEVR 高度图几何残差）
    double z = vox.T_CG.col(2).dot(p_w - vox.center);
    float old_HW = vox.HW(u, v);
    if (old_HW < 0.1f)
    {
      vox.H_P(u, v) = (float)z;
      vox.HW(u, v) = 1.0f;
    }
    else
    {
      // EMA（指数滑动平均）：旧帧指数衰减，避免高度图无限累积平滑导致
      // 墙面起伏（环缝）梯度消失，进而轴向约束失效（self-referential）。
      const float alpha = 0.3f;
      vox.H_P(u, v) = (1.0f - alpha) * vox.H_P(u, v) + alpha * (float)z;
    }
    touchLRU(vloc);
    updated++;
  }

  evictLRU();
  computeInfoScores();

  static int upd_dbg = 0;
  if (upd_dbg++ % 50 == 0)
    printf("[L2_MAP] pts=%d created=%d updated=%d noplane=%d range=%d vox=%d\n",
           n, created, updated, skipped_noplane, skipped_range, (int)voxels_.size());
}

// ======================================================================
//  Info scores: gradient magnitude sum per voxel (Eq.6)
// ======================================================================

void VoxelIntensityMap::computeInfoScores()
{
  const int n = cfg_.grid_n;
  for (auto& [loc, vox] : voxels_)
  {
    if (!vox.has_plane) { vox.info_score = 0.0; continue; }
    double score = 0.0;
    for (int u = 1; u < n - 1; u++)
    {
      for (int v = 1; v < n - 1; v++)
      {
        if (vox.W(u-1, v) < 0.1f || vox.W(u+1, v) < 0.1f ||
            vox.W(u, v-1) < 0.1f || vox.W(u, v+1) < 0.1f)
          continue;
        float gu = (vox.I_P(u+1, v) - vox.I_P(u-1, v)) * 0.5f;
        float gv = (vox.I_P(u, v+1) - vox.I_P(u, v-1)) * 0.5f;
        score += std::sqrt(gu * gu + gv * gv);
      }
    }
    vox.info_score = score;
  }
}

// ======================================================================
//  OLD: Map-pixel sampling (kept for API compat, but should NOT be used)
// ======================================================================

void VoxelIntensityMap::samplePoints(const Eigen::Vector3d& eta,
                                     int n_dirs,
                                     const Eigen::Vector3d& eta2,
                                     const M3D& R_wl,
                                     std::vector<IntensityPoint>& sampled_points)
{
  // Deprecated — use selectScanPoints instead.
  // Kept so old callers compile but produce nothing.
  sampled_points.clear();
}

// ======================================================================
//  NEW: Map-informed scan point selection (§5.4 fix)
//
//  Instead of sampling map pixels (self-referential → zero residual),
//  we bucket the CURRENT SCAN into voxels, score each by l_c (Eq.8),
//  keep top-K, and return those scan points + their measured intensity.
// ======================================================================

void VoxelIntensityMap::selectScanPoints(
    const PointCloudXYZI::Ptr& cloud_body,
    const std::vector<float>& intensities,
    const Eigen::Vector3d& eta, int n_dirs, const Eigen::Vector3d& eta2,
    const M3D& R_wl, const V3D& t_wl,
    const M3D& extR, const V3D& extT,
    std::vector<IntensityPoint>& points)
{
  points.clear();
  if (voxels_.empty() || cloud_body->empty()) return;

  const int n = (int)cloud_body->size();
  const double vs = cfg_.voxel_size;
  const double max_r2 = cfg_.max_range * cfg_.max_range;
  const double min_r2 = cfg_.min_range * cfg_.min_range;

  // Step 1: bucket scan points by intensity voxel location
  struct ScanPt { int idx; V3D p_body; float I; VOXEL_LOCATION vloc; };
  std::unordered_map<VOXEL_LOCATION, std::vector<ScanPt>> buckets;

  for (int i = 0; i < n; i++)
  {
    const auto& pb = cloud_body->points[i];
    V3D p_body(pb.x, pb.y, pb.z);
    double r2 = p_body.squaredNorm();
    if (r2 < min_r2 || r2 > max_r2) continue;

    V3D p_w = R_wl * (extR * p_body + extT) + t_wl;

    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_w[j] / vs;
      if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0f;
    }
    VOXEL_LOCATION vloc((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    // Only keep if this voxel exists in the intensity map and has a plane
    auto vit = voxels_.find(vloc);
    if (vit == voxels_.end() || !vit->second.has_plane || vit->second.info_score < 1e-3)
      continue;

    float I = (i < (int)intensities.size()) ? intensities[i] : pb.intensity;
    buckets[vloc].push_back({i, p_body, I, vloc});
  }

  // Step 2: score each candidate voxel by l_c = |η·x_n| * info + |η·y_n| * info
  struct VoxelScore {
    VOXEL_LOCATION loc;
    double score;
  };
  std::vector<VoxelScore> scores;
  scores.reserve(buckets.size());

  for (auto& [vloc, pts] : buckets)
  {
    auto vit = voxels_.find(vloc);
    if (vit == voxels_.end()) continue;
    const IntensityVoxel& vox = vit->second;

    double proj_u = std::abs(eta.dot(vox.T_CG.col(0)));
    double proj_v = std::abs(eta.dot(vox.T_CG.col(1)));
    double alignment = std::sqrt(proj_u * proj_u + proj_v * proj_v);
    if (n_dirs == 2)
    {
      double p2u = std::abs(eta2.dot(vox.T_CG.col(0)));
      double p2v = std::abs(eta2.dot(vox.T_CG.col(1)));
      alignment += std::sqrt(p2u * p2u + p2v * p2v);
    }
    double lc = alignment * vox.info_score;
    if (lc > 1e-6)
      scores.push_back({vloc, lc});
  }

  // Step 3: partial sort → top-K
  int K = std::min(cfg_.top_k_voxels, (int)scores.size());
  if (K == 0) return;
  std::partial_sort(scores.begin(), scores.begin() + K, scores.end(),
                    [](const VoxelScore& a, const VoxelScore& b) { return a.score > b.score; });

  // Step 4: collect scan points from selected voxels
  for (int k = 0; k < K; k++)
  {
    auto bit = buckets.find(scores[k].loc);
    if (bit == buckets.end()) continue;
    for (const auto& sp : bit->second)
    {
      IntensityPoint ip;
      ip.point_b = sp.p_body;   // body frame — this is the MEASUREMENT
      ip.point_w = R_wl * (extR * sp.p_body + extT) + t_wl; // for debug
      ip.intensity = sp.I;       // scan intensity
      ip.voxel_loc = sp.vloc;
      ip.u = ip.v = -1;         // not used (will be computed in computeResiduals)
      points.push_back(ip);
    }
  }
}

// ======================================================================
//  Residual computation (§5.5) — scan-to-map
//
//  r_i = I_scan(p_i) − I_map(π(T · p_i))
//
//  The Jacobian is:
//    ∂h/∂ξ = dI_map/d(u,v) · d(u,v)/dp_w · dp_w/dξ
//  where h = I_map(π(T·p)) (the predicted map intensity).
//  The residual for ESIKF meas_vec is: meas = −r = I_map − I_scan
//  (to match geometric convention where meas_vec(i) = −dis_to_plane).
// ======================================================================

void VoxelIntensityMap::computeResiduals(const std::vector<IntensityPoint>& points,
                                         const M3D& R_w, const V3D& t_w,
                                         const M3D& extR, const V3D& extT,
                                         std::vector<PhotoResidual>& residuals)
{
  residuals.clear();
  residuals.reserve(points.size());

  const double sigma2 = cfg_.sigma_pho * cfg_.sigma_pho;
  const double lambda = cfg_.lambda_pho;
  const double ps = cfg_.pixel_size;
  const double half = cfg_.voxel_size * 0.5;

  for (const auto& ip : points)
  {
    auto vit = voxels_.find(ip.voxel_loc);
    if (vit == voxels_.end()) continue;
    const IntensityVoxel& vox = vit->second;
    if (!vox.has_plane) continue;

    // ---- Transform scan point to world frame under current state ----
    // p_body is measured in LiDAR frame (fixed).
    V3D point_imu = extR * ip.point_b + extT;  // IMU frame
    V3D p_w = R_w * point_imu + t_w;            // world frame (depends on state)

    // ---- Project into voxel image ----
    Eigen::Vector3d dp = p_w - vox.center;
    double lu = vox.T_CG.col(0).dot(dp);
    double lv = vox.T_CG.col(1).dot(dp);
    double pu = (lu + half) / ps;
    double pv = (lv + half) / ps;

    // ---- Bilinear sample with gradient ----
    double I_map, dI_du, dI_dv;
    if (!bilinearSample(vox, pu, pv, I_map, dI_du, dI_dv)) continue;

    // ---- Residual ----
    // r = I_scan − I_map (scan-to-map; positive when scan is brighter)
    double r = ip.intensity - I_map;

    // Skip large outliers (occlusion, dynamic objects)
    if (std::abs(r) > 60.0) continue;

    // ---- Jacobian ----
    // dI_map/dp_w (1×3):
    //   du/dp_w = x_normal^T / ps,  dv/dp_w = y_normal^T / ps
    //   dI/dp_w = (dI_du / ps) * x_n^T + (dI_dv / ps) * y_n^T
    V3D x_n = vox.T_CG.col(0);
    V3D y_n = vox.T_CG.col(1);
    Eigen::RowVector3d dI_dp_w = (dI_du / ps) * x_n.transpose() +
                                  (dI_dv / ps) * y_n.transpose();

    // dp_w/dξ for ESIKF state [δθ, δp]:
    //   dp_w/dδθ = −R · [point_imu]×   (3×3)
    //   dp_w/dδp = I                    (3×3)
    // To match the existing geometric convention (A = [p_imu]× · R^T · n):
    //   H_rot = ([p_imu]× · R^T · dI_dp_w^T)^T
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_imu);
    V3D A_pho = point_crossmat * R_w.transpose() * dI_dp_w.transpose();

    PhotoResidual pr;
    pr.point_b = point_imu;
    pr.residual = r;
    // J = ∂h/∂ξ where h = I_map(π(T·p)):  row = [A_pho^T, dI_dp_w]
    pr.J.block<1, 3>(0, 0) = A_pho.transpose();
    pr.J.block<1, 3>(0, 3) = dI_dp_w;

    // Huber-like soft weighting: w = min(1, k / |r|)
    double abs_r = std::abs(r);
    double huber_k = 15.0;  // about 2σ for sigma_pho=8
    double w_huber = (abs_r < huber_k) ? 1.0 : (huber_k / abs_r);

    pr.R_inv = lambda * w_huber / sigma2;
    residuals.push_back(pr);
  }
  // Zero-mean the photometric residuals. Any common offset (e.g. residual
  // range/brightness bias between the accumulated map and the current scan)
  // is NOT a function of the pose, so it must not enter HTz: a constant
  // offset only contributes a spurious net thrust along whatever direction
  // the mean Jacobian happens to point. Subtracting the per-frame mean
  // removes that bias while preserving the pose-dependent gradient signal.
  if (!residuals.empty())
  {
    double mean = 0.0;
    for (const auto& pr : residuals) mean += pr.residual;
    mean /= (double)residuals.size();
    for (auto& pr : residuals) pr.residual -= mean;
  }
}

// ======================================================================
//  BIEVR height-map point selection (L3)
// ======================================================================

void VoxelIntensityMap::selectHeightPoints(const PointCloudXYZI::Ptr& cloud_body,
                                           const M3D& R_wl, const V3D& t_wl,
                                           const M3D& extR, const V3D& extT,
                                           std::vector<IntensityPoint>& points)
{
  points.clear();
  if (voxels_.empty() || cloud_body->empty()) return;

  const int n = (int)cloud_body->size();
  const double vs = cfg_.voxel_size;
  const double max_r2 = cfg_.max_range * cfg_.max_range;
  const double min_r2 = cfg_.min_range * cfg_.min_range;

  points.reserve(n);
  for (int i = 0; i < n; i++)
  {
    const auto& pb = cloud_body->points[i];
    V3D p_body(pb.x, pb.y, pb.z);
    double r2 = p_body.squaredNorm();
    if (r2 < min_r2 || r2 > max_r2) continue;

    V3D p_w = R_wl * (extR * p_body + extT) + t_wl;

    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_w[j] / vs;
      if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0f;
    }
    VOXEL_LOCATION vloc((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

    auto vit = voxels_.find(vloc);
    if (vit == voxels_.end() || !vit->second.has_plane) continue;

    IntensityPoint ip;
    ip.point_b = p_body;
    ip.point_w = p_w;
    ip.intensity = pb.intensity;
    ip.voxel_loc = vloc;
    ip.u = ip.v = -1;
    points.push_back(ip);
  }
}

// ======================================================================
//  BIEVR height-map geometric residual (L3)
//
//  r = z − H(u,v): 当前点到体素主导平面的径向距离 z = n·(p_w−center)，减去
//  高度图 H(u,v)（该像素历史累积的平均径向距离）。高度图逐像素保留了墙面
//  精细起伏（管片环缝、接缝），因此几何残差对轴向错位敏感——这是 BIEVR-LIO
//  纯几何在隧道里不发散（0.15–0.28m）而 FAST-LIO2 点到面发散的根本原因。
//
//  Jacobian:
//    ∂z/∂p_w     = n^T                              (与点到面残差相同)
//    ∂H/∂p_w     = (dH_du/ps)·x_n^T + (dH_dv/ps)·y_n^T
//    ∂r/∂p_w     = n^T − (dH_du/ps)·x_n^T − (dH_dv/ps)·y_n^T
// ======================================================================

void VoxelIntensityMap::computeHeightResiduals(const std::vector<IntensityPoint>& points,
                                               const Eigen::Vector3d& eta,
                                               const M3D& R_w, const V3D& t_w,
                                               const M3D& extR, const V3D& extT,
                                               std::vector<PhotoResidual>& residuals)
{
  residuals.clear();
  residuals.reserve(points.size());

  const double ps = cfg_.pixel_size;
  const double half = cfg_.voxel_size * 0.5;
  const int n = cfg_.grid_n;

  // 高度残差噪声：Ouster 测距噪声 ~2cm，加上墙面起伏的不确定性，取 5cm。
  // 权重相对几何残差（R_inv~1e2-1e4）取 1/σ² = 400，量级相当。
  const double sigma_h = 0.05;
  const double R_inv_h = 1.0 / (sigma_h * sigma_h);

  for (const auto& ip : points)
  {
    auto vit = voxels_.find(ip.voxel_loc);
    if (vit == voxels_.end()) continue;
    const IntensityVoxel& vox = vit->second;
    if (!vox.has_plane) continue;

    V3D point_imu = extR * ip.point_b + extT;
    V3D p_w = R_w * point_imu + t_w;

    Eigen::Vector3d dp = p_w - vox.center;
    double lu = vox.T_CG.col(0).dot(dp);
    double lv = vox.T_CG.col(1).dot(dp);
    double pu = (lu + half) / ps;
    double pv = (lv + half) / ps;

    // ---- bilinear sample of the height map H_P ----
    int u0 = (int)std::floor(pu);
    int v0 = (int)std::floor(pv);
    int u1 = u0 + 1;
    int v1 = v0 + 1;
    if (u0 < 0 || u1 >= n || v0 < 0 || v1 >= n) continue;
    if (vox.HW(u0, v0) < 0.1f || vox.HW(u1, v0) < 0.1f ||
        vox.HW(u0, v1) < 0.1f || vox.HW(u1, v1) < 0.1f)
      continue;

    double fu = pu - u0;
    double fv = pv - v0;
    double H00 = vox.H_P(u0, v0);
    double H10 = vox.H_P(u1, v0);
    double H01 = vox.H_P(u0, v1);
    double H11 = vox.H_P(u1, v1);
    double H = (1 - fu) * (1 - fv) * H00 + fu * (1 - fv) * H10 +
               (1 - fu) *      fv  * H01 + fu *      fv  * H11;
    double dH_du = (1 - fv) * (H10 - H00) + fv * (H11 - H01);
    double dH_dv = (1 - fu) * (H01 - H00) + fu * (H11 - H10);

    // ---- residual: z = n·dp (point height), r = z − H ----
    V3D n = vox.T_CG.col(2);
    double z = n.dot(dp);
    double r = z - H;

    // outlier rejection (height residual in meters)
    if (std::abs(r) > 0.5) continue;

    // ---- Jacobian ----
    // 高度残差 r = z − H：z = n·dp 是"测量"（当前点高度），H(u,v) 是"预测"
    // （地图高度，随投影位置变）。ESIKF 的 H = ∂h/∂ξ = ∂H/∂ξ = +梯度项（正号），
    // meas = z − H。z 不进 Jacobian，故无 n^T 项。
    (void)eta;
    V3D x_n = vox.T_CG.col(0);
    V3D y_n = vox.T_CG.col(1);
    Eigen::RowVector3d dr_dp_w =
        (dH_du / ps) * x_n.transpose() + (dH_dv / ps) * y_n.transpose();

    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_imu);
    V3D A = point_crossmat * R_w.transpose() * dr_dp_w.transpose();

    PhotoResidual pr;
    pr.point_b = point_imu;
    pr.residual = r;
    pr.J.block<1, 3>(0, 0) = A.transpose();
    pr.J.block<1, 3>(0, 3) = dr_dp_w;
    pr.R_inv = R_inv_h;
    residuals.push_back(pr);
  }

  // 零均值：高度残差含体素主导平面拟合的系统偏置（与位姿无关），必须去除。
  if (!residuals.empty())
  {
    double mean = 0.0;
    for (const auto& pr : residuals) mean += pr.residual;
    mean /= (double)residuals.size();
    for (auto& pr : residuals) pr.residual -= mean;
  }

}
