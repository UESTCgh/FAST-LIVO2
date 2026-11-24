// Adapted from the COIN-BIEVR reproduction, c560f3be7feebf5636fe4b2def021b77e26db52a.
// Copyright (c) 2026, Autonomous Systems Lab, ETH Zurich
// SPDX-License-Identifier: BSD-3-Clause
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include "utils/types.h"

#include <cstddef>
#include <vector>

namespace fast_livo::bievr
{
class BievrVoxelMap;

inline constexpr float kInvalidIntensity = -1.0F;

struct BievrIntensityParameters
{
  int image_width = 900;
  int image_height = 200;
  int window_width = 37;
  int window_height = 13;
  double fov_up_deg = 40.0;
  double fov_down_deg = -40.0;
  double scale = 1.0;
  double brightness_scale = 140.0;
  double min_range = 0.0;
  double max_range = 30.0;
  double degeneracy_ratio = 10.0;
  std::size_t informed_voxels = 100;
  bool smooth = true;
};

// Livox Avia's nonrepetitive scan uses the spherical projection. Output indices
// match the input scan and can be carried through deskew and point selection.
void normalizeIntensities(const PointCloudXYZI &scan, const BievrIntensityParameters &parameters,
                          std::vector<float> &intensities);

// Intensity features are drawn from voxels that constrain the weak directions
// of the geometric model. Return indices into fine_points in a fixed order.
void selectIntensityPoints(const BievrVoxelMap &map, const PointCloudXYZI &fine_points,
                           const std::vector<float> &fine_intensities, const M3D &world_from_lidar_rotation,
                           const V3D &world_from_lidar_translation, const BievrIntensityParameters &parameters,
                           std::vector<std::size_t> &selected_indices);
} // namespace fast_livo::bievr
