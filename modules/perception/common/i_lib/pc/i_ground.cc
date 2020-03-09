#include "cyber/common/log.h"
/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/common/i_lib/pc/i_ground.h"

#include <algorithm>
#include <cfloat>

namespace apollo {
namespace perception {
namespace common {
void PlaneFitGroundDetectorParam::SetDefault() {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetectorParam::SetDefault";
  nr_points_max = 320000;  // assume max 320000 points
  nr_grids_fine = 256;     // must be 2 and above
  nr_grids_coarse = 16;    // must be 2 and above
  nr_z_comp_candis = 32;
  nr_z_comp_fail_threshold = 4;
  nr_inliers_min_threshold = 8;
  nr_samples_min_threshold = 128;
  nr_samples_max_threshold = 1024;
  sample_region_z_lower = -3.0f;  // -3.0m
  sample_region_z_upper = -1.0f;  // -1.0m
  roi_region_rad_x = 72.0f;       // 72.0f
  roi_region_rad_y = 72.0f;       // 72.0f
  roi_region_rad_z = 10.0f;
  roi_near_rad = 32.0f;                   // near range: 32m;
  planefit_dist_threshold_near = 0.10f;   // 10.cm
  planefit_dist_threshold_far = 0.20f;    // 20.0cm
  planefit_filter_threshold = 0.10f;      // 10.0cm;
  planefit_orien_threshold = 5.0f;        // 5 degree
  termi_inlier_percen_threshold = 0.99f;  // 99%
  nr_ransac_iter_threshold = 32;
  candidate_filter_threshold = 1.0f;  // 1 meter
  nr_smooth_iter = 1;
}

bool PlaneFitGroundDetectorParam::Validate() const {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetectorParam::Validate";
  if (nr_grids_coarse < 2 || nr_grids_fine < 2 ||
      nr_grids_coarse > nr_grids_fine || nr_points_max == 0 ||
      nr_samples_min_threshold == 0 || nr_samples_max_threshold == 0 ||
      nr_inliers_min_threshold == 0 || nr_ransac_iter_threshold == 0 ||
      roi_region_rad_x <= 0.f || roi_region_rad_y <= 0.f ||
      roi_region_rad_z <= 0.f ||
      planefit_dist_threshold_near > planefit_dist_threshold_far) {
    std::cerr << "Invalid ground detector parameters... " << std::endl;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetectorParam::Validate";
  return false;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetectorParam::Validate";
  return true;
}

int PlaneFitPointCandIndices::Prune(unsigned int min_nr_samples,
                                    unsigned int max_nr_samples) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitPointCandIndices::Prune";
  assert(min_nr_samples < max_nr_samples);
  unsigned int size = static_cast<unsigned int>(indices.size());
  unsigned int half = 0;
  if (size > max_nr_samples) {
    IRandomizedShuffle1(indices.data(), static_cast<int>(indices.size()),
                        &random_seed);
    // IRandomSample(indices.Data(), max_nr_samples, static_cast<int>Size,
    // random_seed);
    // for (int i = 0; i < max_nr_samples; ++i) {
    //     int j = indices[i];
    //     // i <= j forever;
    //     indices[i] = indices[j];
    // }
    indices.resize(max_nr_samples);
  } else {
    if (size > min_nr_samples) {
      half = size >> 1;
      while (half > min_nr_samples) {
        half = half >> 1;
      }
      size = half << 1;
      IRandomizedShuffle1(indices.data(), static_cast<int>(indices.size()),
                          &random_seed);
      indices.resize(size);
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitPointCandIndices::Prune";
  return static_cast<int>(indices.size());
}

PlaneFitGroundDetector::PlaneFitGroundDetector(
    const PlaneFitGroundDetectorParam &param)
    : BaseGroundDetector(param) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::PlaneFitGroundDetector";
  assert(Init());
}

PlaneFitGroundDetector::~PlaneFitGroundDetector() { CleanUp(); }

// Init the order lookup table
void PlaneFitGroundDetector::InitOrderTable(const VoxelGridXY<float> *vg,
                                            std::pair<int, int> *order) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::InitOrderTable";
  std::vector<std::pair<float, int> > map_dist;
  float cx = 0.f;
  float cy = 0.f;
  float dist2 = 0.f;
  float radius = 0.f;
  unsigned int i = 0;
  int id = 0;
  for (i = 0; i < vg->NrVoxel(); ++i) {
    const auto &voxel = vg->GetConstVoxels()[i];
    radius = voxel.dim_x_ * 0.5f;
    cx = voxel.v_[0] + radius;
    cy = voxel.v_[1] + radius;
    dist2 = cx * cx + cy * cy;
    map_dist.push_back(std::pair<float, int>(dist2, i));
  }
  sort(map_dist.begin(), map_dist.end(),
       [](const std::pair<float, int> &a, const std::pair<float, int> &b) {
         
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::InitOrderTable";
  return a.first < b.first;
       });
  for (i = 0; i < map_dist.size(); ++i) {
    id = map_dist[i].second;
    const auto &voxel = vg->GetConstVoxels()[id];
    order[i].first = voxel.iy_;
    order[i].second = voxel.ix_;
  }

  AINFO<<"(DMCZP) LeaveMethod: PlaneFitGroundDetector::InitOrderTable";
 }

bool PlaneFitGroundDetector::Init() {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::Init";
  unsigned int r = 0;
  unsigned int c = 0;
  unsigned int pr = 0;
  unsigned int pc = 0;
  unsigned int index = 0;
  unsigned int capacity = 0;
  unsigned int sf = param_.nr_grids_fine / param_.nr_grids_coarse;
  if (!param_.Validate()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  // fine grid:
  vg_fine_ = new VoxelGridXY<float>();
  if (vg_fine_ == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  if (!vg_fine_->Alloc(param_.nr_grids_fine, param_.nr_grids_fine,
                       -param_.roi_region_rad_x, param_.roi_region_rad_x,
                       -param_.roi_region_rad_y, param_.roi_region_rad_y,
                       -param_.roi_region_rad_z, param_.roi_region_rad_z)) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  // coarse grid:
  vg_coarse_ = new VoxelGridXY<float>();
  if (vg_coarse_ == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  if (!vg_coarse_->Alloc(param_.nr_grids_coarse, param_.nr_grids_coarse,
                         -param_.roi_region_rad_x, param_.roi_region_rad_x,
                         -param_.roi_region_rad_y, param_.roi_region_rad_y,
                         -param_.roi_region_rad_z, param_.roi_region_rad_z)) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }

  // Init order lookup table
  order_table_ = IAlloc<std::pair<int, int> >(vg_fine_->NrVoxel());
  InitOrderTable(vg_coarse_, order_table_);

  // ground plane:
  ground_planes_ =
      IAlloc2<GroundPlaneLiDAR>(param_.nr_grids_coarse, param_.nr_grids_coarse);
  if (!ground_planes_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  ground_planes_sphe_ = IAlloc2<GroundPlaneSpherical>(param_.nr_grids_coarse,
                                                      param_.nr_grids_coarse);
  if (!ground_planes_sphe_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  ground_z_ = IAlloc2<std::pair<float, bool> >(param_.nr_grids_coarse,
                                               param_.nr_grids_coarse);
  if (!ground_z_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  // sample candis:
  local_candis_ = IAlloc2<PlaneFitPointCandIndices>(param_.nr_grids_coarse,
                                                    param_.nr_grids_coarse);
  if (!local_candis_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  // Reserve space to avoid runtime memory re-allocation
  for (r = 0; r < param_.nr_grids_coarse; ++r) {
    for (c = 0; c < param_.nr_grids_coarse; ++c) {
      if (r < (param_.nr_grids_coarse << 3) &&
          c < (param_.nr_grids_coarse << 3)) {
        capacity = 16384;
      } else if (r < (param_.nr_grids_coarse << 2) &&
                 c < (param_.nr_grids_coarse << 2)) {
        capacity = 8192;
      } else if (r < (param_.nr_grids_coarse << 1) &&
                 c < (param_.nr_grids_coarse << 1)) {
        capacity = 4096;
      } else {
        capacity = 512;
      }
      local_candis_[r][c].Reserve(capacity);
    }
  }
  // threeds in ransac, in inhomogeneous coordinates:
  pf_threeds_ =
      IAllocAligned<float>(param_.nr_samples_max_threshold * dim_point_, 4);
  if (!pf_threeds_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  memset(reinterpret_cast<void *>(pf_threeds_), 0,
         param_.nr_samples_max_threshold * dim_point_ * sizeof(float));
  // labels:
  labels_ = IAllocAligned<char>(param_.nr_points_max, 4);
  if (!labels_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  memset(reinterpret_cast<void *>(labels_), 0,
         param_.nr_points_max * sizeof(char));
  // map of fine grid id to coarse id:
  map_fine_to_coarse_ = IAllocAligned<unsigned int>(
      param_.nr_grids_fine * param_.nr_grids_fine, 4);
  if (!map_fine_to_coarse_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  for (r = 0; r < param_.nr_grids_fine; ++r) {
    pr = r / sf;
    index = r * param_.nr_grids_fine;
    for (c = 0; c < param_.nr_grids_fine; ++c) {
      pc = c / sf;
      map_fine_to_coarse_[index + c] = pr * param_.nr_grids_coarse + pc;
    }
  }
  // ransac memory:
  sampled_z_values_ = IAllocAligned<float>(param_.nr_z_comp_candis, 4);
  if (!sampled_z_values_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  memset(reinterpret_cast<void *>(sampled_z_values_), 0,
         param_.nr_z_comp_candis * sizeof(float));
  // ransac memory:
  sampled_indices_ = IAllocAligned<int>(param_.nr_z_comp_candis, 4);
  if (!sampled_indices_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  memset(reinterpret_cast<void *>(sampled_indices_), 0,
         param_.nr_z_comp_candis * sizeof(int));
  // ransac thresholds:
  pf_thresholds_ =
      IAlloc2<float>(param_.nr_grids_coarse, param_.nr_grids_coarse);
  if (!pf_thresholds_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return false;
  }
  // compute thresholds
  ComputeAdaptiveThreshold();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Init";
  return true;
}

void PlaneFitGroundDetector::CleanUp() {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::CleanUp";
  if (vg_fine_) {
    delete vg_fine_;
  }
  if (vg_coarse_) {
    delete vg_coarse_;
  }
  IFree2<GroundPlaneLiDAR>(&ground_planes_);
  IFree2<GroundPlaneSpherical>(&ground_planes_sphe_);
  IFree2<std::pair<float, bool> >(&ground_z_);
  IFree2<PlaneFitPointCandIndices>(&local_candis_);
  IFreeAligned<float>(&pf_threeds_);
  IFreeAligned<char>(&labels_);
  IFreeAligned<unsigned int>(&map_fine_to_coarse_);
  IFreeAligned<float>(&sampled_z_values_);
  IFreeAligned<int>(&sampled_indices_);
  IFree2<float>(&pf_thresholds_);
  IFree<std::pair<int, int> >(&order_table_);
}

int PlaneFitGroundDetector::CompareZ(const float *point_cloud,
                                     const std::vector<int> &indices,
                                     const float *z_values,
                                     PlaneFitPointCandIndices *candi,
                                     unsigned int nr_points,
                                     unsigned int nr_point_element,
                                     unsigned int nr_compares) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::CompareZ";
  int pos = 0;
  int nr_candis = 0;
  unsigned int i = 0;
  unsigned int nr_contradi = 0;
  unsigned int nr_z_comp_fail_threshold =
      IMin(param_.nr_z_comp_fail_threshold, (unsigned int)(nr_compares >> 1));
  const float *ptr = nullptr;
  float z = 0.0f;
  float delta_z = 0.0f;
  std::vector<int>::const_iterator iter = indices.cbegin();
  while (iter < indices.cend()) {
    nr_contradi = 0;
    pos = *iter++;
    assert(pos < static_cast<int>(nr_points));
    nr_contradi = 0;
    //  requires the Z element to be in the third position, i.e., after X, Y
    ptr = point_cloud + (pos * nr_point_element);
    z = ptr[2];
    // for near range check valid height
    if (IAbs(ptr[0]) < param_.roi_near_rad &&
        IAbs(ptr[1]) < param_.roi_near_rad) {
      if (z < param_.sample_region_z_lower ||
          z > param_.sample_region_z_upper) {
        continue;
      }
    }
    if (nr_compares > nr_z_comp_fail_threshold) {
      for (i = 0; i < nr_compares; ++i) {
        delta_z = IAbs(z_values[i] - z);
        if (delta_z > param_.planefit_filter_threshold) {
          nr_contradi++;
          if (nr_contradi > nr_z_comp_fail_threshold) {
            break;
          }
        }
      }
    }
    if (nr_contradi <= nr_z_comp_fail_threshold) {
      labels_[pos] = 1;
      candi->PushIndex(pos);
      nr_candis++;
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::CompareZ";
  return nr_candis;
}

void PlaneFitGroundDetector::ComputeAdaptiveThreshold() {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::ComputeAdaptiveThreshold";
  unsigned int r = 0;
  unsigned int c = 0;
  float dr = 0.0f;
  float dc = 0.0f;
  float k = 0.0f;
  float b = 0.0f;
  float min_dist = 0.0f;
  float max_dist = 0.0f;
  float thre = 0.0f;
  float grid_rad = static_cast<float>(param_.nr_grids_coarse - 1) / 2;
  assert(pf_thresholds_ != nullptr);
  for (r = 0; r < param_.nr_grids_coarse; ++r) {
    dr = static_cast<float>(r) - grid_rad;
    dr *= dr;
    for (c = 0; c < param_.nr_grids_coarse; ++c) {
      dc = static_cast<float>(c) - grid_rad;
      dc *= dc;
      // store to center distance:
      pf_thresholds_[r][c] = ISqrt(dr + dc);
    }
  }
  IMinMaxElements(pf_thresholds_[0],
                  param_.nr_grids_coarse * param_.nr_grids_coarse, &min_dist,
                  &max_dist);
  if (max_dist - min_dist < Constant<float>::EPSILON()) {
    thre = (param_.planefit_dist_threshold_near +
            param_.planefit_dist_threshold_far) /
           2;
    for (r = 0; r < param_.nr_grids_coarse; ++r) {
      for (c = 0; c < param_.nr_grids_coarse; ++c) {
        pf_thresholds_[r][c] = thre;
      }
    }
  } else {
    k = param_.planefit_dist_threshold_far -
        param_.planefit_dist_threshold_near;
    k = k / (max_dist - min_dist);
    b = param_.planefit_dist_threshold_far +
        param_.planefit_dist_threshold_near;
    b = (b - k * (min_dist + max_dist)) / 2;
    for (r = 0; r < param_.nr_grids_coarse; ++r) {
      for (c = 0; c < param_.nr_grids_coarse; ++c) {
        thre = k * pf_thresholds_[r][c] + b;
        pf_thresholds_[r][c] = thre;
      }
    }
  }
}

void PlaneFitGroundDetector::ComputeSignedGroundHeight(
    const float *point_cloud, float *height_above_ground,
    unsigned int nr_points, unsigned int nr_point_elements) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::ComputeSignedGroundHeight";
  unsigned int r = 0;
  unsigned int nm1 = param_.nr_grids_coarse - 1;
  for (r = 0; r < nr_points; ++r) {
    height_above_ground[r] = FLT_MAX;
  }
  ComputeSignedGroundHeightLine(
      point_cloud, ground_planes_[0], ground_planes_[0], ground_planes_[1],
      height_above_ground, 0, nr_points, nr_point_elements);
  for (r = 1; r < nm1; ++r) {
    ComputeSignedGroundHeightLine(point_cloud, ground_planes_[r - 1],
                                  ground_planes_[r], ground_planes_[r + 1],
                                  height_above_ground, r, nr_points,
                                  nr_point_elements);
  }
  ComputeSignedGroundHeightLine(point_cloud, ground_planes_[nm1 - 1],
                                ground_planes_[nm1], ground_planes_[nm1],
                                height_above_ground, nm1, nr_points,
                                nr_point_elements);
}

void PlaneFitGroundDetector::ComputeSignedGroundHeightLine(
    const float *point_cloud, const GroundPlaneLiDAR *up,
    const GroundPlaneLiDAR *cn, const GroundPlaneLiDAR *dn,
    float *height_above_ground, unsigned int r, unsigned int nr_points,
    unsigned int nr_point_elements) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::ComputeSignedGroundHeightLine";
  unsigned int i = 0;
  unsigned int c = 0;
  unsigned int id = 0;
  int pos = 0;
  char label = 0;
  const float *cptr = nullptr;
  float dist[] = {0, 0, 0, 0, 0};
  const float *plane[] = {nullptr, nullptr, nullptr, nullptr, nullptr};
  float min_abs_dist = 0.0f;
  unsigned int nm1 = param_.nr_grids_coarse - 1;
  assert(param_.nr_grids_coarse >= 2);
  plane[0] = cn[0].IsValid() ? cn[0].params : nullptr;
  plane[1] = cn[1].IsValid() ? cn[1].params : nullptr;
  plane[2] = up[0].IsValid() ? up[0].params : nullptr;
  plane[3] = dn[0].IsValid() ? dn[0].params : nullptr;
  std::vector<int>::const_iterator iter = (*vg_coarse_)(r, 0).indices_.cbegin();
  while (iter < (*vg_coarse_)(r, 0).indices_.cend()) {
    pos = *iter;
    assert(pos < static_cast<int>(nr_points));
    label = labels_[pos];
    cptr = point_cloud + (nr_point_elements * pos);
    dist[0] = plane[0] != nullptr
                  ? IPlaneToPointSignedDistanceWUnitNorm(plane[0], cptr)
                  : FLT_MAX;
    min_abs_dist = IAbs(dist[0]);
    id = 0;
    // for candidates we take min dist:
    if (label) {
      dist[1] = plane[1] != nullptr
                    ? IPlaneToPointSignedDistanceWUnitNorm(plane[1], cptr)
                    : FLT_MAX;
      dist[2] = plane[2] != nullptr
                    ? IPlaneToPointSignedDistanceWUnitNorm(plane[2], cptr)
                    : FLT_MAX;
      dist[3] = plane[3] != nullptr
                    ? IPlaneToPointSignedDistanceWUnitNorm(plane[3], cptr)
                    : FLT_MAX;
      for (i = 1; i < 4; ++i) {
        if (min_abs_dist > IAbs(dist[i])) {
          min_abs_dist = IAbs(dist[i]);
          id = i;
        }
      }
    }
    height_above_ground[pos] = dist[id];
    ++iter;
  }

  for (c = 1; c < nm1; ++c) {
    plane[0] = cn[c].IsValid() ? cn[c].params : nullptr;
    plane[1] = cn[c - 1].IsValid() ? cn[c - 1].params : nullptr;
    plane[2] = cn[c + 1].IsValid() ? cn[c + 1].params : nullptr;
    plane[3] = up[c].IsValid() ? up[c].params : nullptr;
    plane[4] = dn[c].IsValid() ? dn[c].params : nullptr;
    iter = (*vg_coarse_)(r, c).indices_.cbegin();
    while (iter < (*vg_coarse_)(r, c).indices_.cend()) {
      pos = *iter;
      assert(pos < static_cast<int>(nr_points));
      label = labels_[pos];
      cptr = point_cloud + (nr_point_elements * pos);
      dist[0] = plane[0] != nullptr
                    ? IPlaneToPointSignedDistanceWUnitNorm(plane[0], cptr)
                    : FLT_MAX;
      min_abs_dist = IAbs(dist[0]);
      id = 0;
      if (label) {
        dist[1] = plane[1] != nullptr
                      ? IPlaneToPointSignedDistanceWUnitNorm(plane[1], cptr)
                      : FLT_MAX;
        dist[2] = plane[2] != nullptr
                      ? IPlaneToPointSignedDistanceWUnitNorm(plane[2], cptr)
                      : FLT_MAX;
        dist[3] = plane[3] != nullptr
                      ? IPlaneToPointSignedDistanceWUnitNorm(plane[3], cptr)
                      : FLT_MAX;
        dist[4] = plane[4] != nullptr
                      ? IPlaneToPointSignedDistanceWUnitNorm(plane[4], cptr)
                      : FLT_MAX;
        for (i = 1; i < 5; ++i) {
          if (min_abs_dist > IAbs(dist[i])) {
            min_abs_dist = IAbs(dist[i]);
            id = i;
          }
        }
      }
      height_above_ground[pos] = dist[id];
      ++iter;
    }
  }
  plane[0] = cn[nm1].IsValid() ? cn[nm1].params : nullptr;
  plane[1] = cn[nm1 - 1].IsValid() ? cn[nm1 - 1].params : nullptr;
  plane[2] = up[nm1].IsValid() ? up[nm1].params : nullptr;
  plane[3] = dn[nm1].IsValid() ? dn[nm1].params : nullptr;
  iter = (*vg_coarse_)(r, nm1).indices_.cbegin();
  while (iter < (*vg_coarse_)(r, nm1).indices_.cend()) {
    pos = *iter;
    assert(pos < static_cast<int>(nr_points));
    label = labels_[pos];
    cptr = point_cloud + (nr_point_elements * pos);
    dist[0] = plane[0] != nullptr
                  ? IPlaneToPointSignedDistanceWUnitNorm(plane[0], cptr)
                  : FLT_MAX;
    min_abs_dist = IAbs(dist[0]);
    id = 0;
    // for candidates we take min dist:
    if (label) {
      dist[1] = plane[1] != nullptr
                    ? IPlaneToPointSignedDistanceWUnitNorm(plane[1], cptr)
                    : FLT_MAX;
      dist[2] = plane[2] != nullptr
                    ? IPlaneToPointSignedDistanceWUnitNorm(plane[2], cptr)
                    : FLT_MAX;
      dist[3] = plane[3] != nullptr
                    ? IPlaneToPointSignedDistanceWUnitNorm(plane[3], cptr)
                    : FLT_MAX;
      for (i = 1; i < 4; ++i) {
        if (min_abs_dist > IAbs(dist[i])) {
          min_abs_dist = IAbs(dist[i]);
          id = i;
        }
      }
    }
    height_above_ground[pos] = dist[id];
    ++iter;
  }
}

int PlaneFitGroundDetector::FilterGrid(const Voxel<float> &vx,
                                       const float *point_cloud,
                                       PlaneFitPointCandIndices *candi,
                                       unsigned int nr_points,
                                       unsigned int nr_point_element) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::FilterGrid";
  int pos = 0;
  int rseed = I_DEFAULT_SEED;
  int nr_candis = 0;
  unsigned int i = 0;
  unsigned int nr_samples = IMin(param_.nr_z_comp_candis, vx.NrPoints());
  if (vx.Empty()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FilterGrid";
  return 0;
  }
  //  generate sampled indices
  if (vx.NrPoints() <= param_.nr_z_comp_candis) {
    // IRamp(sampled_indices_, vx.NrPoints());
    //  sampled z values
    for (i = 0; i < vx.NrPoints(); ++i) {
      pos = vx.indices_[i] * nr_point_element;
      //  requires the Z element to be in the third position, i.e., after X, Y
      sampled_z_values_[i] = (point_cloud + pos)[2];
    }
  } else {
    IRandomSample(sampled_indices_, static_cast<int>(param_.nr_z_comp_candis),
                  static_cast<int>(vx.NrPoints()), &rseed);
    //  sampled z values
    for (i = 0; i < nr_samples; ++i) {
      pos = vx.indices_[sampled_indices_[i]] * nr_point_element;
      // requires the Z element to be in the third position, i.e., after X, Y
      sampled_z_values_[i] = (point_cloud + pos)[2];
    }
  }
  // Filter points and get plane fitting candidates
  nr_candis = CompareZ(point_cloud, vx.indices_, sampled_z_values_, candi,
                       nr_points, nr_point_element, nr_samples);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FilterGrid";
  return nr_candis;
}

int PlaneFitGroundDetector::FilterLine(unsigned int r) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::FilterLine";
  int nr_candis = 0;
  unsigned int c = 0;
  const float *point_cloud = vg_fine_->const_data();
  unsigned int nr_points = vg_fine_->NrPoints();
  unsigned int nr_point_element = vg_fine_->NrPointElement();
  unsigned int begin = (r * param_.nr_grids_fine);
  int parent = 0;
  for (c = 0; c < param_.nr_grids_fine; c++) {
    parent = map_fine_to_coarse_[begin + c];
    nr_candis +=
        FilterGrid((*vg_fine_)(r, c), point_cloud, &local_candis_[0][parent],
                   nr_points, nr_point_element);
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FilterLine";
  return nr_candis;
}

int PlaneFitGroundDetector::Filter() {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::Filter";
  int nr_candis = 0;
  unsigned int i = 0;
  unsigned int r = 0;
  memset(reinterpret_cast<void *>(labels_), 0,
         vg_fine_->NrPoints() * sizeof(char));
  //  Clear candidate list
  for (i = 0; i < vg_coarse_->NrVoxel(); ++i) {
    local_candis_[0][i].Clear();
  }
  //  Filter plane fitting candidates
  for (r = 0; r < param_.nr_grids_fine; ++r) {
    nr_candis += FilterLine(r);
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Filter";
  return nr_candis;
}

int PlaneFitGroundDetector::FitGrid(const float *point_cloud,
                                    PlaneFitPointCandIndices *candi,
                                    GroundPlaneLiDAR *groundplane,
                                    unsigned int nr_points,
                                    unsigned int nr_point_element,
                                    float dist_thre) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::FitGrid";
  // initialize the best plane
  groundplane->ForceInvalid();
  // not enough samples, failed and return
  if (candi->Size() < param_.nr_inliers_min_threshold) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGrid";
  return 0;
  }
  GroundPlaneLiDAR plane;
  float ptp_dist = 0.0f;
  float fit_cost = 0.0f;
  float fit_cost_best = dist_thre;
  int nr_inliers = 0;
  int nr_inliers_best = -1;
  int i = 0;
  int j = 0;
  int rseed = I_DEFAULT_SEED;
  int indices_trial[] = {0, 0, 0};
  int nr_samples = candi->Prune(param_.nr_samples_min_threshold,
                                param_.nr_samples_max_threshold);
  int nr_inliers_termi = IRound(static_cast<float>(nr_samples) *
                                param_.termi_inlier_percen_threshold);
  // 3x3 matrix stores: x, y, z; x, y, z; x, y, z;
  float samples[9];
  // copy 3D points
  float *psrc = nullptr;
  float *pdst = pf_threeds_;
  for (i = 0; i < nr_samples; ++i) {
    assert((*candi)[i] < static_cast<int>(nr_points));
    ICopy3(point_cloud + (nr_point_element * (*candi)[i]), pdst);
    pdst += dim_point_;
  }
  // generate plane hypothesis and vote
  for (i = 0; i < param_.nr_ransac_iter_threshold; ++i) {
    IRandomSample(indices_trial, 3, nr_samples, &rseed);
    IScale3(indices_trial, dim_point_);
    ICopy3(pf_threeds_ + indices_trial[0], samples);
    ICopy3(pf_threeds_ + indices_trial[1], samples + 3);
    ICopy3(pf_threeds_ + indices_trial[2], samples + 6);
    IPlaneFitDestroyed(samples, plane.params);
    // check if the plane hypothesis has valid geometry
    if (plane.GetDegreeNormalToZ() > param_.planefit_orien_threshold) {
      continue;
    }
    // iterate samples and check if the point to plane distance is below
    // threshold
    psrc = pf_threeds_;
    nr_inliers = 0;
    fit_cost = 0;
    for (j = 0; j < nr_samples; ++j) {
      ptp_dist = IPlaneToPointDistanceWUnitNorm(plane.params, psrc);
      if (ptp_dist < dist_thre) {
        nr_inliers++;
        fit_cost += ptp_dist;
      }
      psrc += dim_point_;
    }
    // Assign number of supports
    plane.SetNrSupport(nr_inliers);

    fit_cost = nr_inliers > 0 ? (fit_cost / static_cast<float>(nr_inliers))
                              : dist_thre;
    // record the best plane
    if (nr_inliers >= nr_inliers_best) {
      if (nr_inliers == nr_inliers_best) {
        if (fit_cost < fit_cost_best) {
          nr_inliers_best = nr_inliers;
          fit_cost_best = fit_cost;
          *groundplane = plane;
        }
      } else {
        nr_inliers_best = nr_inliers;
        fit_cost_best = fit_cost;
        *groundplane = plane;
      }
      // found enough inliers - terminate the ransac
      if (nr_inliers_best > nr_inliers_termi) {
        break;
      }
    }
  }
  // check if meet the inlier number requirement
  if (!groundplane->IsValid()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGrid";
  return 0;
  }
  if (groundplane->GetNrSupport() <
      static_cast<int>(param_.nr_inliers_min_threshold)) {
    groundplane->ForceInvalid();
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGrid";
  return 0;
  }
  // iterate samples and check if the point to plane distance is within
  // threshold
  nr_inliers = 0;
  psrc = pf_threeds_;
  pdst = pf_threeds_;
  for (i = 0; i < nr_samples; ++i) {
    ptp_dist = IPlaneToPointDistanceWUnitNorm(groundplane->params, psrc);
    if (ptp_dist < dist_thre) {
      ICopy3(psrc, pdst);
      pdst += 3;
      nr_inliers++;
    }
    psrc += dim_point_;
  }
  groundplane->SetNrSupport(nr_inliers);
  // note that pf_threeds_ will be destroyed after calling this routine
  IPlaneFitTotalLeastSquare(pf_threeds_, groundplane->params, nr_inliers);
  // filtering: the best plane orientation is not valid*/
  // std::cout << groundplane->GetDegreeNormalToZ() << std::endl;
  if (groundplane->GetDegreeNormalToZ() > param_.planefit_orien_threshold) {
    groundplane->ForceInvalid();
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGrid";
  return 0;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGrid";
  return nr_inliers;
}

int PlaneFitGroundDetector::FitLine(unsigned int r) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::FitLine";
  int nr_grids = 0;
  unsigned int c = 0;
  GroundPlaneLiDAR gp;
  for (c = 0; c < param_.nr_grids_coarse; c++) {
    if (FitGrid(vg_coarse_->const_data(), &local_candis_[r][c], &gp,
                vg_coarse_->NrPoints(), vg_coarse_->NrPointElement(),
                pf_thresholds_[r][c]) >=
        static_cast<int>(param_.nr_inliers_min_threshold)) {
      // transform to polar coordinates and store:
      IPlaneEucliToSpher(gp, &ground_planes_sphe_[r][c]);
      ground_planes_[r][c] = gp;
      nr_grids++;
    } else {
      ground_planes_sphe_[r][c].ForceInvalid();
      ground_planes_[r][c].ForceInvalid();
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitLine";
  return nr_grids;
}

int PlaneFitGroundDetector::Fit() {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::Fit";
  int nr_grids = 0;
  for (unsigned int r = 0; r < param_.nr_grids_coarse; ++r) {
    nr_grids += FitLine(r);
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Fit";
  return nr_grids;
}

//  Filter candidates by neighbors
int PlaneFitGroundDetector::FilterCandidates(
    int r, int c, const float *point_cloud, PlaneFitPointCandIndices *candi,
    std::vector<std::pair<int, int> > *neighbors,
    unsigned int nr_point_element) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::FilterCandidates";
  float avg_z = 0.f;
  int count = 0;
  unsigned int i = 0;
  int r_n = 0;
  int c_n = 0;
  float z = 0.f;
  std::vector<int> filtered_indices;
  filtered_indices.reserve(candi->Size());
  for (i = 0; i < neighbors->size(); ++i) {
    r_n = (*neighbors)[i].first;
    c_n = (*neighbors)[i].second;
    if (ground_z_[r_n][c_n].second) {
      avg_z += ground_z_[r_n][c_n].first;
      count++;
    }
  }
  if (count > 0) {
    avg_z /= static_cast<float>(count);
    ground_z_[r][c].first = avg_z;
    ground_z_[r][c].second = true;
    for (i = 0; i < candi->Size(); ++i) {
      z = (point_cloud + (nr_point_element * (*candi)[i]))[2];
      if (z > avg_z - param_.candidate_filter_threshold &&
          z < avg_z + param_.candidate_filter_threshold) {
        filtered_indices.push_back((*candi)[i]);
      } else {
        labels_[(*candi)[i]] = 0;
      }
    }
    if (filtered_indices.size() != candi->Size()) {
      candi->indices.assign(filtered_indices.begin(), filtered_indices.end());
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FilterCandidates";
  return count;
}

inline float calculate_two_angles(const GroundPlaneLiDAR &p1,
                                  const GroundPlaneLiDAR &p2) {
AINFO<<"(DMCZP) EnteringMethod: calculate_two_angles";
  float numerator = IDot3(p1.params, p2.params);
  float denominator = IL2Norm(p1.params, 3) * IL2Norm(p2.params, 3);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: calculate_two_angles";
  return IAcos(numerator * IRec(denominator));
}

int PlaneFitGroundDetector::FitGridWithNeighbors(
    int r, int c, const float *point_cloud, GroundPlaneLiDAR *groundplane,
    unsigned int nr_points, unsigned int nr_point_element, float dist_thre) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::FitGridWithNeighbors";
  // initialize the best plane
  groundplane->ForceInvalid();
  // not enough samples, failed and return

  PlaneFitPointCandIndices &candi = local_candis_[r][c];
  std::vector<std::pair<int, int> > neighbors;
  GetNeighbors(r, c, param_.nr_grids_coarse, param_.nr_grids_coarse,
               &neighbors);
  FilterCandidates(r, c, point_cloud, &candi, &neighbors, nr_point_element);

  if (candi.Size() < param_.nr_inliers_min_threshold) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGridWithNeighbors";
  return 0;
  }

  GroundPlaneLiDAR plane;
  int kNr_iter =
      param_.nr_ransac_iter_threshold + static_cast<int>(neighbors.size());
  //  check hypothesis initialized correct or not
  if (kNr_iter < 1) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGridWithNeighbors";
  return 0;
  }
  GroundPlaneLiDAR hypothesis[kNr_iter];

  float ptp_dist = 0.0f;
  int best = 0;
  int nr_inliers = 0;
  int nr_inliers_best = -1;
  float angle_best = FLT_MAX;

  int rseed = I_DEFAULT_SEED;
  int indices_trial[] = {0, 0, 0};
  int nr_samples = candi.Prune(param_.nr_samples_min_threshold,
                               param_.nr_samples_max_threshold);
  int nr_inliers_termi = IRound(static_cast<float>(nr_samples) *
                                param_.termi_inlier_percen_threshold);
  // 3x3 matrix stores: x, y, z; x, y, z; x, y, z;
  float samples[9];
  // copy 3D points
  float *psrc = nullptr;
  float *pdst = pf_threeds_;
  int r_n = 0;
  int c_n = 0;
  float angle = -1.f;
  for (int i = 0; i < nr_samples; ++i) {
    assert(candi[i] < static_cast<int>(nr_points));
    ICopy3(point_cloud + (nr_point_element * candi[i]), pdst);
    pdst += dim_point_;
  }
  // generate plane hypothesis and vote
  for (int i = 0; i < param_.nr_ransac_iter_threshold; ++i) {
    IRandomSample(indices_trial, 3, nr_samples, &rseed);
    IScale3(indices_trial, dim_point_);
    ICopy3(pf_threeds_ + indices_trial[0], samples);
    ICopy3(pf_threeds_ + indices_trial[1], samples + 3);
    ICopy3(pf_threeds_ + indices_trial[2], samples + 6);
    IPlaneFitDestroyed(samples, hypothesis[i].params);
    // check if the plane hypothesis has valid geometry
    if (hypothesis[i].GetDegreeNormalToZ() > param_.planefit_orien_threshold) {
      continue;
    }
    // iterate samples and check if the point to plane distance is below
    // threshold
    psrc = pf_threeds_;
    nr_inliers = 0;
    for (int j = 0; j < nr_samples; ++j) {
      ptp_dist = IPlaneToPointDistanceWUnitNorm(hypothesis[i].params, psrc);
      if (ptp_dist < dist_thre) {
        nr_inliers++;
      }
      psrc += dim_point_;
    }
    // Assign number of supports
    hypothesis[i].SetNrSupport(nr_inliers);

    if (nr_inliers > nr_inliers_termi) {
      break;
    }
  }

  for (size_t i = 0; i < neighbors.size(); ++i) {
    r_n = neighbors[i].first;
    c_n = neighbors[i].second;
    if (ground_planes_[r_n][c_n].IsValid()) {
      hypothesis[i + param_.nr_ransac_iter_threshold] =
          ground_planes_[r_n][c_n];
      psrc = pf_threeds_;
      nr_inliers = 0;
      for (int j = 0; j < nr_samples; ++j) {
        ptp_dist = IPlaneToPointDistanceWUnitNorm(
            hypothesis[i + param_.nr_ransac_iter_threshold].params, psrc);
        if (ptp_dist < dist_thre) {
          nr_inliers++;
        }
        psrc += dim_point_;
      }
      if (nr_inliers < static_cast<int>(param_.nr_inliers_min_threshold)) {
        hypothesis[i + param_.nr_ransac_iter_threshold].ForceInvalid();
        continue;
      }
      hypothesis[i + param_.nr_ransac_iter_threshold].SetNrSupport(nr_inliers);
    }
  }

  nr_inliers_best = -1;
  for (int i = 0; i < kNr_iter; ++i) {
    if (!(hypothesis[i].IsValid())) {
      continue;
    }
    nr_inliers = hypothesis[i].GetNrSupport();
    if (nr_inliers >= nr_inliers_best) {
      if (nr_inliers == nr_inliers_best) {
        angle = CalculateAngleDist(hypothesis[i], neighbors);
        if (angle < angle_best && angle > 0) {
          angle_best = angle;
          best = i;
        }
      } else {
        nr_inliers_best = nr_inliers;
        angle_best = CalculateAngleDist(hypothesis[i], neighbors);
        best = i;
      }
    }
  }

  *groundplane = hypothesis[best];

  // check if meet the inlier number requirement
  if (!groundplane->IsValid()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGridWithNeighbors";
  return 0;
  }
  if (groundplane->GetNrSupport() <
      static_cast<int>(param_.nr_inliers_min_threshold)) {
    groundplane->ForceInvalid();
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGridWithNeighbors";
  return 0;
  }
  // iterate samples and check if the point to plane distance is within
  // threshold
  nr_inliers = 0;
  psrc = pf_threeds_;
  pdst = pf_threeds_;
  for (int i = 0; i < nr_samples; ++i) {
    ptp_dist = IPlaneToPointDistanceWUnitNorm(groundplane->params, psrc);
    if (ptp_dist < dist_thre) {
      ICopy3(psrc, pdst);
      pdst += 3;
      nr_inliers++;
    }
    psrc += dim_point_;
  }
  groundplane->SetNrSupport(nr_inliers);

  // note that pf_threeds_ will be destroyed after calling this routine
  IPlaneFitTotalLeastSquare(pf_threeds_, groundplane->params, nr_inliers);
  if (angle_best <= CalculateAngleDist(*groundplane, neighbors)) {
    *groundplane = hypothesis[best];
    groundplane->SetStatus(true);
  }

  if (groundplane->GetDegreeNormalToZ() > param_.planefit_orien_threshold) {
    groundplane->ForceInvalid();
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGridWithNeighbors";
  return 0;
  }

  const auto &voxel_cur = (*vg_coarse_)(r, c);
  float radius = voxel_cur.dim_x_ * 0.5f;
  float cx = voxel_cur.v_[0] + radius;
  float cy = voxel_cur.v_[1] + radius;
  float cz = -(groundplane->params[0] * cx + groundplane->params[1] * cy +
               groundplane->params[3]) /
             groundplane->params[2];
  ground_z_[r][c].first = cz;
  ground_z_[r][c].second = true;

  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitGridWithNeighbors";
  return nr_inliers;
}

float PlaneFitGroundDetector::CalculateAngleDist(
    const GroundPlaneLiDAR &plane,
    const std::vector<std::pair<int, int> > &neighbors) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::CalculateAngleDist";
  float angle_dist = 0.0f;
  int count = 0;
  unsigned int j = 0;
  int r_n = 0;
  int c_n = 0;
  for (j = 0; j < neighbors.size(); ++j) {
    r_n = neighbors[j].first;
    c_n = neighbors[j].second;
    if (ground_planes_[r_n][c_n].IsValid()) {
      angle_dist += calculate_two_angles(ground_planes_[r_n][c_n], plane);
      count++;
    }
  }
  if (count == 0) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::CalculateAngleDist";
  return -1;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::CalculateAngleDist";
  return angle_dist / static_cast<float>(count);
}

int PlaneFitGroundDetector::FitInOrder() {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::FitInOrder";
  int nr_grids = 0;
  unsigned int i = 0;
  unsigned int j = 0;
  int r = 0;
  int c = 0;
  GroundPlaneLiDAR gp;
  for (i = 0; i < param_.nr_grids_coarse; ++i) {
    for (j = 0; j < param_.nr_grids_coarse; ++j) {
      ground_z_[i][j].first = 0.f;
      ground_z_[i][j].second = false;
    }
  }
  for (i = 0; i < vg_coarse_->NrVoxel(); ++i) {
    r = order_table_[i].first;
    c = order_table_[i].second;
    if (FitGridWithNeighbors(
            r, c, vg_coarse_->const_data(), &gp, vg_coarse_->NrPoints(),
            vg_coarse_->NrPointElement(), pf_thresholds_[r][c]) >=
        static_cast<int>(param_.nr_inliers_min_threshold)) {
      IPlaneEucliToSpher(gp, &ground_planes_sphe_[r][c]);
      ground_planes_[r][c] = gp;
      nr_grids++;
    } else {
      ground_planes_sphe_[r][c].ForceInvalid();
      ground_planes_[r][c].ForceInvalid();
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::FitInOrder";
  return nr_grids;
}

void PlaneFitGroundDetector::GetNeighbors(
    int r, int c, int rows, int cols,
    std::vector<std::pair<int, int> > *neighbors) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::GetNeighbors";
  int left = IMax(0, c - 1);
  int right = IMin(cols - 1, c + 1);
  int up = IMax(0, r - 1);
  int down = IMin(rows - 1, r + 1);
  int x = 0;
  int y = 0;
  neighbors->reserve(8);
  for (x = left; x <= right; ++x) {
    for (y = up; y <= down; ++y) {
      if (x == c && y == r) {
        continue;
      }
      neighbors->push_back(std::pair<int, int>(y, x));
    }
  }
}

int PlaneFitGroundDetector::SmoothLine(unsigned int up, unsigned int r,
                                       unsigned int dn) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::SmoothLine";
  int nr_grids = 0;
  unsigned int c = 0;
  unsigned int nm1 = param_.nr_grids_coarse - 1;
  GroundPlaneSpherical plane;
  assert(param_.nr_grids_coarse >= 2);
  assert(up < param_.nr_grids_coarse);
  assert(r < param_.nr_grids_coarse);
  assert(dn < param_.nr_grids_coarse);
  if (/*!(*_vg_coarse)(r, 0).empty()*/ true) {
    if (!ground_planes_sphe_[r][0].IsValid()) {
      nr_grids += CompleteGrid(
          ground_planes_sphe_[r][0], ground_planes_sphe_[r][1],
          ground_planes_sphe_[up][0], ground_planes_sphe_[dn][0], &plane);
    } else {
      nr_grids +=
          SmoothGrid(ground_planes_sphe_[r][0], ground_planes_sphe_[r][0],
                     ground_planes_sphe_[r][1], ground_planes_sphe_[up][0],
                     ground_planes_sphe_[dn][0], &plane);
    }
    IPlaneSpherToEucli(plane, &ground_planes_[r][0]);
  }
  for (c = 1; c < nm1; ++c) {
    if (/*!(*_vg_coarse)(r, c).empty()*/ true) {
      if (!ground_planes_sphe_[r][c].IsValid()) {
        nr_grids += CompleteGrid(
            ground_planes_sphe_[r][c - 1], ground_planes_sphe_[r][c + 1],
            ground_planes_sphe_[up][c], ground_planes_sphe_[dn][c], &plane);
      } else {
        nr_grids += SmoothGrid(
            ground_planes_sphe_[r][c], ground_planes_sphe_[r][c - 1],
            ground_planes_sphe_[r][c + 1], ground_planes_sphe_[up][c],
            ground_planes_sphe_[dn][c], &plane);
      }
      IPlaneSpherToEucli(plane, &ground_planes_[r][c]);
    }
  }
  if (/*!(*_vg_coarse)(r, nm1).empty()*/ true) {
    if (!ground_planes_sphe_[r][nm1].IsValid()) {
      nr_grids += CompleteGrid(
          ground_planes_sphe_[r][nm1 - 1], ground_planes_sphe_[r][nm1],
          ground_planes_sphe_[up][nm1], ground_planes_sphe_[dn][nm1], &plane);
    } else {
      nr_grids += SmoothGrid(
          ground_planes_sphe_[r][nm1], ground_planes_sphe_[r][nm1 - 1],
          ground_planes_sphe_[r][nm1], ground_planes_sphe_[up][nm1],
          ground_planes_sphe_[dn][nm1], &plane);
    }
    IPlaneSpherToEucli(plane, &ground_planes_[r][nm1]);
  }

  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::SmoothLine";
  return nr_grids;
}

int PlaneFitGroundDetector::CompleteGrid(const GroundPlaneSpherical &lt,
                                         const GroundPlaneSpherical &rt,
                                         const GroundPlaneSpherical &up,
                                         const GroundPlaneSpherical &dn,
                                         GroundPlaneSpherical *gp) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::CompleteGrid";
  int supports[] = {0, 0, 0, 0};
  float weights[] = {0.f, 0.f, 0.f, 0.f};
  gp->ForceInvalid();
  supports[0] = lt.GetNrSupport();
  supports[1] = rt.GetNrSupport();
  supports[2] = up.GetNrSupport();
  supports[3] = dn.GetNrSupport();
  int support_sum = 0;
  support_sum = ISum4(supports);
  if (!support_sum) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::CompleteGrid";
  return 0;
  }
  weights[0] =
      static_cast<float>(supports[0]) / static_cast<float>(support_sum);
  weights[1] =
      static_cast<float>(supports[1]) / static_cast<float>(support_sum);
  weights[2] =
      static_cast<float>(supports[2]) / static_cast<float>(support_sum);
  weights[3] =
      static_cast<float>(supports[3]) / static_cast<float>(support_sum);
  // weighted average:
  gp->theta = weights[0] * lt.theta + weights[1] * rt.theta +
              weights[2] * up.theta + weights[3] * dn.theta;
  gp->phi = weights[0] * lt.phi + weights[1] * rt.phi + weights[2] * up.phi +
            weights[3] * dn.phi;
  gp->d = weights[0] * lt.d + weights[1] * rt.d + weights[2] * up.d +
          weights[3] * dn.d;
  // compute average - diveided by 4, round to nearest int
  support_sum = IMax(((support_sum + 2) >> 2), 1);
  gp->SetNrSupport(support_sum);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::CompleteGrid";
  return 1;
}

int PlaneFitGroundDetector::SmoothGrid(const GroundPlaneSpherical &g,
                                       const GroundPlaneSpherical &lt,
                                       const GroundPlaneSpherical &rt,
                                       const GroundPlaneSpherical &up,
                                       const GroundPlaneSpherical &dn,
                                       GroundPlaneSpherical *gp) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::SmoothGrid";
  int supports[] = {0, 0, 0, 0, 0};
  float weights[] = {0.f, 0.f, 0.f, 0.f, 0.f};
  gp->ForceInvalid();
  if (!g.IsValid()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::SmoothGrid";
  return 0;
  }
  //  geometry weight:
  //   1
  // 1 4 1
  //   1
  if (lt.GetStatus()) {
    supports[0] = lt.GetNrSupport();
  }
  if (rt.GetStatus()) {
    supports[1] = rt.GetNrSupport();
  }
  if (up.GetStatus()) {
    supports[2] = up.GetNrSupport();
  }
  if (dn.GetStatus()) {
    supports[3] = dn.GetNrSupport();
  }
  supports[4] = (g.GetNrSupport() << 2);
  int support_sum = 0;
  support_sum = ISum4(supports);
  if (support_sum == 0) {
    *gp = g;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::SmoothGrid";
  return 0;
  }
  support_sum += supports[4];
  weights[0] =
      static_cast<float>(supports[0]) / static_cast<float>(support_sum);
  weights[1] =
      static_cast<float>(supports[1]) / static_cast<float>(support_sum);
  weights[2] =
      static_cast<float>(supports[2]) / static_cast<float>(support_sum);
  weights[3] =
      static_cast<float>(supports[3]) / static_cast<float>(support_sum);
  weights[4] =
      static_cast<float>(supports[4]) / static_cast<float>(support_sum);
  // weighted average:
  gp->theta = weights[0] * lt.theta + weights[1] * rt.theta +
              weights[2] * up.theta + weights[3] * dn.theta +
              weights[4] * g.theta;
  gp->phi = weights[0] * lt.phi + weights[1] * rt.phi + weights[2] * up.phi +
            weights[3] * dn.phi + weights[4] * g.phi;
  gp->d = weights[0] * lt.d + weights[1] * rt.d + weights[2] * up.d +
          weights[3] * dn.d + weights[4] * g.d;
  // compute weighted average - diveided by 8, round to nearest int
  support_sum = IMax(((support_sum + 2) >> 3), 1);
  gp->SetNrSupport(support_sum);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::SmoothGrid";
  return 1;
}

int PlaneFitGroundDetector::Smooth() {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::Smooth";
  int nr_grids = 0;
  unsigned int r = 0;
  unsigned int c = 0;
  unsigned int nm1 = param_.nr_grids_coarse - 1;
  assert(param_.nr_grids_coarse >= 2);
  nr_grids += SmoothLine(0, 0, 1);
  for (r = 1; r < nm1; ++r) {
    nr_grids += SmoothLine(r - 1, r, r + 1);
  }
  nr_grids += SmoothLine(nm1 - 1, nm1, nm1);
  for (r = 0; r < param_.nr_grids_coarse; ++r) {
    for (c = 0; c < param_.nr_grids_coarse; ++c) {
      IPlaneEucliToSpher(ground_planes_[r][c], &ground_planes_sphe_[r][c]);
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Smooth";
  return nr_grids;
}

bool PlaneFitGroundDetector::Detect(const float *point_cloud,
                                    float *height_above_ground,
                                    unsigned int nr_points,
                                    unsigned int nr_point_elements) {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::Detect";
  assert(point_cloud != nullptr);
  assert(height_above_ground != nullptr);
  assert(nr_points <= param_.nr_points_max);
  assert(nr_point_elements >= 3);
  // setup the fine voxel grid
  if (!vg_fine_->SetS(point_cloud, nr_points, nr_point_elements)) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Detect";
  return false;
  }
  // setup the coarse voxel grid
  if (!vg_coarse_->SetS(point_cloud, nr_points, nr_point_elements)) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Detect";
  return false;
  }
  // int nr_candis = 0;
  // int nr_valid_grid = 0;
  unsigned int r = 0;
  unsigned int c = 0;
  // Filter to generate plane fitting candidates
  // int nr_candis =
  Filter();
  // std::cout << "# of plane candidates: " << nr_candis << std::endl;
  //  Fit local plane using ransac
  // nr_valid_grid = Fit();
  // int nr_valid_grid =
  FitInOrder();
  // std::cout << "# of valid plane geometry (fitting): " << nr_valid_grid <<
  // std::endl;
  // Smooth plane using neighborhood information:
  for (int iter = 0; iter < param_.nr_smooth_iter; ++iter) {
    Smooth();
  }

  for (r = 0; r < param_.nr_grids_coarse; ++r) {
    for (c = 0; c < param_.nr_grids_coarse; ++c) {
      if ((*vg_coarse_)(r, c).Empty()) {
        ground_planes_[r][c].ForceInvalid();
      }
    }
  }

  // std::cout << "# of valid plane geometry (Smooth): " << nr_valid_grid <<
  // std::endl;
  // compute point to ground distance
  ComputeSignedGroundHeight(point_cloud, height_above_ground, nr_points,
                            nr_point_elements);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::Detect";
  return true;
}

const char *PlaneFitGroundDetector::GetLabel() const {
  AINFO<<"(DMCZP) EnteringMethod: *PlaneFitGroundDetector::GetLabel";
 
  AINFO<<"(DMCZ
  AINFO<<"(DMCZP) LeaveMethod: *PlaneFitGroundDetector::GetLabel";
 P) (return) LeaveMethod: *PlaneFitGroundDetector::GetLabel";
  return labels_; }

const VoxelGridXY<float> *PlaneFitGroundDetector::GetGrid() const {
  return vg_coarse_;
}

const GroundPlaneLiDAR *PlaneFitGroundDetector::GetGroundPlane(int r,
                                                               int c) const {
AINFO<<"(DMCZP) EnteringMethod: *PlaneFitGroundDetector::GetGroundPlane";
  assert(r >= 0 && r < static_cast<int>(param_.nr_grids_coarse));
  assert(c >= 0 && c < static_cast<int>(param_.nr_grids_coarse));
  
  AINFO<<"(DMCZP) (return) LeaveMethod: *PlaneFitGroundDetector::GetGroundPlane";
  return ground_planes_ != nullptr ? ground_planes_[r] + c : nullptr;
}

const unsigned int PlaneFitGroundDetector::GetGridDimX() const {
AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::GetGridDimX";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::GetGridDimX";
  return vg_coarse_->NrVoxelX();
}

const unsigned int PlaneFitGroundDetector::GetGridDimY() const {
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PlaneFitGroundDetector::GetGridDimY";
  return vg_coarse_->NrVoxelY();
}

float PlaneFitGroundDetector::GetUnknownHeight() {
  AINFO<<"(DMCZP) EnteringMethod: PlaneFitGroundDetector::GetUnknownHeight";
 
  AINFO<<"(DMCZ
  AINFO<<"(DMCZP) LeaveMethod: PlaneFitGroundDetector::GetUnknownHeight";
 P) (return) LeaveMethod: PlaneFitGroundDetector::GetUnknownHeight";
  return FLT_MAX; }

PlaneFitPointCandIndices **PlaneFitGroundDetector::GetCandis() const {
AINFO<<"(DMCZP) EnteringMethod: **PlaneFitGroundDetector::GetCandis";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: **PlaneFitGroundDetector::GetCandis";
  return local_candis_;
}

void IPlaneEucliToSpher(const GroundPlaneLiDAR &src,
                        GroundPlaneSpherical *dst) {
AINFO<<"(DMCZP) EnteringMethod: IPlaneEucliToSpher";
  if (!src.IsValid()) {
    dst->ForceInvalid();
  } else {
    GroundPlaneLiDAR p = src;
    assert(p.params[2] != 0 || p.params[1] != 0);
    p.ForceUnitNorm();  // to be safe
    p.ForcePositiveNormalZ();
    dst->theta = IAcos(p.params[0]);
    dst->phi = IAtan2(p.params[1], p.params[2]);
    dst->d = p.params[3];
    dst->SetNrSupport(src.GetNrSupport());
    dst->SetStatus(src.GetStatus());
  }
}

void IPlaneSpherToEucli(const GroundPlaneSpherical &src,
                        GroundPlaneLiDAR *dst) {
AINFO<<"(DMCZP) EnteringMethod: IPlaneSpherToEucli";
  if (!src.IsValid()) {
    dst->ForceInvalid();
  } else {
    // assume positive nz;
    // ny = nz * ny_over_nz;
    // nx * nx + ny * ny + nz * nz = 1.0;
    // nz^2 + nz^2*ny_over_nz^2 = 1 - nx^2;
    // nz^2(1 + ny_over_nz^2) = 1 - nx^2;
    // nz = sqrt((1-nx^2)/(1 + ny_over_nz^2))
    // nz is positive, guaranteed
    float nx = ICos(src.theta);
    float ny_over_nz = ITan(src.phi);
    float nz = ISqrt((1 - nx * nx) / (1 + ny_over_nz * ny_over_nz));
    float ny = nz * ny_over_nz;
    dst->params[0] = nx;
    dst->params[1] = ny;
    dst->params[2] = nz;
    dst->params[3] = src.d;
    dst->SetNrSupport(src.GetNrSupport());
    dst->SetStatus(src.GetStatus());
  }
}
} /*namespace common*/
} /*namespace perception*/
} /*namespace apollo*/
