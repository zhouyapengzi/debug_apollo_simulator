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
#include "modules/perception/lidar/common/cloud_mask.h"

namespace apollo {
namespace perception {
namespace lidar {

using base::AttributePointCloud;
using base::PointF;

size_t CloudMask::ValidIndicesCount() const {
AINFO<<"(DMCZP) EnteringMethod: CloudMask::ValidIndicesCount";
  size_t count = 0;
  for (auto& i : mask_) {
    if (i > 0) {
      ++count;
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: CloudMask::ValidIndicesCount";
  return count;

  AINFO<<"(DMCZP) LeaveMethod: CloudMask::ValidIndicesCount";
 }

void CloudMask::GetValidCloud(const AttributePointCloud<PointF>& source_cloud,
                              AttributePointCloud<PointF>* target_cloud) const {
AINFO<<"(DMCZP) EnteringMethod: CloudMask::GetValidCloud";
  if (target_cloud == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: CloudMask::GetValidCloud";
  return;
  }
  indices_.clear();
  indices_.reserve(mask_.size());
  for (size_t i = 0; i < mask_.size(); ++i) {
    if (mask_[i] > 0) {
      indices_.push_back(static_cast<int>(i));
    }
  }
  target_cloud->CopyPointCloud(source_cloud, indices_);

  AINFO<<"(DMCZP) LeaveMethod: CloudMask::GetValidCloud";
 }

void CloudMask::Flip() {
AINFO<<"(DMCZP) EnteringMethod: CloudMask::Flip";
  for (auto& i : mask_) {
    i = i > 0 ? 0 : 1;
  }

  AINFO<<"(DMCZP) LeaveMethod: CloudMask::Flip";
 }

void CloudMask::AddIndices(const base::PointIndices& indices, int value) {
AINFO<<"(DMCZP) EnteringMethod: CloudMask::AddIndices";
  AddIndices(indices.indices, value);

  AINFO<<"(DMCZP) LeaveMethod: CloudMask::AddIndices";
 }

void CloudMask::AddIndicesOfIndices(
    const base::PointIndices& indices,
    const base::PointIndices& indices_of_indices, int value) {
AINFO<<"(DMCZP) EnteringMethod: CloudMask::AddIndicesOfIndices";
  for (auto& id : indices_of_indices.indices) {
    mask_[indices.indices[id]] = value;
  }

  AINFO<<"(DMCZP) LeaveMethod: CloudMask::AddIndicesOfIndices";
 }

void CloudMask::RemoveIndices(const base::PointIndices& indices) {
AINFO<<"(DMCZP) EnteringMethod: CloudMask::RemoveIndices";
  RemoveIndices(indices.indices);

  AINFO<<"(DMCZP) LeaveMethod: CloudMask::RemoveIndices";
 }

void CloudMask::RemoveIndicesOfIndices(
    const base::PointIndices& indices,
    const base::PointIndices& indices_of_indices) {
AINFO<<"(DMCZP) EnteringMethod: CloudMask::RemoveIndicesOfIndices";
  for (auto& id : indices_of_indices.indices) {
    mask_[indices.indices[id]] = 0;
  }

  AINFO<<"(DMCZP) LeaveMethod: CloudMask::RemoveIndicesOfIndices";
 }

void CloudMask::GetValidMask(CloudMask* rhs) const {
AINFO<<"(DMCZP) EnteringMethod: CloudMask::GetValidMask";
  if (rhs == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: CloudMask::GetValidMask";
  return;
  }
  rhs->clear();
  for (const auto& i : mask_) {
    if (i > 0) {
      rhs->mask_.push_back(i);
    }
  }

  AINFO<<"(DMCZP) LeaveMethod: CloudMask::GetValidMask";
 }

void CloudMask::ResetValue(int source_value, int target_value) {
AINFO<<"(DMCZP) EnteringMethod: CloudMask::ResetValue";
  for (auto& i : mask_) {
    if (i == source_value) {
      i = target_value;
    }
  }

  AINFO<<"(DMCZP) LeaveMethod: CloudMask::ResetValue";
 }

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
