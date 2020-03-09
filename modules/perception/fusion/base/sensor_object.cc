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
#include "modules/perception/fusion/base/sensor_object.h"

#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

// SensorObject implementations
SensorObject::SensorObject(
    const std::shared_ptr<const base::Object>& object_ptr)
    : object_(object_ptr), frame_header_(nullptr) {
  AINFO<<"(DMCZP) EnteringMethod: SensorObject::SensorObject";

  AINFO<<"(DMCZP) LeaveMethod: SensorObject::SensorObject";
 
  AINFO<<"(DMCZP) EnteringMethod: SensorObject::SensorObject";

  AINFO<<"(DMCZP) LeaveMethod: SensorObject::SensorObject";
 
  AINFO<<"(DMCZP) EnteringMethod: SensorObject::SensorObject";

  AINFO<<"(DMCZP) LeaveMethod: SensorObject::SensorObject";
 }

SensorObject::SensorObject(
    const std::shared_ptr<const base::Object>& object_ptr,
    const std::shared_ptr<const SensorFrameHeader>& frame_header)
    : object_(object_ptr), frame_header_(frame_header) {}

SensorObject::SensorObject(
    const std::shared_ptr<const base::Object>& object_ptr,
    const std::shared_ptr<SensorFrame>& frame_ptr)
    : object_(object_ptr),
      frame_header_((frame_ptr == nullptr) ? nullptr : frame_ptr->GetHeader()) {
}

double SensorObject::GetTimestamp() const {
AINFO<<"(DMCZP) EnteringMethod: SensorObject::GetTimestamp";
  if (frame_header_ == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorObject::GetTimestamp";
  return 0.0;
  }

  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorObject::GetTimestamp";
  return frame_header_->timestamp;

  AINFO<<"(DMCZP) LeaveMethod: SensorObject::GetTimestamp";
 }

bool SensorObject::GetRelatedFramePose(Eigen::Affine3d* pose) const {
AINFO<<"(DMCZP) EnteringMethod: SensorObject::GetRelatedFramePose";
  CHECK_NOTNULL(pose);
  if (frame_header_ == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorObject::GetRelatedFramePose";
  return false;
  }

  *pose = frame_header_->sensor2world_pose;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorObject::GetRelatedFramePose";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: SensorObject::GetRelatedFramePose";
 }

std::string SensorObject::GetSensorId() const {
AINFO<<"(DMCZP) EnteringMethod: SensorObject::GetSensorId";
  if (frame_header_ == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorObject::GetSensorId";
  return std::string("");
  }

  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorObject::GetSensorId";
  return frame_header_->sensor_info.name;

  AINFO<<"(DMCZP) LeaveMethod: SensorObject::GetSensorId";
 }

base::SensorType SensorObject::GetSensorType() const {
AINFO<<"(DMCZP) EnteringMethod: SensorObject::GetSensorType";
  if (frame_header_ == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorObject::GetSensorType";
  return base::SensorType::UNKNOWN_SENSOR_TYPE;
  }

  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorObject::GetSensorType";
  return frame_header_->sensor_info.type;

  AINFO<<"(DMCZP) LeaveMethod: SensorObject::GetSensorType";
 }

// FusedObject implementations
FusedObject::FusedObject() {
AINFO<<"(DMCZP) EnteringMethod: FusedObject::FusedObject";
  base::ObjectPool& object_pool = base::ObjectPool::Instance();
  object_ = object_pool.Get();

  AINFO<<"(DMCZP) LeaveMethod: FusedObject::FusedObject";
 }

bool IsLidar(const SensorObjectConstPtr& obj) {
AINFO<<"(DMCZP) EnteringMethod: IsLidar";
  base::SensorType type = obj->GetSensorType();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: IsLidar";
  return common::SensorManager::Instance()->IsLidar(type);

  AINFO<<"(DMCZP) LeaveMethod: IsLidar";
 }

bool IsRadar(const SensorObjectConstPtr& obj) {
AINFO<<"(DMCZP) EnteringMethod: IsRadar";
  base::SensorType type = obj->GetSensorType();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: IsRadar";
  return common::SensorManager::Instance()->IsRadar(type);

  AINFO<<"(DMCZP) LeaveMethod: IsRadar";
 }

bool IsCamera(const SensorObjectConstPtr& obj) {
AINFO<<"(DMCZP) EnteringMethod: IsCamera";
  base::SensorType type = obj->GetSensorType();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: IsCamera";
  return common::SensorManager::Instance()->IsCamera(type);

  AINFO<<"(DMCZP) LeaveMethod: IsCamera";
 }

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
