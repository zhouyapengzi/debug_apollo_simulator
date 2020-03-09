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
#include "modules/perception/fusion/base/sensor_data_manager.h"

#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace fusion {

SensorDataManager::SensorDataManager() {
  AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::SensorDataManager";
 CHECK_EQ(this->Init(), true); 
  AINFO<<"(DMCZP) LeaveMethod: SensorDataManager::SensorDataManager";
 }

bool SensorDataManager::Init() {
AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::Init";
  if (inited_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::Init";
  return true;
  }
  sensor_manager_ = common::SensorManager::Instance();
  inited_ = true;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::Init";
  return true;
}

void SensorDataManager::Reset() {
AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::Reset";
  inited_ = false;
  sensor_manager_ = nullptr;
  sensors_.clear();

  AINFO<<"(DMCZP) LeaveMethod: SensorDataManager::Reset";
 }

void SensorDataManager::AddSensorMeasurements(
    const base::FrameConstPtr& frame_ptr) {
AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::AddSensorMeasurements";
  const base::SensorInfo& sensor_info = frame_ptr->sensor_info;
  std::string sensor_id = sensor_info.name;
  const auto it = sensors_.find(sensor_id);
  SensorPtr sensor_ptr = nullptr;
  if (it == sensors_.end()) {
    if (!sensor_manager_->IsSensorExist(sensor_id)) {
      AERROR << "Failed to find sensor " << sensor_id << " in sensor manager.";
      
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::AddSensorMeasurements";
  return;
    }
    sensor_ptr = std::make_shared<Sensor>(Sensor(sensor_info));
    sensors_.emplace(sensor_id, sensor_ptr);
  } else {
    sensor_ptr = it->second;
  }

  sensor_ptr->AddFrame(frame_ptr);

  AINFO<<"(DMCZP) LeaveMethod: SensorDataManager::AddSensorMeasurements";
 }

bool SensorDataManager::IsLidar(const base::FrameConstPtr& frame_ptr) {
AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::IsLidar";
  base::SensorType type = frame_ptr->sensor_info.type;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::IsLidar";
  return sensor_manager_->IsLidar(type);
}

bool SensorDataManager::IsRadar(const base::FrameConstPtr& frame_ptr) {
AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::IsRadar";
  base::SensorType type = frame_ptr->sensor_info.type;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::IsRadar";
  return sensor_manager_->IsRadar(type);
}

bool SensorDataManager::IsCamera(const base::FrameConstPtr& frame_ptr) {
AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::IsCamera";
  base::SensorType type = frame_ptr->sensor_info.type;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::IsCamera";
  return sensor_manager_->IsCamera(type);
}

void SensorDataManager::GetLatestSensorFrames(
    double timestamp, const std::string& sensor_id,
    std::vector<SensorFramePtr>* frames) const {
AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::GetLatestSensorFrames";
  if (frames == nullptr) {
    AERROR << "Nullptr error.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::GetLatestSensorFrames";
  return;
  }
  const auto it = sensors_.find(sensor_id);
  if (it == sensors_.end()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::GetLatestSensorFrames";
  return;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::GetLatestSensorFrames";
  return it->second->QueryLatestFrames(timestamp, frames);
}

void SensorDataManager::GetLatestFrames(
    double timestamp, std::vector<SensorFramePtr>* frames) const {
AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::GetLatestFrames";
  if (frames == nullptr) {
    AERROR << "Nullptr error.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::GetLatestFrames";
  return;
  }

  frames->clear();
  for (auto it = sensors_.begin(); it != sensors_.end(); ++it) {
    SensorFramePtr frame = it->second->QueryLatestFrame(timestamp);
    if (frame != nullptr) {
      frames->push_back(frame);
    }
  }

  if (frames->empty()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::GetLatestFrames";
  return;
  }

  for (size_t i = 0; i < frames->size() - 1; ++i) {
    for (size_t j = i + 1; j < frames->size(); ++j) {
      if ((*frames)[j]->GetTimestamp() < (*frames)[i]->GetTimestamp()) {
        std::swap((*frames)[j], (*frames)[i]);
      }
    }
  }

  AINFO<<"(DMCZP) LeaveMethod: SensorDataManager::GetLatestFrames";
 }

bool SensorDataManager::GetPose(const std::string& sensor_id, double timestamp,
                                Eigen::Affine3d* pose) const {
AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::GetPose";
  if (pose == nullptr) {
    AERROR << "Nullptr error.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::GetPose";
  return false;
  }

  const auto it = sensors_.find(sensor_id);
  if (it == sensors_.end()) {
    AERROR << "Failed to find sensor " << sensor_id << " for get pose.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::GetPose";
  return false;
  }

  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::GetPose";
  return it->second->GetPose(timestamp, pose);
}

base::BaseCameraModelPtr SensorDataManager::GetCameraIntrinsic(
    const std::string& sensor_id) const {
AINFO<<"(DMCZP) EnteringMethod: SensorDataManager::GetCameraIntrinsic";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorDataManager::GetCameraIntrinsic";
  return sensor_manager_->GetUndistortCameraModel(sensor_id);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
