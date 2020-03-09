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
#include "modules/perception/common/sensor_manager/sensor_manager.h"

#include <utility>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/proto/sensor_meta_schema.pb.h"

namespace apollo {
namespace perception {
namespace common {

using apollo::cyber::common::GetProtoFromASCIIFile;
using apollo::perception::base::BrownCameraDistortionModel;
using apollo::perception::base::SensorInfo;
using apollo::perception::base::SensorOrientation;
using apollo::perception::base::SensorType;

SensorManager::SensorManager() {
  AINFO<<"(DMCZP) EnteringMethod: SensorManager::SensorManager";
 CHECK_EQ(this->Init(), true); 
  AINFO<<"(DMCZP) LeaveMethod: SensorManager::SensorManager";
 }

bool SensorManager::Init() {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::Init";
  std::lock_guard<std::mutex> lock(mutex_);
  if (inited_) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::Init";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::Init";
  return true;
  }

  sensor_info_map_.clear();
  distort_model_map_.clear();
  undistort_model_map_.clear();

  const std::string file_path = cyber::common::GetAbsolutePath(
      lib::ConfigManager::Instance()->work_root(), FLAGS_obs_sensor_meta_path);

  MultiSensorMeta sensor_list_proto;
  if (!GetProtoFromASCIIFile(file_path, &sensor_list_proto)) {
    AERROR << "Invalid MultiSensorMeta file: " << FLAGS_obs_sensor_meta_path;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::Init";
  return false;
  }

  auto AddSensorInfo = [this](const SensorMeta& sensor_meta_proto) {
    SensorInfo sensor_info;
    sensor_info.name = sensor_meta_proto.name();
    sensor_info.type = static_cast<SensorType>(sensor_meta_proto.type());
    sensor_info.orientation =
        static_cast<SensorOrientation>(sensor_meta_proto.orientation());
    sensor_info.frame_id = sensor_meta_proto.name();

    auto pair = sensor_info_map_.insert(
        make_pair(sensor_meta_proto.name(), sensor_info));
    if (!pair.second) {
      AERROR << "Duplicate sensor name error.";
      
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::Init";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::Init";
  return false;
    }

    if (this->IsCamera(sensor_info.type)) {
      std::shared_ptr<BrownCameraDistortionModel> distort_model(
          new BrownCameraDistortionModel());
      auto intrinsic_file = IntrinsicPath(sensor_info.frame_id);
      if (!LoadBrownCameraIntrinsic(intrinsic_file, distort_model.get())) {
        AERROR << "Failed to load camera intrinsic:" << intrinsic_file;
        
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::Init";
  return false;
      }
      distort_model_map_.insert(make_pair(
          sensor_meta_proto.name(),
          std::dynamic_pointer_cast<BaseCameraDistortionModel>(distort_model)));
      undistort_model_map_.insert(make_pair(sensor_meta_proto.name(),
                                            distort_model->get_camera_model()));
    }
    return true;
  };

  for (const SensorMeta& sensor_meta_proto : sensor_list_proto.sensor_meta()) {
    if (!AddSensorInfo(sensor_meta_proto)) {
      AERROR << "Failed to add sensor_info: " << sensor_meta_proto.name();
      return false;
    }
  }

  inited_ = true;
  AINFO << "Init sensor_manager success.";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::Init";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::Init";
 }

bool SensorManager::IsSensorExist(const std::string& name) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsSensorExist";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsSensorExist";
  return sensor_info_map_.find(name) != sensor_info_map_.end();

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsSensorExist";
 }

bool SensorManager::GetSensorInfo(const std::string& name,
                                  SensorInfo* sensor_info) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::GetSensorInfo";
  if (sensor_info == nullptr) {
    AERROR << "Nullptr error.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::GetSensorInfo";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::GetSensorInfo";
  return false;
  }

  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    return false;
  }

  *sensor_info = itr->second;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::GetSensorInfo";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::GetSensorInfo";
 }

std::shared_ptr<BaseCameraDistortionModel> SensorManager::GetDistortCameraModel(
    const std::string& name) const {
  const auto& itr = distort_model_map_.find(name);

  return itr == distort_model_map_.end() ? nullptr : itr->second;
}

std::shared_ptr<BaseCameraModel> SensorManager::GetUndistortCameraModel(
    const std::string& name) const {
  const auto& itr = undistort_model_map_.find(name);

  return itr == undistort_model_map_.end() ? nullptr : itr->second;
}

bool SensorManager::IsHdLidar(const std::string& name) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsHdLidar";
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsHdLidar";
  return false;
  }

  SensorType type = itr->second.type;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsHdLidar";
  return this->IsHdLidar(type);

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsHdLidar";
 }

bool SensorManager::IsHdLidar(const SensorType& type) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsHdLidar";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsHdLidar";
  return type == SensorType::VELODYNE_128 || type == SensorType::VELODYNE_64 ||
         type == SensorType::VELODYNE_32 || type == SensorType::VELODYNE_16;

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsHdLidar";
 }

bool SensorManager::IsLdLidar(const std::string& name) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsLdLidar";
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsLdLidar";
  return false;
  }

  SensorType type = itr->second.type;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsLdLidar";
  return this->IsLdLidar(type);

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsLdLidar";
 }

bool SensorManager::IsLdLidar(const SensorType& type) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsLdLidar";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsLdLidar";
  return type == SensorType::LDLIDAR_4 || type == SensorType::LDLIDAR_1;

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsLdLidar";
 }

bool SensorManager::IsLidar(const std::string& name) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsLidar";
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsLidar";
  return false;
  }

  SensorType type = itr->second.type;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsLidar";
  return this->IsLidar(type);

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsLidar";
 }

bool SensorManager::IsLidar(const SensorType& type) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsLidar";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsLidar";
  return this->IsHdLidar(type) || this->IsLdLidar(type);

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsLidar";
 }

bool SensorManager::IsRadar(const std::string& name) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsRadar";
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsRadar";
  return false;
  }

  SensorType type = itr->second.type;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsRadar";
  return this->IsRadar(type);

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsRadar";
 }

bool SensorManager::IsRadar(const SensorType& type) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsRadar";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsRadar";
  return type == SensorType::SHORT_RANGE_RADAR ||
         type == SensorType::LONG_RANGE_RADAR;

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsRadar";
 }

bool SensorManager::IsCamera(const std::string& name) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsCamera";
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsCamera";
  return false;
  }

  SensorType type = itr->second.type;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsCamera";
  return this->IsCamera(type);

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsCamera";
 }

bool SensorManager::IsCamera(const SensorType& type) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsCamera";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsCamera";
  return type == SensorType::MONOCULAR_CAMERA ||
         type == SensorType::STEREO_CAMERA;

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsCamera";
 }

bool SensorManager::IsUltrasonic(const std::string& name) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsUltrasonic";
  const auto& itr = sensor_info_map_.find(name);
  if (itr == sensor_info_map_.end()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsUltrasonic";
  return false;
  }

  SensorType type = itr->second.type;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsUltrasonic";
  return this->IsUltrasonic(type);

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsUltrasonic";
 }

bool SensorManager::IsUltrasonic(const SensorType& type) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::IsUltrasonic";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::IsUltrasonic";
  return type == SensorType::ULTRASONIC;

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::IsUltrasonic";
 }

std::string SensorManager::GetFrameId(const std::string& name) const {
AINFO<<"(DMCZP) EnteringMethod: SensorManager::GetFrameId";
  const auto& itr = sensor_info_map_.find(name);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorManager::GetFrameId";
  return itr == sensor_info_map_.end() ? std::string("") : itr->second.frame_id;

  AINFO<<"(DMCZP) LeaveMethod: SensorManager::GetFrameId";
 }

}  // namespace common
}  // namespace perception
}  // namespace apollo
