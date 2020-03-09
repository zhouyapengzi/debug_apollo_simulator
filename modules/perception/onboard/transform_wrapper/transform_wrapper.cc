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
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"

#include "cyber/common/log.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace onboard {

DEFINE_string(obs_sensor2novatel_tf2_frame_id, "novatel",
              "sensor to novatel frame id");
DEFINE_string(obs_novatel2world_tf2_frame_id, "world",
              "novatel to world frame id");
DEFINE_string(obs_novatel2world_tf2_child_frame_id, "novatel",
              "novatel to world child frame id");
DEFINE_double(obs_tf2_buff_size, 0.01, "query Cyber TF buffer size in second");
DEFINE_double(obs_transform_cache_size, 1.0, "transform cache size in second");
DEFINE_double(obs_max_local_pose_extrapolation_latency, 0.15,
              "max local pose extrapolation period in second");
DEFINE_bool(obs_enable_local_pose_extrapolation, true,
            "use local pose extrapolation");

void TransformCache::AddTransform(const StampedTransform& transform) {
AINFO<<"(DMCZP) EnteringMethod: DEFINE_string";
AINFO<<"(DMCZP) EnteringMethod: DEFINE_string";
AINFO<<"(DMCZP) EnteringMethod: DEFINE_string";
AINFO<<"(DMCZP) EnteringMethod: DEFINE_double";
AINFO<<"(DMCZP) EnteringMethod: DEFINE_double";
AINFO<<"(DMCZP) EnteringMethod: DEFINE_double";
AINFO<<"(DMCZP) EnteringMethod: DEFINE_bool";
AINFO<<"(DMCZP) EnteringMethod: TransformCache::AddTransform";
  if (transforms_.empty()) {
    transforms_.push_back(transform);
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformCache::AddTransform";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_bool";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_double";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_double";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_double";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_string";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_string";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_string";
  return;
  }
  double delt = transform.timestamp - transforms_.back().timestamp;
  if (delt < 0.0) {
    AINFO << "ERROR: add earlier transform to transform cache";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformCache::AddTransform";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_bool";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_double";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_double";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_double";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_string";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_string";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DEFINE_string";
  return;
  }

  do {
    delt = transform.timestamp - transforms_.front().timestamp;
    if (delt < cache_duration_) {
      break;
    }
    transforms_.pop_front();
  } while (!transforms_.empty());

  transforms_.push_back(transform);

  AINFO<<"(DMCZP) LeaveMethod: TransformCache::AddTransform";
 
  AINFO<<"(DMCZP) LeaveMethod: DEFINE_bool";
 
  AINFO<<"(DMCZP) LeaveMethod: DEFINE_double";
 
  AINFO<<"(DMCZP) LeaveMethod: DEFINE_double";
 
  AINFO<<"(DMCZP) LeaveMethod: DEFINE_double";
 
  AINFO<<"(DMCZP) LeaveMethod: DEFINE_string";
 
  AINFO<<"(DMCZP) LeaveMethod: DEFINE_string";
 
  AINFO<<"(DMCZP) LeaveMethod: DEFINE_string";
 }

bool TransformCache::QueryTransform(double timestamp,
                                    StampedTransform* transform,
                                    double max_duration) {
AINFO<<"(DMCZP) EnteringMethod: TransformCache::QueryTransform";
  if (transforms_.empty() || transform == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformCache::QueryTransform";
  return false;
  }

  double delt = timestamp - transforms_.back().timestamp;
  if (delt > max_duration) {
    AINFO << "ERROR: query timestamp is " << delt
          << "s later than cached timestamp";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformCache::QueryTransform";
  return false;
  } else if (delt < 0.0) {
    AINFO << "ERROR: query earlier timestamp than transform cache";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformCache::QueryTransform";
  return false;
  }

  int size = static_cast<int>(transforms_.size());
  if (size == 1) {
    (*transform) = transforms_.back();
    transform->timestamp = timestamp;
    AINFO << "use transform at " << std::to_string(transforms_.back().timestamp)
          << " for " << std::to_string(timestamp);
  } else {
    double ratio =
        (timestamp - transforms_[size - 2].timestamp) /
        (transforms_[size - 1].timestamp - transforms_[size - 2].timestamp);

    transform->rotation = transforms_[size - 2].rotation.slerp(
        ratio, transforms_[size - 1].rotation);

    transform->translation.x() =
        transforms_[size - 2].translation.x() * (1 - ratio) +
        transforms_[size - 1].translation.x() * ratio;
    transform->translation.y() =
        transforms_[size - 2].translation.y() * (1 - ratio) +
        transforms_[size - 1].translation.y() * ratio;
    transform->translation.z() =
        transforms_[size - 2].translation.z() * (1 - ratio) +
        transforms_[size - 1].translation.z() * ratio;

    AINFO << "estimate pose at " << std::to_string(timestamp)
          << " from poses at "
          << std::to_string(transforms_[size - 2].timestamp) << " and "
          << std::to_string(transforms_[size - 1].timestamp);
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformCache::QueryTransform";
  return true;
}

void TransformWrapper::Init(
    const std::string& sensor2novatel_tf2_child_frame_id) {
AINFO<<"(DMCZP) EnteringMethod: TransformWrapper::Init";
AINFO<<"(DMCZP) EnteringMethod: TransformWrapper::Init";
  sensor2novatel_tf2_frame_id_ = FLAGS_obs_sensor2novatel_tf2_frame_id;
  sensor2novatel_tf2_child_frame_id_ = sensor2novatel_tf2_child_frame_id;
  novatel2world_tf2_frame_id_ = FLAGS_obs_novatel2world_tf2_frame_id;
  novatel2world_tf2_child_frame_id_ =
      FLAGS_obs_novatel2world_tf2_child_frame_id;
  transform_cache_.SetCacheDuration(FLAGS_obs_transform_cache_size);
  inited_ = true;

  AINFO<<"(DMCZP) LeaveMethod: TransformWrapper::Init";
 
  AINFO<<"(DMCZP) LeaveMethod: TransformWrapper::Init";
 }

void TransformWrapper::Init(
    const std::string& sensor2novatel_tf2_frame_id,
    const std::string& sensor2novatel_tf2_child_frame_id,
    const std::string& novatel2world_tf2_frame_id,
    const std::string& novatel2world_tf2_child_frame_id) {
  sensor2novatel_tf2_frame_id_ = sensor2novatel_tf2_frame_id;
  sensor2novatel_tf2_child_frame_id_ = sensor2novatel_tf2_child_frame_id;
  novatel2world_tf2_frame_id_ = novatel2world_tf2_frame_id;
  novatel2world_tf2_child_frame_id_ = novatel2world_tf2_child_frame_id;
  transform_cache_.SetCacheDuration(FLAGS_obs_transform_cache_size);
  inited_ = true;
}

bool TransformWrapper::GetSensor2worldTrans(
    double timestamp, Eigen::Affine3d* sensor2world_trans,
    Eigen::Affine3d* novatel2world_trans) {
AINFO<<"(DMCZP) EnteringMethod: TransformWrapper::GetSensor2worldTrans";
  if (!inited_) {
    AERROR << "TransformWrapper not Initialized,"
           << " unable to call GetSensor2worldTrans.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetSensor2worldTrans";
  return false;
  }

  if (sensor2novatel_extrinsics_ == nullptr) {
    StampedTransform trans_sensor2novatel;
    if (!QueryTrans(timestamp, &trans_sensor2novatel,
                    sensor2novatel_tf2_frame_id_,
                    sensor2novatel_tf2_child_frame_id_)) {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetSensor2worldTrans";
  return false;
    }
    sensor2novatel_extrinsics_.reset(new Eigen::Affine3d);
    *sensor2novatel_extrinsics_ =
        trans_sensor2novatel.translation * trans_sensor2novatel.rotation;
    AINFO << "Get sensor2novatel extrinsics successfully.";
  }

  StampedTransform trans_novatel2world;
  trans_novatel2world.timestamp = timestamp;
  Eigen::Affine3d novatel2world;

  if (!QueryTrans(timestamp, &trans_novatel2world, novatel2world_tf2_frame_id_,
                  novatel2world_tf2_child_frame_id_)) {
    if (FLAGS_obs_enable_local_pose_extrapolation) {
      if (!transform_cache_.QueryTransform(
              timestamp, &trans_novatel2world,
              FLAGS_obs_max_local_pose_extrapolation_latency)) {
        
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetSensor2worldTrans";
  return false;
      }
    } else {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetSensor2worldTrans";
  return false;
    }
  } else if (FLAGS_obs_enable_local_pose_extrapolation) {
    transform_cache_.AddTransform(trans_novatel2world);
  }

  novatel2world =
      trans_novatel2world.translation * trans_novatel2world.rotation;
  *sensor2world_trans = novatel2world * (*sensor2novatel_extrinsics_);
  if (novatel2world_trans != nullptr) {
    *novatel2world_trans = novatel2world;
  }
  AINFO << "Get pose timestamp: " << std::to_string(timestamp)
        << ", pose: " << std::endl
        << (*sensor2world_trans).matrix();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetSensor2worldTrans";
  return true;
}

bool TransformWrapper::GetExtrinsics(Eigen::Affine3d* trans) {
AINFO<<"(DMCZP) EnteringMethod: TransformWrapper::GetExtrinsics";
  if (!inited_ || trans == nullptr || sensor2novatel_extrinsics_ == nullptr) {
    AERROR << "TransformWrapper get extrinsics failed";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetExtrinsics";
  return false;
  }
  *trans = *sensor2novatel_extrinsics_;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetExtrinsics";
  return true;
}

bool TransformWrapper::GetTrans(double timestamp, Eigen::Affine3d* trans,
                                const std::string& frame_id,
                                const std::string& child_frame_id) {
AINFO<<"(DMCZP) EnteringMethod: TransformWrapper::GetTrans";
  StampedTransform transform;
  if (!QueryTrans(timestamp, &transform, frame_id, child_frame_id)) {
    if (!FLAGS_obs_enable_local_pose_extrapolation ||
        !transform_cache_.QueryTransform(
            timestamp, &transform,
            FLAGS_obs_max_local_pose_extrapolation_latency)) {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetTrans";
  return false;
    }
  }

  if (FLAGS_obs_enable_local_pose_extrapolation) {
    transform_cache_.AddTransform(transform);
  }

  *trans = transform.translation * transform.rotation;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetTrans";
  return true;
}

bool TransformWrapper::QueryTrans(double timestamp, StampedTransform* trans,
                                  const std::string& frame_id,
                                  const std::string& child_frame_id) {
AINFO<<"(DMCZP) EnteringMethod: TransformWrapper::QueryTrans";
  // cyber::Time query_time(timestamp);
  cyber::Time query_time = cyber::Time(0);

  std::string err_string;
  if (!tf2_buffer_->canTransform(frame_id, child_frame_id, query_time,
                                 static_cast<float>(FLAGS_obs_tf2_buff_size),
                                 &err_string)) {
    AERROR << "Can not find transform. " << std::to_string(timestamp)
           << " frame_id: " << frame_id << " child_frame_id: " << child_frame_id
           << " Error info: " << err_string;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::QueryTrans";
  return false;
  }

  apollo::transform::TransformStamped stamped_transform;
  try {
    stamped_transform =
        tf2_buffer_->lookupTransform(frame_id, child_frame_id, query_time);

    trans->translation =
        Eigen::Translation3d(stamped_transform.transform().translation().x(),
                             stamped_transform.transform().translation().y(),
                             stamped_transform.transform().translation().z());
    trans->rotation =
        Eigen::Quaterniond(stamped_transform.transform().rotation().qw(),
                           stamped_transform.transform().rotation().qx(),
                           stamped_transform.transform().rotation().qy(),
                           stamped_transform.transform().rotation().qz());
  } catch (tf2::TransformException& ex) {
    AERROR << ex.what();
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::QueryTrans";
  return false;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::QueryTrans";
  return true;
}

bool TransformWrapper::GetExtrinsicsBySensorId(
    const std::string& from_sensor_id, const std::string& to_sensor_id,
    Eigen::Affine3d* trans) {
AINFO<<"(DMCZP) EnteringMethod: TransformWrapper::GetExtrinsicsBySensorId";
  if (trans == nullptr) {
    AERROR << "TransformWrapper get extrinsics failed";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetExtrinsicsBySensorId";
  return false;
  }

  common::SensorManager* sensor_manager = common::SensorManager::Instance();
  std::string frame_id = sensor_manager->GetFrameId(to_sensor_id);
  std::string child_frame_id = sensor_manager->GetFrameId(from_sensor_id);

  StampedTransform transform;
  bool status = QueryTrans(0.0, &transform, frame_id, child_frame_id);
  if (status) {
    *trans = transform.translation * transform.rotation;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TransformWrapper::GetExtrinsicsBySensorId";
  return status;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
