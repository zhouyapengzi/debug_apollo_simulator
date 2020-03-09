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
#include "modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h"

#include "cyber/common/file.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/fusion/base/base_init_options.h"
#include "modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper/proto/pbf_gatekeeper_config.pb.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

using cyber::common::GetAbsolutePath;

PbfGatekeeper::PbfGatekeeper() {
  AINFO<<"(DMCZP) EnteringMethod: PbfGatekeeper::PbfGatekeeper";

  AINFO<<"(DMCZP) LeaveMethod: PbfGatekeeper::PbfGatekeeper";
 }

PbfGatekeeper::~PbfGatekeeper() {}

bool PbfGatekeeper::Init() {
AINFO<<"(DMCZP) EnteringMethod: PbfGatekeeper::Init";
  BaseInitOptions options;
  if (!GetFusionInitOptions("PbfGatekeeper", &options)) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::Init";
  return false;
  }

  std::string woork_root_config = GetAbsolutePath(
      lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(woork_root_config, options.conf_file);
  PbfGatekeeperConfig params;

  if (!cyber::common::GetProtoFromFile(config, &params)) {
    AERROR << "Read config failed: " << config;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::Init";
  return false;
  }
  params_.publish_if_has_lidar = params.publish_if_has_lidar();
  params_.publish_if_has_radar = params.publish_if_has_radar();
  params_.publish_if_has_camera = params.publish_if_has_camera();
  params_.use_camera_3d = params.use_camera_3d();
  params_.min_radar_confident_distance = params.min_radar_confident_distance();
  params_.max_radar_confident_angle = params.max_radar_confident_angle();
  params_.min_camera_publish_distance = params.min_camera_publish_distance();
  params_.invisible_period_threshold = params.invisible_period_threshold();
  params_.existance_threshold = params.existance_threshold();
  params_.radar_existance_threshold = params.radar_existance_threshold();
  params_.toic_threshold = params.toic_threshold();
  params_.use_track_time_pub_strategy = params.use_track_time_pub_strategy();
  params_.pub_track_time_thresh = params.pub_track_time_thresh();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::Init";
  return true;
}

std::string PbfGatekeeper::Name() const {
  AINFO<<"(DMCZP) EnteringMethod: PbfGatekeeper::Name";
 
  AINFO<<"(DMCZP) (retu
  AINFO<<"(DMCZP) LeaveMethod: PbfGatekeeper::Name";
 rn) LeaveMethod: PbfGatekeeper::Name";
  return "PbfGatekeeper"; }

bool PbfGatekeeper::AbleToPublish(const TrackPtr &track) {
AINFO<<"(DMCZP) EnteringMethod: PbfGatekeeper::AbleToPublish";
  bool invisible_in_lidar = !(track->IsLidarVisible());
  bool invisible_in_radar = !(track->IsRadarVisible());
  bool invisible_in_camera = !(track->IsCameraVisible());
  if (invisible_in_lidar && invisible_in_radar &&
      (!params_.use_camera_3d || invisible_in_camera)) {
    auto sensor_obj = track->GetFusedObject();
    if (sensor_obj != nullptr && sensor_obj->GetBaseObject()->sub_type !=
                                     base::ObjectSubType::TRAFFICCONE) {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::AbleToPublish";
  return false;
    }
  }
  time_t rawtime = static_cast<time_t>(track->GetFusedObject()->GetTimestamp());

  // use thread-safe localtime_r instead of localtime
  struct tm timeinfo;
  localtime_r(&rawtime, &timeinfo);
  bool is_night = (timeinfo.tm_hour >= 23);
  if (!LidarAbleToPublish(track) && !RadarAbleToPublish(track, is_night) &&
      !CameraAbleToPublish(track, is_night)) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::AbleToPublish";
  return false;
  }

  track->AddTrackedTimes();
  if (params_.use_track_time_pub_strategy &&
      track->GetTrackedTimes() <=
          static_cast<size_t>(params_.pub_track_time_thresh)) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::AbleToPublish";
  return false;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::AbleToPublish";
  return true;
}

bool PbfGatekeeper::LidarAbleToPublish(const TrackPtr &track) {
AINFO<<"(DMCZP) EnteringMethod: PbfGatekeeper::LidarAbleToPublish";
  bool visible_in_lidar = track->IsLidarVisible();
  if (params_.publish_if_has_lidar && visible_in_lidar) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::LidarAbleToPublish";
  return true;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::LidarAbleToPublish";
  return false;
}

bool PbfGatekeeper::RadarAbleToPublish(const TrackPtr &track, bool is_night) {
AINFO<<"(DMCZP) EnteringMethod: PbfGatekeeper::RadarAbleToPublish";
  bool visible_in_radar = track->IsRadarVisible();
  SensorObjectConstPtr radar_object = track->GetLatestRadarObject();
  if (params_.publish_if_has_radar && visible_in_radar &&
      radar_object != nullptr) {
    if (radar_object->GetSensorId() == "radar_front") {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::RadarAbleToPublish";
  return false;
      // if (radar_object->GetBaseObject()->radar_supplement.range >
      //         params_.min_radar_confident_distance &&
      //     radar_object->GetBaseObject()->radar_supplement.angle <
      //         params_.max_radar_confident_angle) {
      //   double heading_v =
      //       std::abs(track->GetFusedObject()->GetBaseObject()->velocity.dot(
      //           track->GetFusedObject()->GetBaseObject()->direction));
      //   double toic_p = track->GetToicProb();
      //   auto set_velocity_to_zero = [heading_v, track]() {
      //     if (heading_v < 0.3) {
      //       track->GetFusedObject()->GetBaseObject()->velocity.setZero();
      //     }
      //   };
      //   if (!is_night) {
      //     if (toic_p > params_.toic_threshold) {
      //       set_velocity_to_zero();
      //       
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::RadarAbleToPublish";
  return true;
      //     }
      //   } else {
      //     // the velocity buffer is [-3, +3] m/s
      //     double v_ct = 4.0;
      //     double v_slope = 1.0;
      //     auto heading_v_decision = [](double x, double c, double k) {
      //       x = x - c;
      //       
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::RadarAbleToPublish";
  return 0.5 + 0.5 * x * k / std::sqrt(1 + x * x * k * k);
      //     };
      //     auto fuse_two_probabilities = [](double p1, double p2) {
      //       double p = (p1 * p2) / (2 * p1 * p2 + 1 - p1 - p2);
      //       p = std::min(1.0 - std::numeric_limits<float>::epsilon(), p);
      //       
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::RadarAbleToPublish";
  return p;
      //     };

      //     double min_toic_p = 0.2;
      //     toic_p = std::max(min_toic_p, toic_p);
      //     double v_p = heading_v_decision(heading_v, v_ct, v_slope);
      //     double p = fuse_two_probabilities(toic_p, v_p);
      //     if (p > 0.5) {
      //       set_velocity_to_zero();
      //       
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::RadarAbleToPublish";
  return true;
      //     }
      //   }
      // }
    } else if (radar_object->GetSensorId() == "radar_rear") {
      ADEBUG << "radar_rear: min_dis: " << params_.min_radar_confident_distance
             << " obj dist: "
             << radar_object->GetBaseObject()->radar_supplement.range
             << " track_id: " << track->GetTrackId()
             << " exist_prob: " << track->GetExistanceProb();
      if (radar_object->GetBaseObject()->radar_supplement.range >
              params_.min_radar_confident_distance &&
          (radar_object->GetBaseObject()->velocity.norm() > 4.0 ||
           track->GetExistanceProb() > params_.radar_existance_threshold)) {
        
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::RadarAbleToPublish";
  return true;
      }
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::RadarAbleToPublish";
  return false;
}

bool PbfGatekeeper::CameraAbleToPublish(const TrackPtr &track, bool is_night) {
AINFO<<"(DMCZP) EnteringMethod: PbfGatekeeper::CameraAbleToPublish";
  bool visible_in_camera = track->IsCameraVisible();
  SensorId2ObjectMap &camera_objects = track->GetCameraObjects();
  auto iter = camera_objects.find("front_6mm");
  auto iter_narrow = camera_objects.find("front_12mm");
  iter = iter != camera_objects.end() ? iter : iter_narrow;
  if (params_.publish_if_has_camera && visible_in_camera &&
      iter != camera_objects.end() && params_.use_camera_3d && !is_night) {
    SensorObjectConstPtr camera_object = iter->second;
    double range =
        camera_object->GetBaseObject()->camera_supplement.local_center.norm();
    // If sub_type of object is traffic cone publish it regardless of range
    if ((camera_object->GetBaseObject()->sub_type ==
         base::ObjectSubType::TRAFFICCONE) ||
        (range >= params_.min_camera_publish_distance ||
         ((camera_object->GetBaseObject()->type ==
           base::ObjectType::UNKNOWN_UNMOVABLE) &&
          (range >= params_.min_camera_publish_distance)))) {
      double exist_prob = track->GetExistanceProb();
      if (exist_prob > params_.existance_threshold) {
        static int cnt_cam = 1;
        AINFO << "publish camera only object : cnt =  " << cnt_cam;
        cnt_cam++;
        
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::CameraAbleToPublish";
  return true;
      }
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfGatekeeper::CameraAbleToPublish";
  return false;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
