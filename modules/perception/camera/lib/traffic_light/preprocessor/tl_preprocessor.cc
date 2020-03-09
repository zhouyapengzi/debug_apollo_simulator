/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera/lib/traffic_light/preprocessor/tl_preprocessor.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/utils/time_util.h"

namespace apollo {
namespace perception {
namespace camera {

bool TLPreprocessor::Init(const TrafficLightPreprocessorInitOptions &options) {
AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::Init";
  camera::MultiCamerasInitOption projection_init_option;
  projection_init_option.camera_names = options.camera_names;
  if (!projection_.Init(projection_init_option)) {
    AERROR << "init multi_camera_projection failed.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::Init";
  return false;
  }

  num_cameras_ = projection_.getCameraNamesByDescendingFocalLen().size();
  lights_on_image_array_.resize(num_cameras_);
  lights_outside_image_array_.resize(num_cameras_);
  sync_interval_seconds_ = options.sync_interval_seconds;

  AINFO << "preprocessor init succeed";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::Init";
  return true;
}

bool TLPreprocessor::UpdateCameraSelection(
    const CarPose &pose, const TLPreprocessorOption &option,
    std::vector<base::TrafficLightPtr> *lights) {
AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::UpdateCameraSelection";
  const double &timestamp = pose.getTimestamp();
  selected_camera_name_.first = timestamp;
  selected_camera_name_.second = GetMaxFocalLenWorkingCameraName();

  AINFO << "TLPreprocessor Got signal number: " << lights->size()
        << ", ts: " << std::to_string(timestamp);
  if (lights->empty()) {
    AINFO << "No signals, select camera with max focal length: "
          << selected_camera_name_.second;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::UpdateCameraSelection";
  return true;
  }

  if (!ProjectLightsAndSelectCamera(pose, option,
                                    &(selected_camera_name_.second), lights)) {
    AERROR << "project_lights_and_select_camera failed, ts: "
           << std::to_string(timestamp);
  }

  AINFO << "selected_camera_id: " << selected_camera_name_.second;

  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::UpdateCameraSelection";
  return true;
}

bool TLPreprocessor::SyncInformation(const double image_timestamp,
                                     const std::string &cam_name) {
AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::SyncInformation";
  const double &proj_ts = selected_camera_name_.first;
  const std::string &proj_camera_name = selected_camera_name_.second;
  AINFO << "ready to sync information";

  if (!projection_.HasCamera(cam_name)) {
    AERROR << "sync_image failed, "
           << "get invalid camera_name: " << cam_name;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::SyncInformation";
  return false;
  }

  AINFO << "Enter TLPreprocessor::sync_image. proj_ts: "
        << std::to_string(proj_ts) << " proj_camera_name: " << proj_camera_name
        << " image_ts: " << std::to_string(image_timestamp)
        << " image_camera_name: " << cam_name;
  if (image_timestamp < last_pub_img_ts_) {
    AWARN << "TLPreprocessor reject the image pub ts:"
          << std::to_string(image_timestamp)
          << " which is earlier than last output ts:"
          << std::to_string(last_pub_img_ts_)
          << ", image_camera_name: " << cam_name;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::SyncInformation";
  return false;
  }

  if (proj_camera_name != cam_name) {
    AWARN << "sync_image failed - find close enough projection,"
          << "but camera_name not match.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::SyncInformation";
  return false;
  }
  AINFO << "sync_image succeeded.";
  last_pub_img_ts_ = image_timestamp;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::SyncInformation";
  return true;
}

bool TLPreprocessor::UpdateLightsProjection(
    const CarPose &pose, const TLPreprocessorOption &option,
    const std::string &camera_name,
    std::vector<base::TrafficLightPtr> *lights) {
  lights_on_image_.clear();
  lights_outside_image_.clear();

  AINFO << "clear lights_outside_image_ " << lights_outside_image_.size();

  if (lights->empty()) {
    AINFO << "No lights to be projected";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::UpdateLightsProjection";
  return true;
  }

  if (!ProjectLights(pose, camera_name, lights, &lights_on_image_,
                     &lights_outside_image_)) {
    AERROR << "update_lights_projection project lights on " << camera_name
           << " image failed";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::UpdateLightsProjection";
  return false;
  }

  if (lights_outside_image_.size() > 0) {
    AERROR << "update_lights_projection failed,"
           << "lights_outside_image->size() " << lights_outside_image_.size()
           << " ts: " << std::to_string(pose.getTimestamp());
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::UpdateLightsProjection";
  return false;
  }

  auto min_focal_len_working_camera = GetMinFocalLenWorkingCameraName();
  if (camera_name == min_focal_len_working_camera) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::UpdateLightsProjection";
  return lights_on_image_.size() > 0;
  }
  for (const base::TrafficLightPtr &light : lights_on_image_) {
    if (OutOfValidRegion(light->region.projection_roi,
                         projection_.getImageWidth(camera_name),
                         projection_.getImageHeight(camera_name),
                         option.image_borders_size->at(camera_name))) {
      AINFO << "update_lights_projection light project out of image region. "
            << "camera_name: " << camera_name;
      
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::UpdateLightsProjection";
  return false;
    }
  }

  AINFO << "UpdateLightsProjection success";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::UpdateLightsProjection";
  return true;
}

bool TLPreprocessor::SetCameraWorkingFlag(const std::string &camera_name,
                                          bool is_working) {
AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::SetCameraWorkingFlag";
  if (!projection_.HasCamera(camera_name)) {
    AERROR << "SetCameraWorkingFlag failed, "
           << "get invalid camera_name: " << camera_name;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::SetCameraWorkingFlag";
  return false;
  }
  camera_is_working_flags_[camera_name] = is_working;
  AINFO << "SetCameraWorkingFlag succeeded, camera_name: " << camera_name
        << ", flag: " << camera_is_working_flags_[camera_name];
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::SetCameraWorkingFlag";
  return true;
}

bool TLPreprocessor::GetCameraWorkingFlag(const std::string &camera_name,
                                          bool *is_working) const {
AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::GetCameraWorkingFlag";
  if (!projection_.HasCamera(camera_name)) {
    AERROR << "GetCameraWorkingFlag failed, "
           << "get invalid camera_name: " << camera_name;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::GetCameraWorkingFlag";
  return false;
  }

  if (camera_is_working_flags_.find(camera_name) ==
          camera_is_working_flags_.end() ||
      !camera_is_working_flags_.at(camera_name)) {
    *is_working = false;
  } else {
    *is_working = true;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::GetCameraWorkingFlag";
  return true;
}

void TLPreprocessor::SelectCamera(
    std::vector<base::TrafficLightPtrs> *lights_on_image_array,
    std::vector<base::TrafficLightPtrs> *lights_outside_image_array,
    const TLPreprocessorOption &option, std::string *selected_camera_name) {
AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::SelectCamera";
  // do not check boundary if this is min focal camera
  auto min_focal_len_working_camera = GetMinFocalLenWorkingCameraName();
  AINFO << "working camera with minimum focal length: "
        << min_focal_len_working_camera;

  const auto &camera_names = projection_.getCameraNamesByDescendingFocalLen();
  for (size_t cam_id = 0; cam_id < lights_on_image_array->size(); ++cam_id) {
    const auto &camera_name = camera_names[cam_id];
    bool is_working = false;
    // camera is not working ,skip
    if (!GetCameraWorkingFlag(camera_name, &is_working) || !is_working) {
      AINFO << "camera " << camera_name << "is not working";
      continue;
    }

    bool ok = true;
    if (camera_name != min_focal_len_working_camera) {
      // lights must project on the image if this is not min focal camera
      if (lights_outside_image_array->at(cam_id).size() > 0) {
        AINFO << "light project out of image, "
              << "camera_name: " << camera_name
              << " lights_outside_image_array->at(cam_id).size(): "
              << lights_outside_image_array->at(cam_id).size();
        continue;
      }
      auto lights = lights_on_image_array->at(cam_id);
      for (const auto light : lights) {
        // check boundary
        if (OutOfValidRegion(light->region.projection_roi,
                             projection_.getImageWidth(camera_name),
                             projection_.getImageHeight(camera_name),
                             option.image_borders_size->at(camera_name))) {
          ok = false;
          AINFO << "light project out of image region, "
                << "camera_name: " << camera_name << " border_size: "
                << option.image_borders_size->at(camera_name);
          break;
        }
      }
    } else {
      // do not checkout the boundary if this is min focal camera
      ok = (lights_on_image_array->at(cam_id).size() > 0);
    }

    if (ok) {
      *selected_camera_name = camera_name;
      break;
    }
  }
  AINFO << "select_camera selection: " << *selected_camera_name;

  AINFO<<"(DMCZP) LeaveMethod: TLPreprocessor::SelectCamera";
 }

bool TLPreprocessor::ProjectLights(
    const CarPose &pose, const std::string &camera_name,
    std::vector<base::TrafficLightPtr> *lights,
    base::TrafficLightPtrs *lights_on_image,
    base::TrafficLightPtrs *lights_outside_image) {
AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::ProjectLights";
  if (lights->empty()) {
    AINFO << "project_lights get empty signals.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::ProjectLights";
  return true;
  }
  if (!projection_.HasCamera(camera_name)) {
    AERROR << "project_lights get invalid camera_name: " << camera_name;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::ProjectLights";
  return false;
  }

  // camera is not working
  bool is_working = false;
  if (!GetCameraWorkingFlag(camera_name, &is_working) || !is_working) {
    AWARN << "TLPreprocessor::project_lights not project lights, "
          << "camera is not working, camera_name: " << camera_name;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::ProjectLights";
  return true;
  }

  for (size_t i = 0; i < lights->size(); ++i) {
    base::TrafficLightPtr light_proj(new base::TrafficLight);
    auto light = lights->at(i);
    if (!projection_.Project(pose, ProjectOption(camera_name), light.get())) {
      light->region.outside_image = true;
      *light_proj = *light;
      lights_outside_image->push_back(light_proj);
    } else {
      light->region.outside_image = false;
      *light_proj = *light;
      lights_on_image->push_back(light_proj);
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::ProjectLights";
  return true;
}

bool TLPreprocessor::ProjectLightsAndSelectCamera(
    const CarPose &pose, const TLPreprocessorOption &option,
    std::string *selected_camera_name,
    std::vector<base::TrafficLightPtr> *lights) {
  CHECK_NOTNULL(selected_camera_name);
  CHECK_NOTNULL(lights);

  for (auto &light_ptrs : lights_on_image_array_) {
    light_ptrs.clear();
  }
  for (auto &light_ptrs : lights_outside_image_array_) {
    light_ptrs.clear();
  }

  // project light region on each camera's image plane
  const auto &camera_names = projection_.getCameraNamesByDescendingFocalLen();
  for (size_t cam_id = 0; cam_id < num_cameras_; ++cam_id) {
    const std::string &camera_name = camera_names[cam_id];
    if (!ProjectLights(pose, camera_name, lights,
                       &(lights_on_image_array_[cam_id]),
                       &(lights_outside_image_array_[cam_id]))) {
      AERROR << "select_camera_by_lights_projection project lights on "
             << camera_name << " image failed";
      
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::ProjectLightsAndSelectCamera";
  return false;
    }
  }

  projections_outside_all_images_ = !lights->empty();
  for (size_t cam_id = 0; cam_id < num_cameras_; ++cam_id) {
    projections_outside_all_images_ =
        projections_outside_all_images_ &&
        (lights_on_image_array_[cam_id].size() < lights->size());
  }
  if (projections_outside_all_images_) {
    AWARN << "lights projections outside all images";
  }

  // select which camera to be used
  SelectCamera(&lights_on_image_array_, &lights_outside_image_array_, option,
               selected_camera_name);

  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::ProjectLightsAndSelectCamera";
  return true;
}

bool TLPreprocessor::GetAlllightsOutsideFlag() const {
AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::GetAlllightsOutsideFlag";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::GetAlllightsOutsideFlag";
  return projections_outside_all_images_;
}

std::string TLPreprocessor::Name() const {
  AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::Name";
 
  AINFO<<"(DMCZP) (retur
  AINFO<<"(DMCZP) LeaveMethod: TLPreprocessor::Name";
 n) LeaveMethod: TLPreprocessor::Name";
  return "TLPreprocessor"; }

std::string TLPreprocessor::GetMinFocalLenWorkingCameraName() const {
AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::GetMinFocalLenWorkingCameraName";
  const auto &camera_names = projection_.getCameraNamesByDescendingFocalLen();
  for (auto itr = camera_names.crbegin(); itr != camera_names.crend(); ++itr) {
    bool is_working = false;
    if (GetCameraWorkingFlag(*itr, &is_working) && is_working) {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::GetMinFocalLenWorkingCameraName";
  return *itr;
    }
  }
  AWARN << "No working camera, 
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::GetMinFocalLenWorkingCameraName";
  return empty camera_name";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::GetMinFocalLenWorkingCameraName";
  return "";
}

std::string TLPreprocessor::GetMaxFocalLenWorkingCameraName() const {
AINFO<<"(DMCZP) EnteringMethod: TLPreprocessor::GetMaxFocalLenWorkingCameraName";
  const auto &camera_names = projection_.getCameraNamesByDescendingFocalLen();
  for (const auto &camera_name : camera_names) {
    bool is_working = false;
    if (GetCameraWorkingFlag(camera_name, &is_working) && is_working) {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::GetMaxFocalLenWorkingCameraName";
  return camera_name;
    }
  }
  AWARN << "No working camera, 
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::GetMaxFocalLenWorkingCameraName";
  return empty camera_name";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TLPreprocessor::GetMaxFocalLenWorkingCameraName";
  return "";
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
