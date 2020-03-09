#include "cyber/common/log.h"
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
#include "modules/perception/camera/lib/traffic_light/detector/recognition/recognition.h"

#include "cyber/common/file.h"
#include <thread>

namespace apollo {
namespace perception {
namespace camera {

bool TrafficLightRecognition::Init(
    const TrafficLightDetectorInitOptions& options) {
AINFO<<"(DMCZP) EnteringMethod: TrafficLightRecognition::Init";

  AINFO << "(pengzi) in method : TrafficLightRecognition::Init( const TrafficLightDetectorInitOptions& options) .thread: "<<std::this_thread::get_id();
  std::string proto_path =
      cyber::common::GetAbsolutePath(options.root_dir, options.conf_file);

  AINFO << "proto_path " << proto_path;
  if (!cyber::common::GetProtoFromFile(proto_path, &recognize_param_)) {
    AINFO << "load proto param failed, root dir: " << options.root_dir;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: TrafficLightRecognition::Init";
  return false;
  }

  AINFO << "(pengzi) load proto param for traffic light detector classify. proto path:"
        <<proto_path
        << " thread: "<<std::this_thread::get_id();  

  classify_quadrate_.reset(new ClassifyBySimple);
  classify_vertical_.reset(new ClassifyBySimple);
  classify_horizontal_.reset(new ClassifyBySimple);

  classify_quadrate_->Init(recognize_param_.quadrate_model(), options.gpu_id,
                           options.root_dir);
  classify_vertical_->Init(recognize_param_.vertical_model(), options.gpu_id,
                           options.root_dir);
  classify_horizontal_->Init(recognize_param_.horizontal_model(),
                             options.gpu_id, options.root_dir);

  
  AINFO<<"(DMCZP) (return) LeaveMethod: TrafficLightRecognition::Init";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: TrafficLightRecognition::Init";
 }

bool TrafficLightRecognition::Detect(const TrafficLightDetectorOptions& options,
                                     CameraFrame* frame) {
AINFO<<"(DMCZP) EnteringMethod: TrafficLightRecognition::Detect";

  AINFO << "(pengzi) in method : TrafficLightRecognition::Detect(const TrafficLightDetectorOptions& options,CameraFrame* frame) .thread: "<<std::this_thread::get_id();

  std::vector<base::TrafficLightPtr> candidate(1);

  for (base::TrafficLightPtr light : frame->traffic_lights) {
    if (light->region.is_detected) {
      candidate[0] = light;
      if (light->region.detect_class_id ==
          base::TLDetectionClass::TL_QUADRATE_CLASS) {
        AINFO << "Recognize Use Quadrate Model!";
        classify_quadrate_->Perform(frame, &candidate);
      } else if (light->region.detect_class_id ==
                 base::TLDetectionClass::TL_VERTICAL_CLASS) {
        AINFO << "Recognize Use Vertical Model!";
        classify_vertical_->Perform(frame, &candidate);
      } else if (light->region.detect_class_id ==
                 base::TLDetectionClass::TL_HORIZONTAL_CLASS) {
        AINFO << "Recognize Use Horizonal Model!";
        classify_horizontal_->Perform(frame, &candidate);
      } else {
        
  AINFO<<"(DMCZP) (return) LeaveMethod: TrafficLightRecognition::Detect";
  return false;
      }
    } else {
      light->status.color = base::TLColor::TL_UNKNOWN_COLOR;
      light->status.confidence = 0;
    }
  }

  
  AINFO<<"(DMCZP) (return) LeaveMethod: TrafficLightRecognition::Detect";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: TrafficLightRecognition::Detect";
 }

std::string TrafficLightRecognition::Name() const {
AINFO<<"(DMCZP) EnteringMethod: TrafficLightRecognition::Name";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: TrafficLightRecognition::Name";
  return "TrafficLightRecognition";

  AINFO<<"(DMCZP) LeaveMethod: TrafficLightRecognition::Name";
 }

REGISTER_TRAFFIC_LIGHT_DETECTOR(TrafficLightRecognition);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
