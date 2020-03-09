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

#include "modules/perception/lidar/lib/dummy/dummy_segmentation.h"

namespace apollo {
namespace perception {
namespace lidar {

bool DummySegmentation::Init(const SegmentationInitOptions& options) {
AINFO<<"(DMCZP) EnteringMethod: DummySegmentation::Init";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DummySegmentation::Init";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: DummySegmentation::Init";
 }

bool DummySegmentation::Segment(const SegmentationOptions& options,
                                LidarFrame* frame) {
AINFO<<"(DMCZP) EnteringMethod: DummySegmentation::Segment";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DummySegmentation::Segment";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: DummySegmentation::Segment";
 }

PERCEPTION_REGISTER_SEGMENTATION(DummySegmentation);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
