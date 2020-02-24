#include "cyber/common/log.h"
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/navi/decider/navi_task.h"

#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

NaviTask::NaviTask(const std::string& name) : name_(name) {}
AINFO<<"(DMCZP) EnteringMethod: NaviTask::NaviTask";

const std::string& NaviTask::Name() const { return name_; }
AINFO<<"(DMCZP) EnteringMethod: NaviTask::Name";

bool NaviTask::Init(const PlanningConfig& config) { return true; }
AINFO<<"(DMCZP) EnteringMethod: NaviTask::Init";

Status NaviTask::Execute(Frame* frame, ReferenceLineInfo* reference_line_info) {
AINFO<<"(DMCZP) EnteringMethod: NaviTask::Execute";
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
