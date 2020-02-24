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

#include "cyber/component/timer_component.h"
#include "cyber/timer/timer.h"

namespace apollo {
namespace cyber {

TimerComponent::TimerComponent() {}
AINFO<<"(DMCZP) EnteringMethod: TimerComponent::TimerComponent";

TimerComponent::~TimerComponent() {}

bool TimerComponent::Process() {
AINFO<<"(DMCZP) EnteringMethod: TimerComponent::Process";
  if (is_shutdown_.load()) {
    return true;
  }
  return Proc();
}

bool TimerComponent::Initialize(const TimerComponentConfig& config) {
AINFO<<"(DMCZP) EnteringMethod: TimerComponent::Initialize";
  if (!config.has_name() || !config.has_interval()) {
    AERROR << "Missing required field in config file.";
    return false;
  }
  node_.reset(new Node(config.name()));
  LoadConfigFiles(config);
  if (!Init()) {
    return false;
  }

  std::weak_ptr<TimerComponent> self =
      std::dynamic_pointer_cast<TimerComponent>(shared_from_this());
  auto func = [self]() {
    auto ptr = self.lock();
    if (ptr) {
      ptr->Proc();
    }
  };
  timer_.reset(new Timer(config.interval(), func, false));
  timer_->Start();
  return true;
}

}  // namespace cyber
}  // namespace apollo
