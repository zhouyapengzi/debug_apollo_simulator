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

/**
 * @file
 */

#include "modules/prediction/scenario/scenario_features/scenario_features.h"

namespace apollo {
namespace prediction {

ScenarioFeatures::ScenarioFeatures() {
  AINFO<<"(DMCZP) EnteringMethod: ScenarioFeatures::ScenarioFeatures";
 scenario_.set_type(Scenario::UNKNOWN); 
  AINFO<<"(DMCZP) LeaveMethod: ScenarioFeatures::ScenarioFeatures";
 }

const Scenario& ScenarioFeatures::scenario() const {
  AINFO<<"(DMCZP) EnteringMethod: ScenarioFeatures::scenario";
 
  AINFO<<"(DMCZP)
  AINFO<<"(DMCZP) LeaveMethod: ScenarioFeatures::scenario";
  (return) LeaveMethod: ScenarioFeatures::scenario";
  return scenario_; }

}  // namespace prediction
}  // namespace apollo
