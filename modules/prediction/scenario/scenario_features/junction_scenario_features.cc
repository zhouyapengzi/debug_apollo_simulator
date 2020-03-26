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

#include "modules/prediction/scenario/scenario_features/junction_scenario_features.h"

namespace apollo {
namespace prediction {

JunctionScenarioFeatures::JunctionScenarioFeatures() {
  AINFO<<"(DMCZP) EnteringMethod: JunctionScenarioFeatures::JunctionScenarioFeatures";

  scenario_.set_type(Scenario::JUNCTION);

  AINFO<<"(DMCZP) LeaveMethod: JunctionScenarioFeatures::JunctionScenarioFeatures";
 }

JunctionScenarioFeatures::~JunctionScenarioFeatures() {}

void JunctionScenarioFeatures::BuildJunctionScenarioFeatures(
    const EnvironmentFeatures& environment_features) {
  AINFO<<"(DMCZP) EnteringMethod: JunctionScenarioFeatures::BuildJunctionScenarioFeatures";

  // CHECK(environment_features.has_front_junction());
  scenario_.set_junction_id(environment_features.GetFrontJunction().first);

  AINFO<<"(DMCZP) LeaveMethod: JunctionScenarioFeatures::BuildJunctionScenarioFeatures";
 }

}  // namespace prediction
}  // namespace apollo
