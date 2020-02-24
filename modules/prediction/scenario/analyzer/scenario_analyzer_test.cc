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

#include "modules/prediction/scenario/analyzer/scenario_analyzer.h"

#include "modules/prediction/common/kml_map_based_test.h"

namespace apollo {
namespace prediction {

class ScenarioAnalyzerTest : public KMLMapBasedTest {};

TEST_F(ScenarioAnalyzerTest, unknown) {
AINFO<<"(DMCZP) EnteringMethod: TEST_F";
  EnvironmentFeatures environment_features;
  auto ptr_scenario_features = ScenarioAnalyzer::Analyze(environment_features);
  EXPECT_EQ(ptr_scenario_features->scenario().type(), Scenario::UNKNOWN);
}

TEST_F(ScenarioAnalyzerTest, junction) {
AINFO<<"(DMCZP) EnteringMethod: TEST_F";
  EnvironmentFeatures environment_features;
  environment_features.SetFrontJunction("1", 3.0);
  auto ptr_scenario_features = ScenarioAnalyzer::Analyze(environment_features);
  EXPECT_EQ(ptr_scenario_features->scenario().type(), Scenario::JUNCTION);
}

}  // namespace prediction
}  // namespace apollo
