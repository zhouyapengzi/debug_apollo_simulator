#include "cyber/common/log.h"
/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/ch/ch_vehicle_factory.h"
#include "gtest/gtest.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"

namespace apollo {
namespace canbus {

class ChVehicleFactoryTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    VehicleParameter parameter;
    parameter.set_brand(apollo::common::CH);
    ch_factory_.SetVehicleParameter(parameter);
  }
  virtual void TearDown() {}

 protected:
  ChVehicleFactory ch_factory_;
};

TEST_F(ChVehicleFactoryTest, InitVehicleController) {
AINFO<<"(DMCZP) EnteringMethod: TEST_F";
  EXPECT_NE(ch_factory_.CreateVehicleController(), nullptr);
}

TEST_F(ChVehicleFactoryTest, InitMessageManager) {
AINFO<<"(DMCZP) EnteringMethod: TEST_F";
  EXPECT_NE(ch_factory_.CreateMessageManager(), nullptr);
}

}  // namespace canbus
}  // namespace apollo
