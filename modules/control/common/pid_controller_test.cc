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

#include "modules/control/common/pid_controller.h"

#include <iostream>
#include <string>

#include "cyber/common/file.h"
#include "gtest/gtest.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pid_conf.pb.h"

namespace apollo {
namespace control {

class PidControllerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string control_conf_file =
        "/apollo/modules/control/testdata/conf/control_conf.pb.txt";
    CHECK(cyber::common::GetProtoFromFile(control_conf_file, &control_conf_));
    lon_controller_conf_ = control_conf_.lon_controller_conf();
  }

 protected:
  ControlConf control_conf_;
  LonControllerConf lon_controller_conf_;
};

TEST_F(PidControllerTest, StationPidController) {
AINFO<<"(DMCZP) EnteringMethod: TEST_F";
  PidConf pid_conf = lon_controller_conf_.station_pid_conf();
  PIDController pid_controller;
  pid_controller.Init(pid_conf);
  pid_controller.Reset();
  double dt = 0.01;
  EXPECT_NEAR(pid_controller.Control(0.0, dt), 0.0, 1e-6);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(0.1, dt), 0.01, 1e-6);
  pid_controller.Reset();
  double control_value = pid_controller.Control(-0.1, dt);
  EXPECT_NEAR(control_value, -0.01, 1e-6);
  dt = 0.0;
  EXPECT_EQ(pid_controller.Control(100, dt), control_value);
  EXPECT_FALSE(pid_controller.IntegratorHold());
}

TEST_F(PidControllerTest, SpeedPidController) {
AINFO<<"(DMCZP) EnteringMethod: TEST_F";
  PidConf pid_conf = lon_controller_conf_.low_speed_pid_conf();
  PIDController pid_controller;
  pid_controller.Init(pid_conf);
  pid_controller.Reset();
  double dt = 0.01;
  EXPECT_NEAR(pid_controller.Control(0.0, dt), 0.0, 1e-6);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(0.1, dt), 0.1505, 1e-6);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(-0.1, dt), -0.1505, 1e-6);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(500.0, dt), 750.3, 1e-6);
  EXPECT_EQ(pid_controller.IntegratorSaturationStatus(), 1);
  pid_controller.Reset();
  EXPECT_NEAR(pid_controller.Control(-500.0, dt), -750.3, 1e-6);
  EXPECT_EQ(pid_controller.IntegratorSaturationStatus(), -1);
}

}  // namespace control
}  // namespace apollo
