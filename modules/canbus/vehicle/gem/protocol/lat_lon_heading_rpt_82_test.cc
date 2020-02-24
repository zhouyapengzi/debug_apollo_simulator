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

#include "modules/canbus/vehicle/gem/protocol/lat_lon_heading_rpt_82.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace gem {

class Latlonheadingrpt82Test : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Latlonheadingrpt82Test, reset) {
AINFO<<"(DMCZP) EnteringMethod: TEST_F";
  Latlonheadingrpt82 Latlonheading;
  int32_t length = 8;
  ChassisDetail chassis_detail;
  uint8_t bytes[8] = {0x01, 0x02, 0x03, 0x04, 0x11, 0x12, 0x13, 0x14};
  Latlonheading.Parse(bytes, length, &chassis_detail);
  EXPECT_DOUBLE_EQ(chassis_detail.gem().lat_lon_heading_rpt_82().heading(),
                   48.84);
  EXPECT_DOUBLE_EQ(
      chassis_detail.gem().lat_lon_heading_rpt_82().longitude_seconds(), 18);
  EXPECT_DOUBLE_EQ(
      chassis_detail.gem().lat_lon_heading_rpt_82().longitude_minutes(), 17);
  EXPECT_DOUBLE_EQ(
      chassis_detail.gem().lat_lon_heading_rpt_82().longitude_degrees(), 4);
  EXPECT_DOUBLE_EQ(
      chassis_detail.gem().lat_lon_heading_rpt_82().latitude_seconds(), 3);
  EXPECT_DOUBLE_EQ(
      chassis_detail.gem().lat_lon_heading_rpt_82().latitude_minutes(), 2);
  EXPECT_DOUBLE_EQ(
      chassis_detail.gem().lat_lon_heading_rpt_82().latitude_degrees(), 1);
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
