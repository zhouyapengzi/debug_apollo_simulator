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
 * @file os_interface_test.cc
 * @brief test v2x proxy module and apollo os interface
 */

#include "modules/v2x/v2x_proxy/os_interface/os_interface.h"

#include "gtest/gtest.h"

namespace apollo {
namespace v2x {

TEST(OsInterFaceTest, Construct) {
AINFO<<"(DMCZP) EnteringMethod: TEST";
  apollo::cyber::Init("os_inteface_test");
  OsInterFace os_interface;
  EXPECT_TRUE(os_interface.InitFlag());
}
}  // namespace v2x
}  // namespace apollo
