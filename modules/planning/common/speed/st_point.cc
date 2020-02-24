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
 * @file st_point.cpp
 **/

#include "modules/planning/common/speed/st_point.h"

#include "modules/common/util/string_util.h"

namespace apollo {
namespace planning {

using apollo::common::util::StringPrintf;

STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}
AINFO<<"(DMCZP) EnteringMethod: STPoint::STPoint";

STPoint::STPoint(const common::math::Vec2d& vec2d_point) : Vec2d(vec2d_point) {}
AINFO<<"(DMCZP) EnteringMethod: STPoint::STPoint";

double STPoint::s() const { return y_; }
AINFO<<"(DMCZP) EnteringMethod: STPoint::s";

double STPoint::t() const { return x_; }
AINFO<<"(DMCZP) EnteringMethod: STPoint::t";

void STPoint::set_s(const double s) { y_ = s; }
AINFO<<"(DMCZP) EnteringMethod: STPoint::set_s";

void STPoint::set_t(const double t) { x_ = t; }
AINFO<<"(DMCZP) EnteringMethod: STPoint::set_t";

std::string STPoint::DebugString() const {
AINFO<<"(DMCZP) EnteringMethod: STPoint::DebugString";
  return StringPrintf("{ \"s\" : %.6f, \"t\" : %.6f }", s(), t());
}

}  // namespace planning
}  // namespace apollo
