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

/**
 * @file
 **/

#include "modules/planning/common/path_boundary.h"

namespace apollo {
namespace planning {

PathBoundary::PathBoundary(const double start_s, const double delta_s,
                           std::vector<std::pair<double, double>> path_boundary)
    : start_s_(start_s),
      delta_s_(delta_s),
      boundary_(std::move(path_boundary)) {}
AINFO<<"(DMCZP) EnteringMethod: PathBoundary::PathBoundary";

double PathBoundary::start_s() const { return start_s_; }
AINFO<<"(DMCZP) EnteringMethod: PathBoundary::start_s";

double PathBoundary::delta_s() const { return delta_s_; }
AINFO<<"(DMCZP) EnteringMethod: PathBoundary::delta_s";

void PathBoundary::set_boundary(
    const std::vector<std::pair<double, double>>& boundary) {
AINFO<<"(DMCZP) EnteringMethod: PathBoundary::set_boundary";
  boundary_ = boundary;
}

const std::vector<std::pair<double, double>>& PathBoundary::boundary() const {
  return boundary_;
}

void PathBoundary::set_label(const std::string& label) { label_ = label; }
AINFO<<"(DMCZP) EnteringMethod: PathBoundary::set_label";

const std::string& PathBoundary::label() const { return label_; }
AINFO<<"(DMCZP) EnteringMethod: PathBoundary::label";

void PathBoundary::set_blocking_obstacle_id(const std::string& obs_id) {
AINFO<<"(DMCZP) EnteringMethod: PathBoundary::set_blocking_obstacle_id";
  blocking_obstacle_id_ = obs_id;
}

const std::string& PathBoundary::blocking_obstacle_id() const {
AINFO<<"(DMCZP) EnteringMethod: PathBoundary::blocking_obstacle_id";
  return blocking_obstacle_id_;
}

}  // namespace planning
}  // namespace apollo
