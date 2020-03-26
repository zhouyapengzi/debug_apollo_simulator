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
#include "modules/prediction/common/environment_features.h"

#include "cyber/common/log.h"

namespace apollo {
namespace prediction {

void EnvironmentFeatures::set_ego_position(const double x, const double y) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::set_ego_position";

  ego_position_.set_x(x);
  ego_position_.set_y(y);
  ego_position_.set_z(0.0);

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::set_ego_position";
 }

const apollo::common::Point3D& EnvironmentFeatures::get_ego_position() const {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::get_ego_position";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: EnvironmentFeatures::get_ego_position";
  return ego_position_;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::get_ego_position";
 }

void EnvironmentFeatures::set_ego_speed(const double ego_speed) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::set_ego_speed";

  ego_speed_ = ego_speed;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::set_ego_speed";
 }

double EnvironmentFeatures::get_ego_speed() const {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::get_ego_speed";
 
  AINFO<<"(DMCZP) 
  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::get_ego_speed";
 (return) LeaveMethod: EnvironmentFeatures::get_ego_speed";
  return ego_speed_; }

void EnvironmentFeatures::set_ego_heading(const double ego_heading) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::set_ego_heading";

  ego_heading_ = ego_heading;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::set_ego_heading";
 }

double EnvironmentFeatures::get_ego_heading() const {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::get_ego_heading";
 
  AINFO<<"(DMCZP) (r
  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::get_ego_heading";
 eturn) LeaveMethod: EnvironmentFeatures::get_ego_heading";
  return ego_heading_; }

void EnvironmentFeatures::set_ego_acceleration(const double ego_acceleration) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::set_ego_acceleration";

  ego_acceleration_ = ego_acceleration;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::set_ego_acceleration";
 }

double EnvironmentFeatures::get_ego_acceleration() const {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::get_ego_acceleration";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: EnvironmentFeatures::get_ego_acceleration";
  return ego_acceleration_;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::get_ego_acceleration";
 }

bool EnvironmentFeatures::has_ego_lane() const {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::has_ego_lane";
 
  AINFO<<"(DMCZP) (re
  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::has_ego_lane";
 turn) LeaveMethod: EnvironmentFeatures::has_ego_lane";
  return has_ego_lane_; }

void EnvironmentFeatures::reset_ego_lane() {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::reset_ego_lane";

  has_ego_lane_ = false;
  ego_lane_id_ = "";
  ego_lane_s_ = -1.0;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::reset_ego_lane";
 }

void EnvironmentFeatures::SetEgoLane(const std::string& lane_id,
                                     const double lane_s) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::SetEgoLane";

  has_ego_lane_ = true;
  ego_lane_id_ = lane_id;
  ego_lane_s_ = lane_s;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::SetEgoLane";
 }

std::pair<std::string, double> EnvironmentFeatures::GetEgoLane() const {
  CHECK(has_ego_lane_);
  return {ego_lane_id_, ego_lane_s_};
}

bool EnvironmentFeatures::has_left_neighbor_lane() const {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::has_left_neighbor_lane";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: EnvironmentFeatures::has_left_neighbor_lane";
  return has_left_neighbor_lane_;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::has_left_neighbor_lane";
 }

void EnvironmentFeatures::reset_left_neighbor_lane() {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::reset_left_neighbor_lane";

  has_left_neighbor_lane_ = false;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::reset_left_neighbor_lane";
 }

void EnvironmentFeatures::SetLeftNeighborLane(const std::string& lane_id,
                                              const double lane_s) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::SetLeftNeighborLane";

  has_left_neighbor_lane_ = true;
  left_neighbor_lane_id_ = lane_id;
  left_neighbor_lane_s_ = lane_s;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::SetLeftNeighborLane";
 }

std::pair<std::string, double> EnvironmentFeatures::GetLeftNeighborLane()
    const {
  CHECK(has_left_neighbor_lane_);
  return {left_neighbor_lane_id_, left_neighbor_lane_s_};
}

bool EnvironmentFeatures::has_right_neighbor_lane() const {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::has_right_neighbor_lane";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: EnvironmentFeatures::has_right_neighbor_lane";
  return has_right_neighbor_lane_;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::has_right_neighbor_lane";
 }

void EnvironmentFeatures::reset_right_neighbor_lane() {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::reset_right_neighbor_lane";

  has_right_neighbor_lane_ = false;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::reset_right_neighbor_lane";
 }

void EnvironmentFeatures::SetRightNeighborLane(const std::string& lane_id,
                                               const double lane_s) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::SetRightNeighborLane";

  has_right_neighbor_lane_ = true;
  right_neighbor_lane_id_ = lane_id;
  right_neighbor_lane_s_ = lane_s;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::SetRightNeighborLane";
 }

std::pair<std::string, double> EnvironmentFeatures::GetRightNeighborLane()
    const {
  CHECK(has_right_neighbor_lane_);
  return {right_neighbor_lane_id_, right_neighbor_lane_s_};
}

bool EnvironmentFeatures::has_front_junction() const {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::has_front_junction";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: EnvironmentFeatures::has_front_junction";
  return has_front_junction_;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::has_front_junction";
 }

void EnvironmentFeatures::reset_front_junction() {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::reset_front_junction";

  has_front_junction_ = false;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::reset_front_junction";
 }

void EnvironmentFeatures::SetFrontJunction(const std::string& junction_id,
                                           const double dist) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::SetFrontJunction";

  has_front_junction_ = true;
  front_junction_id_ = junction_id;
  dist_to_front_junction_ = dist;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::SetFrontJunction";
 }

std::pair<std::string, double> EnvironmentFeatures::GetFrontJunction() const {
  CHECK(has_front_junction_);
  return {front_junction_id_, dist_to_front_junction_};
}

void EnvironmentFeatures::AddObstacleId(const int obstacle_id) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::AddObstacleId";

  obstacle_ids_.push_back(obstacle_id);

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::AddObstacleId";
 }

const std::vector<int>& EnvironmentFeatures::get_obstacle_ids() const {
  return obstacle_ids_;
}

const std::unordered_set<std::string>&
EnvironmentFeatures::nonneglectable_reverse_lanes() const {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::nonneglectable_reverse_lanes";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: EnvironmentFeatures::nonneglectable_reverse_lanes";
  return nonneglectable_reverse_lanes_;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::nonneglectable_reverse_lanes";
 }

void EnvironmentFeatures::AddNonneglectableReverseLanes(
    const std::string& lane_id) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::AddNonneglectableReverseLanes";

  nonneglectable_reverse_lanes_.insert(lane_id);

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::AddNonneglectableReverseLanes";
 }

bool EnvironmentFeatures::RemoveNonneglectableReverseLanes(
    const std::string& lane_id) {
  AINFO<<"(DMCZP) EnteringMethod: EnvironmentFeatures::RemoveNonneglectableReverseLanes";

  if (nonneglectable_reverse_lanes_.find(lane_id) ==
      nonneglectable_reverse_lanes_.end()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: EnvironmentFeatures::RemoveNonneglectableReverseLanes";
  return false;
  }
  nonneglectable_reverse_lanes_.erase(lane_id);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: EnvironmentFeatures::RemoveNonneglectableReverseLanes";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: EnvironmentFeatures::RemoveNonneglectableReverseLanes";
 }

}  // namespace prediction
}  // namespace apollo
