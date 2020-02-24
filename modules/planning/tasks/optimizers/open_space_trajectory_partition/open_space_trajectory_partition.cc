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

/**
 * @file
 **/
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/tasks/optimizers/open_space_trajectory_partition/open_space_trajectory_partition.h"

#include "modules/common/math/polygon2d.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::NormalizeAngle;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;

OpenSpaceTrajectoryPartition::OpenSpaceTrajectoryPartition(
    const TaskConfig& config)
    : TrajectoryOptimizer(config) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::OpenSpaceTrajectoryPartition";
  open_space_trajectory_partition_config_ =
      config_.open_space_trajectory_partition_config();
  heading_search_range_ =
      open_space_trajectory_partition_config_.heading_search_range();
  heading_track_range_ =
      open_space_trajectory_partition_config_.heading_track_range();
  distance_search_range_ =
      open_space_trajectory_partition_config_.distance_search_range();
  heading_offset_to_midpoint_ =
      open_space_trajectory_partition_config_.heading_offset_to_midpoint();
  lateral_offset_to_midpoint_ =
      open_space_trajectory_partition_config_.lateral_offset_to_midpoint();
  longitudinal_offset_to_midpoint_ =
      open_space_trajectory_partition_config_.longitudinal_offset_to_midpoint();
  vehicle_box_iou_threshold_to_midpoint_ =
      open_space_trajectory_partition_config_
          .vehicle_box_iou_threshold_to_midpoint();
  linear_velocity_threshold_on_ego_ = open_space_trajectory_partition_config_
                                          .linear_velocity_threshold_on_ego();

  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  ego_length_ = vehicle_param_.length();
  ego_width_ = vehicle_param_.width();
  shift_distance_ = ego_length_ / 2.0 - vehicle_param_.back_edge_to_center();
  wheel_base_ = vehicle_param_.wheel_base();
}

void OpenSpaceTrajectoryPartition::Restart() {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::Restart";
  auto* current_gear_status =
      frame_->mutable_open_space_info()->mutable_gear_switch_states();
  current_gear_status->gear_switching_flag = false;
  current_gear_status->gear_shift_period_finished = true;
  current_gear_status->gear_shift_period_started = true;
  current_gear_status->gear_shift_period_time = 0.0;
  current_gear_status->gear_shift_start_time = 0.0;
  current_gear_status->gear_shift_position = canbus::Chassis::GEAR_DRIVE;
}

Status OpenSpaceTrajectoryPartition::Process() {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::Process";
  const auto& open_space_info = frame_->open_space_info();
  auto open_space_info_ptr = frame_->mutable_open_space_info();
  const auto& stitched_trajectory_result =
      open_space_info.stitched_trajectory_result();

  auto* interpolated_trajectory_result_ptr =
      open_space_info_ptr->mutable_interpolated_trajectory_result();

  InterpolateTrajectory(stitched_trajectory_result,
                        interpolated_trajectory_result_ptr);

  auto* paritioned_trajectories =
      open_space_info_ptr->mutable_paritioned_trajectories();

  PartitionTrajectory(*interpolated_trajectory_result_ptr,
                      paritioned_trajectories);

  // Choose the one to follow based on the closest partitioned trajectory
  size_t trajectories_size = paritioned_trajectories->size();

  size_t current_trajectory_index = 0;
  size_t current_trajectory_point_index = 0;
  bool flag_change_to_next = false;

  // Vehicle related information used to choose closest point
  UpdateVehicleInfo();

  std::priority_queue<std::pair<std::pair<size_t, size_t>, double>,
                      std::vector<std::pair<std::pair<size_t, size_t>, double>>,
                      pair_comp_>
      closest_point_on_trajs;

  std::vector<std::string> trajectories_encodings;
  for (size_t i = 0; i < trajectories_size; ++i) {
    const auto& trajectory = paritioned_trajectories->at(i).first;
    std::string trajectory_encoding;
    if (!EncodeTrajectory(trajectory, &trajectory_encoding)) {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Trajectory empty in trajectory partition");
    }
    trajectories_encodings.emplace_back(std::move(trajectory_encoding));
  }

  for (size_t i = 0; i < trajectories_size; ++i) {
    const auto& gear = paritioned_trajectories->at(i).second;
    const auto& trajectory = paritioned_trajectories->at(i).first;
    size_t trajectory_size = trajectory.size();
    CHECK_GT(trajectory_size, 0);

    flag_change_to_next = CheckReachTrajectoryEnd(
        trajectory, gear, trajectories_size, i, &current_trajectory_index,
        &current_trajectory_point_index);
    if (flag_change_to_next &&
        !CheckTrajTraversed(trajectories_encodings[current_trajectory_index])) {
      UpdateTrajHistory(trajectories_encodings[current_trajectory_index]);
      break;
    }

    // Choose the closest point to track
    std::priority_queue<std::pair<size_t, double>,
                        std::vector<std::pair<size_t, double>>, comp_>
        closest_point;
    for (size_t j = 0; j < trajectory_size; ++j) {
      const TrajectoryPoint& trajectory_point = trajectory.at(j);
      const PathPoint& path_point = trajectory_point.path_point();
      const double path_point_x = path_point.x();
      const double path_point_y = path_point.y();
      const double path_point_theta = path_point.theta();
      const Vec2d tracking_vector(path_point_x - ego_x_, path_point_y - ego_y_);
      const double distance = tracking_vector.Length();
      const double tracking_direction = tracking_vector.Angle();
      const double traj_point_moving_direction =
          gear == canbus::Chassis::GEAR_REVERSE
              ? NormalizeAngle(path_point_theta + M_PI)
              : path_point_theta;
      const double head_track_difference = std::abs(
          NormalizeAngle(tracking_direction - vehicle_moving_direction_));
      const double heading_search_difference = std::abs(NormalizeAngle(
          traj_point_moving_direction - vehicle_moving_direction_));

      if (distance < distance_search_range_ &&
          heading_search_difference < heading_search_range_ &&
          head_track_difference < heading_track_range_) {
        // get vehicle box and path point box, compute IOU
        Box2d path_point_box({path_point_x, path_point_y}, path_point_theta,
                             ego_length_, ego_width_);
        Vec2d shift_vec{shift_distance_ * std::cos(path_point_theta),
                        shift_distance_ * std::sin(path_point_theta)};
        path_point_box.Shift(shift_vec);
        double iou_ratio =
            Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_point_box));
        closest_point.emplace(j, iou_ratio);
      }
    }

    if (!closest_point.empty()) {
      size_t closest_point_index = closest_point.top().first;
      double max_iou_ratio = closest_point.top().second;
      closest_point_on_trajs.emplace(std::make_pair(i, closest_point_index),
                                     max_iou_ratio);
    }
  }

  if (!flag_change_to_next) {
    bool use_fail_safe_search = false;
    if (closest_point_on_trajs.empty()) {
      use_fail_safe_search = true;
    } else {
      bool closest_and_not_repeated_traj_found = false;
      while (!closest_point_on_trajs.empty()) {
        current_trajectory_index = closest_point_on_trajs.top().first.first;
        current_trajectory_point_index =
            closest_point_on_trajs.top().first.second;
        if (CheckTrajTraversed(
                trajectories_encodings[current_trajectory_index])) {
          closest_point_on_trajs.pop();
        } else {
          closest_and_not_repeated_traj_found = true;
          UpdateTrajHistory(trajectories_encodings[current_trajectory_index]);
          break;
        }
      }
      if (!closest_and_not_repeated_traj_found) {
        use_fail_safe_search = true;
      }
    }

    if (use_fail_safe_search) {
      if (!UseFailSafeSearch(*paritioned_trajectories, trajectories_encodings,
                             &current_trajectory_index,
                             &current_trajectory_point_index)) {
        std::string msg("Fail to find nearest trajectory point to follow");
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
    }
  }

  auto* chosen_paritioned_trajectory =
      open_space_info_ptr->mutable_chosen_paritioned_trajectory();

  if (FLAGS_use_gear_shift_trajectory) {
    if (InsertGearShiftTrajectory(flag_change_to_next, current_trajectory_index,
                                  open_space_info.paritioned_trajectories(),
                                  chosen_paritioned_trajectory)) {
      return Status::OK();
    }
  }

  auto* mutable_trajectory =
      open_space_info_ptr->mutable_stitched_trajectory_result();
  AdjustRelativeTimeAndS(open_space_info.paritioned_trajectories(),
                         current_trajectory_index,
                         current_trajectory_point_index, mutable_trajectory,
                         chosen_paritioned_trajectory);
  return Status::OK();
}

void OpenSpaceTrajectoryPartition::InterpolateTrajectory(
    const DiscretizedTrajectory& stitched_trajectory_result,
    DiscretizedTrajectory* interpolated_trajectory) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::InterpolateTrajectory";
  if (FLAGS_use_iterative_anchoring_smoother) {
    *interpolated_trajectory = stitched_trajectory_result;
    return;
  }
  interpolated_trajectory->clear();
  size_t interpolated_pieces_num =
      open_space_trajectory_partition_config_.interpolated_pieces_num();
  CHECK_GT(stitched_trajectory_result.size(), 0);
  CHECK_GT(interpolated_pieces_num, 0);
  size_t trajectory_to_be_partitioned_intervals_num =
      stitched_trajectory_result.size() - 1;
  size_t interpolated_points_num = interpolated_pieces_num - 1;
  for (size_t i = 0; i < trajectory_to_be_partitioned_intervals_num; ++i) {
    double relative_time_interval =
        (stitched_trajectory_result.at(i + 1).relative_time() -
         stitched_trajectory_result.at(i).relative_time()) /
        static_cast<double>(interpolated_pieces_num);
    interpolated_trajectory->push_back(stitched_trajectory_result.at(i));
    for (size_t j = 0; j < interpolated_points_num; ++j) {
      double relative_time =
          stitched_trajectory_result.at(i).relative_time() +
          (static_cast<double>(j) + 1.0) * relative_time_interval;
      interpolated_trajectory->emplace_back(
          common::math::InterpolateUsingLinearApproximation(
              stitched_trajectory_result.at(i),
              stitched_trajectory_result.at(i + 1), relative_time));
    }
  }
  interpolated_trajectory->push_back(stitched_trajectory_result.back());
}

void OpenSpaceTrajectoryPartition::UpdateVehicleInfo() {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::UpdateVehicleInfo";
  const common::VehicleState& vehicle_state = frame_->vehicle_state();
  ego_theta_ = vehicle_state.heading();
  ego_x_ = vehicle_state.x();
  ego_y_ = vehicle_state.y();
  ego_v_ = vehicle_state.linear_velocity();
  Box2d box({ego_x_, ego_y_}, ego_theta_, ego_length_, ego_width_);
  ego_box_ = std::move(box);
  Vec2d ego_shift_vec{shift_distance_ * std::cos(ego_theta_),
                      shift_distance_ * std::sin(ego_theta_)};
  ego_box_.Shift(ego_shift_vec);
  vehicle_moving_direction_ =
      vehicle_state.gear() == canbus::Chassis::GEAR_REVERSE
          ? NormalizeAngle(ego_theta_ + M_PI)
          : ego_theta_;
}

bool OpenSpaceTrajectoryPartition::EncodeTrajectory(
    const DiscretizedTrajectory& trajectory, std::string* const encoding) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::EncodeTrajectory";
  if (trajectory.empty()) {
    AERROR << "Fail to encode trajectory because it is empty";
    return false;
  }
  constexpr double encoding_origin_x = 58700.0;
  constexpr double encoding_origin_y = 4141000.0;
  const auto& init_path_point = trajectory.front().path_point();
  const auto& last_path_point = trajectory.back().path_point();

  const std::string init_point_x_encoding = std::to_string(
      static_cast<int>((init_path_point.x() - encoding_origin_x) * 1000.0));
  const std::string init_point_y_encoding = std::to_string(
      static_cast<int>((init_path_point.y() - encoding_origin_y) * 1000.0));
  const std::string init_point_heading_encoding =
      std::to_string(static_cast<int>(init_path_point.theta() * 10000.0));
  const std::string last_point_x_encoding = std::to_string(
      static_cast<int>((last_path_point.x() - encoding_origin_x) * 1000.0));
  const std::string last_point_y_encoding = std::to_string(
      static_cast<int>((last_path_point.y() - encoding_origin_y) * 1000.0));
  const std::string last_point_heading_encoding =
      std::to_string(static_cast<int>(last_path_point.theta() * 10000.0));

  const std::string init_point_encoding = init_point_x_encoding + "_" +
                                          init_point_y_encoding + "_" +
                                          init_point_heading_encoding;
  const std::string last_point_encoding = last_point_x_encoding + "_" +
                                          last_point_y_encoding + "_" +
                                          last_point_heading_encoding;

  *encoding = init_point_encoding + "/" + last_point_encoding;
  return true;
}

bool OpenSpaceTrajectoryPartition::CheckTrajTraversed(
    const std::string& trajectory_encoding_to_check) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::CheckTrajTraversed";
  const auto& index_history = PlanningContext::Instance()
                                  ->open_space_info()
                                  .partitioned_trajectories_index_history;
  const size_t index_history_length = index_history.size();
  if (index_history_length <= 1) {
    return false;
  }
  for (size_t i = 0; i < index_history_length - 1; ++i) {
    if (index_history[i] == trajectory_encoding_to_check) {
      return true;
    }
  }
  return false;
}

void OpenSpaceTrajectoryPartition::UpdateTrajHistory(
    const std::string& chosen_trajectory_encoding) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::UpdateTrajHistory";
  auto* trajectory_history = &(PlanningContext::Instance()
                                   ->mutable_open_space_info()
                                   ->partitioned_trajectories_index_history);
  if (trajectory_history->empty()) {
    trajectory_history->push_back(chosen_trajectory_encoding);
    return;
  }
  if (trajectory_history->back() == chosen_trajectory_encoding) {
    return;
  }
  trajectory_history->push_back(chosen_trajectory_encoding);
}

void OpenSpaceTrajectoryPartition::PartitionTrajectory(
    const DiscretizedTrajectory& raw_trajectory,
    std::vector<TrajGearPair>* paritioned_trajectories) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::PartitionTrajectory";
  CHECK_NOTNULL(paritioned_trajectories);

  size_t horizon = raw_trajectory.size();

  paritioned_trajectories->clear();
  paritioned_trajectories->emplace_back();
  TrajGearPair* current_trajectory_gear = &(paritioned_trajectories->back());

  auto* trajectory = &(current_trajectory_gear->first);
  auto* gear = &(current_trajectory_gear->second);

  // Decide initial gear
  const auto& first_path_point = raw_trajectory.front().path_point();
  const auto& second_path_point = raw_trajectory[1].path_point();
  double heading_angle = first_path_point.theta();
  const Vec2d init_tracking_vector(
      second_path_point.x() - first_path_point.x(),
      second_path_point.y() - first_path_point.y());
  double tracking_angle = init_tracking_vector.Angle();
  *gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
              (M_PI_2)
          ? canbus::Chassis::GEAR_DRIVE
          : canbus::Chassis::GEAR_REVERSE;

  // Set accumulated distance
  Vec2d last_pos_vec(first_path_point.x(), first_path_point.y());
  double distance_s = 0.0;

  for (size_t i = 0; i < horizon - 1; ++i) {
    const TrajectoryPoint& trajectory_point = raw_trajectory.at(i);
    const TrajectoryPoint& next_trajectory_point = raw_trajectory.at(i + 1);

    // Check gear change
    heading_angle = trajectory_point.path_point().theta();
    const Vec2d tracking_vector(next_trajectory_point.path_point().x() -
                                    trajectory_point.path_point().x(),
                                next_trajectory_point.path_point().y() -
                                    trajectory_point.path_point().y());
    tracking_angle = tracking_vector.Angle();
    auto cur_gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
                (M_PI_2)
            ? canbus::Chassis::GEAR_DRIVE
            : canbus::Chassis::GEAR_REVERSE;

    if (cur_gear != *gear) {
      LoadTrajectoryPoint(trajectory_point, *gear, &last_pos_vec, &distance_s,
                          trajectory);

      paritioned_trajectories->emplace_back();
      current_trajectory_gear = &(paritioned_trajectories->back());
      current_trajectory_gear->second = cur_gear;
      distance_s = 0.0;
    }

    trajectory = &(current_trajectory_gear->first);
    gear = &(current_trajectory_gear->second);

    LoadTrajectoryPoint(trajectory_point, *gear, &last_pos_vec, &distance_s,
                        trajectory);
  }

  const TrajectoryPoint& last_trajectory_point = raw_trajectory.back();
  LoadTrajectoryPoint(last_trajectory_point, *gear, &last_pos_vec, &distance_s,
                      trajectory);
}

void OpenSpaceTrajectoryPartition::LoadTrajectoryPoint(
    const TrajectoryPoint& trajectory_point,
    const canbus::Chassis::GearPosition& gear, Vec2d* last_pos_vec,
    double* distance_s, DiscretizedTrajectory* current_trajectory) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::LoadTrajectoryPoint";
  current_trajectory->emplace_back();
  TrajectoryPoint* point = &(current_trajectory->back());
  point->set_relative_time(trajectory_point.relative_time());
  point->mutable_path_point()->set_x(trajectory_point.path_point().x());
  point->mutable_path_point()->set_y(trajectory_point.path_point().y());
  point->mutable_path_point()->set_theta(trajectory_point.path_point().theta());
  point->set_v(trajectory_point.v());
  point->mutable_path_point()->set_s(*distance_s);
  Vec2d cur_pos_vec(trajectory_point.path_point().x(),
                    trajectory_point.path_point().y());
  *distance_s += (gear == canbus::Chassis::GEAR_REVERSE ? -1.0 : 1.0) *
                 (cur_pos_vec.DistanceTo(*last_pos_vec));
  *last_pos_vec = cur_pos_vec;
  point->mutable_path_point()->set_kappa(std::tan(trajectory_point.steer()) /
                                         wheel_base_);
  point->set_a(trajectory_point.a());
}

bool OpenSpaceTrajectoryPartition::CheckReachTrajectoryEnd(
    const DiscretizedTrajectory& trajectory,
    const canbus::Chassis::GearPosition& gear, const size_t trajectories_size,
    const size_t trajectories_index, size_t* current_trajectory_index,
    size_t* current_trajectory_point_index) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::CheckReachTrajectoryEnd";
  // Check if have reached endpoint of trajectory
  const TrajectoryPoint& trajectory_end_point = trajectory.back();
  const size_t trajectory_size = trajectory.size();
  const PathPoint& path_end_point = trajectory_end_point.path_point();
  const double path_end_point_x = path_end_point.x();
  const double path_end_point_y = path_end_point.y();
  const Vec2d tracking_vector(ego_x_ - path_end_point_x,
                              ego_y_ - path_end_point_y);
  const double path_end_point_theta = path_end_point.theta();
  const double included_angle =
      NormalizeAngle(path_end_point_theta - tracking_vector.Angle());
  const double distance_to_trajs_end =
      std::sqrt((path_end_point_x - ego_x_) * (path_end_point_x - ego_x_) +
                (path_end_point_y - ego_y_) * (path_end_point_y - ego_y_));
  const double lateral_offset =
      std::abs(distance_to_trajs_end * std::sin(included_angle));
  const double longitudinal_offset =
      std::abs(distance_to_trajs_end * std::cos(included_angle));
  const double traj_end_point_moving_direction =
      gear == canbus::Chassis::GEAR_REVERSE
          ? NormalizeAngle(path_end_point_theta + M_PI)
          : path_end_point_theta;

  const double heading_search_to_trajs_end = std::abs(NormalizeAngle(
      traj_end_point_moving_direction - vehicle_moving_direction_));

  // If close to the end point, start on the next trajectory
  double end_point_iou_ratio = 0.0;
  if (lateral_offset < lateral_offset_to_midpoint_ &&
      longitudinal_offset < longitudinal_offset_to_midpoint_ &&
      heading_search_to_trajs_end < heading_offset_to_midpoint_ &&
      std::abs(ego_v_) < linear_velocity_threshold_on_ego_) {
    // get vehicle box and path point box, compare with a threadhold in IOU
    Box2d path_end_point_box({path_end_point_x, path_end_point_y},
                             path_end_point_theta, ego_length_, ego_width_);
    Vec2d shift_vec{shift_distance_ * std::cos(path_end_point_theta),
                    shift_distance_ * std::sin(path_end_point_theta)};
    path_end_point_box.Shift(shift_vec);
    end_point_iou_ratio =
        Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_end_point_box));

    if (end_point_iou_ratio > vehicle_box_iou_threshold_to_midpoint_) {
      if (trajectories_index + 1 >= trajectories_size) {
        *current_trajectory_index = trajectories_size - 1;
        *current_trajectory_point_index = trajectory_size - 1;
      } else {
        *current_trajectory_index = trajectories_index + 1;
        *current_trajectory_point_index = 0;
      }
      ADEBUG << "Reach the end of a trajectory, switching to next one";
      return true;
    }
  }

  ADEBUG << "Vehicle did not reach end of a trajectory with conditions for "
            "lateral distance_check: "
         << (lateral_offset < lateral_offset_to_midpoint_)
         << " and actual lateral distance: " << lateral_offset
         << "; longitudinal distance_check: "
         << (longitudinal_offset < longitudinal_offset_to_midpoint_)
         << " and actual longitudinal distance: " << longitudinal_offset
         << "; heading_check: "
         << (heading_search_to_trajs_end < heading_offset_to_midpoint_)
         << " with actual heading: " << heading_search_to_trajs_end
         << "; velocity_check: "
         << (std::abs(ego_v_) < linear_velocity_threshold_on_ego_)
         << " with actual linear velocity: " << ego_v_ << "; iou_check: "
         << (end_point_iou_ratio > vehicle_box_iou_threshold_to_midpoint_)
         << " with actual iou: " << end_point_iou_ratio;
  return false;
}

bool OpenSpaceTrajectoryPartition::UseFailSafeSearch(
    const std::vector<TrajGearPair>& paritioned_trajectories,
    const std::vector<std::string>& trajectories_encodings,
    size_t* current_trajectory_index, size_t* current_trajectory_point_index) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::UseFailSafeSearch";
  AERROR << "Trajectory paritition fail, using failsafe search";
  const size_t trajectories_size = paritioned_trajectories.size();
  std::priority_queue<std::pair<std::pair<size_t, size_t>, double>,
                      std::vector<std::pair<std::pair<size_t, size_t>, double>>,
                      pair_comp_>
      failsafe_closest_point_on_trajs;
  for (size_t i = 0; i < trajectories_size; ++i) {
    const auto& trajectory = paritioned_trajectories.at(i).first;
    size_t trajectory_size = trajectory.size();
    CHECK_GT(trajectory_size, 0);
    std::priority_queue<std::pair<size_t, double>,
                        std::vector<std::pair<size_t, double>>, comp_>
        failsafe_closest_point;

    for (size_t j = 0; j < trajectory_size; ++j) {
      const TrajectoryPoint& trajectory_point = trajectory.at(j);
      const PathPoint& path_point = trajectory_point.path_point();
      const double path_point_x = path_point.x();
      const double path_point_y = path_point.y();
      const double path_point_theta = path_point.theta();
      const Vec2d tracking_vector(path_point_x - ego_x_, path_point_y - ego_y_);
      const double distance = tracking_vector.Length();
      if (distance < distance_search_range_) {
        // get vehicle box and path point box, compute IOU
        Box2d path_point_box({path_point_x, path_point_y}, path_point_theta,
                             ego_length_, ego_width_);
        Vec2d shift_vec{shift_distance_ * std::cos(path_point_theta),
                        shift_distance_ * std::sin(path_point_theta)};
        path_point_box.Shift(shift_vec);
        double iou_ratio =
            Polygon2d(ego_box_).ComputeIoU(Polygon2d(path_point_box));
        failsafe_closest_point.emplace(j, iou_ratio);
      }
    }
    if (!failsafe_closest_point.empty()) {
      size_t closest_point_index = failsafe_closest_point.top().first;
      double max_iou_ratio = failsafe_closest_point.top().second;
      failsafe_closest_point_on_trajs.emplace(
          std::make_pair(i, closest_point_index), max_iou_ratio);
    }
  }
  if (failsafe_closest_point_on_trajs.empty()) {
    return false;
  } else {
    bool closest_and_not_repeated_traj_found = false;
    while (!failsafe_closest_point_on_trajs.empty()) {
      *current_trajectory_index =
          failsafe_closest_point_on_trajs.top().first.first;
      *current_trajectory_point_index =
          failsafe_closest_point_on_trajs.top().first.second;
      if (CheckTrajTraversed(
              trajectories_encodings[*current_trajectory_index])) {
        failsafe_closest_point_on_trajs.pop();
      } else {
        closest_and_not_repeated_traj_found = true;
        UpdateTrajHistory(trajectories_encodings[*current_trajectory_index]);
        return true;
      }
    }
    if (!closest_and_not_repeated_traj_found) {
      return false;
    }

    return true;
  }
}

bool OpenSpaceTrajectoryPartition::InsertGearShiftTrajectory(
    const bool flag_change_to_next, const size_t current_trajectory_index,
    const std::vector<TrajGearPair>& paritioned_trajectories,
    TrajGearPair* gear_switch_idle_time_trajectory) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::InsertGearShiftTrajectory";
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::GenerateGearShiftTrajectory";
  const auto* last_frame = FrameHistory::Instance()->Latest();
  const auto& last_gear_status =
      last_frame->open_space_info().gear_switch_states();
  auto* current_gear_status =
      frame_->mutable_open_space_info()->mutable_gear_switch_states();
  *(current_gear_status) = last_gear_status;

  if (flag_change_to_next || !current_gear_status->gear_shift_period_finished) {
    current_gear_status->gear_shift_period_finished = false;
    if (current_gear_status->gear_shift_period_started) {
      current_gear_status->gear_shift_start_time = Clock::NowInSeconds();
      current_gear_status->gear_shift_position =
          paritioned_trajectories.at(current_trajectory_index).second;
      current_gear_status->gear_shift_period_started = false;
    }
    if (current_gear_status->gear_shift_period_time >
        open_space_trajectory_partition_config_.gear_shift_period_duration()) {
      current_gear_status->gear_shift_period_finished = true;
      current_gear_status->gear_shift_period_started = true;
    } else {
      GenerateGearShiftTrajectory(current_gear_status->gear_shift_position,
                                  gear_switch_idle_time_trajectory);
      current_gear_status->gear_shift_period_time =
          Clock::NowInSeconds() - current_gear_status->gear_shift_start_time;
      return true;
    }
  }

  return true;
}

void OpenSpaceTrajectoryPartition::GenerateGearShiftTrajectory(
    const canbus::Chassis::GearPosition& gear_position,
    TrajGearPair* gear_switch_idle_time_trajectory) {
  gear_switch_idle_time_trajectory->first.clear();
  const double gear_shift_max_t =
      open_space_trajectory_partition_config_.gear_shift_max_t();
  const double gear_shift_unit_t =
      open_space_trajectory_partition_config_.gear_shift_unit_t();
  const double vehicle_x = frame_->vehicle_state().x();
  const double vehicle_y = frame_->vehicle_state().y();
  const double vehicle_heading = frame_->vehicle_state().heading();
  const double vehicle_kappa = frame_->vehicle_state().kappa();
  for (double t = 0.0; t < gear_shift_max_t; t += gear_shift_unit_t) {
    gear_switch_idle_time_trajectory->first.emplace_back();
    auto* trajectory_point = &(gear_switch_idle_time_trajectory->first.back());
    auto* path_point = trajectory_point->mutable_path_point();
    path_point->set_x(vehicle_x);
    path_point->set_y(vehicle_y);
    path_point->set_theta(vehicle_heading);
    path_point->set_kappa(vehicle_kappa);
    path_point->set_s(0.0);
    trajectory_point->set_v(0.0);
    trajectory_point->set_a(0.0);
    trajectory_point->set_relative_time(t);
  }
  gear_switch_idle_time_trajectory->second = gear_position;
}

void OpenSpaceTrajectoryPartition::AdjustRelativeTimeAndS(
    const std::vector<TrajGearPair>& paritioned_trajectories,
    const size_t current_trajectory_index,
    const size_t closest_trajectory_point_index,
    DiscretizedTrajectory* unpartitioned_trajectory_result,
    TrajGearPair* current_paritioned_trajectory) {
AINFO<<"(DMCZP) EnteringMethod: OpenSpaceTrajectoryPartition::AdjustRelativeTimeAndS";
  const size_t paritioned_trajectories_size = paritioned_trajectories.size();
  CHECK_GT(paritioned_trajectories_size, current_trajectory_index);

  // Reassign relative time and relative s to have the closest point as origin
  // point
  *(current_paritioned_trajectory) =
      paritioned_trajectories.at(current_trajectory_index);
  auto trajectory = &(current_paritioned_trajectory->first);
  double time_shift =
      trajectory->at(closest_trajectory_point_index).relative_time();
  double s_shift =
      trajectory->at(closest_trajectory_point_index).path_point().s();
  const size_t trajectory_size = trajectory->size();
  for (size_t i = 0; i < trajectory_size; ++i) {
    TrajectoryPoint* trajectory_point = &(trajectory->at(i));
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);
  }

  // Reassign relative t and s on stitched_trajectory_result for accurate next
  // frame stitching
  const size_t interpolated_pieces_num =
      open_space_trajectory_partition_config_.interpolated_pieces_num();
  const size_t unpartitioned_trajectory_size =
      unpartitioned_trajectory_result->size();
  size_t index_estimate = 0;
  for (size_t i = 0; i < current_trajectory_index; ++i) {
    index_estimate += paritioned_trajectories.at(i).first.size();
  }
  index_estimate += closest_trajectory_point_index;
  index_estimate /= interpolated_pieces_num;
  if (index_estimate >= unpartitioned_trajectory_size) {
    index_estimate = unpartitioned_trajectory_size - 1;
  }
  time_shift =
      unpartitioned_trajectory_result->at(index_estimate).relative_time();
  s_shift =
      unpartitioned_trajectory_result->at(index_estimate).path_point().s();
  for (size_t i = 0; i < unpartitioned_trajectory_size; ++i) {
    TrajectoryPoint* trajectory_point =
        &(unpartitioned_trajectory_result->at(i));
    trajectory_point->set_relative_time(trajectory_point->relative_time() -
                                        time_shift);
    trajectory_point->mutable_path_point()->set_s(
        trajectory_point->path_point().s() - s_shift);
  }
}

}  // namespace planning
}  // namespace apollo
