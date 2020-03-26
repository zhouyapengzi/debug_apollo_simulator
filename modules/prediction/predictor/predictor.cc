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

#include "modules/prediction/predictor/predictor.h"

#include <algorithm>

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;

int Predictor::NumOfTrajectories(const Obstacle& obstacle) {
  AINFO<<"(DMCZP) EnteringMethod: Predictor::NumOfTrajectories";

  CHECK_GT(obstacle.history_size(), 0);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::NumOfTrajectories";
  return obstacle.latest_feature().predicted_trajectory_size();

  AINFO<<"(DMCZP) LeaveMethod: Predictor::NumOfTrajectories";
 }

Trajectory Predictor::GenerateTrajectory(
    const std::vector<TrajectoryPoint>& points) {
  AINFO<<"(DMCZP) EnteringMethod: Predictor::GenerateTrajectory";

  Trajectory trajectory;
  *trajectory.mutable_trajectory_point() = {points.begin(), points.end()};
  
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::GenerateTrajectory";
  return trajectory;

  AINFO<<"(DMCZP) LeaveMethod: Predictor::GenerateTrajectory";
 }

void Predictor::SetEqualProbability(const double total_probability,
                                    const int start_index,
                                    Obstacle* obstacle_ptr) {
  AINFO<<"(DMCZP) EnteringMethod: Predictor::SetEqualProbability";

  int num = NumOfTrajectories(*obstacle_ptr);
  CHECK(num > start_index);

  const auto prob = total_probability / static_cast<double>(num - start_index);
  for (int i = start_index; i < num; ++i) {
    obstacle_ptr->mutable_latest_feature()
        ->mutable_predicted_trajectory(i)
        ->set_probability(prob);
  }

  AINFO<<"(DMCZP) LeaveMethod: Predictor::SetEqualProbability";
 }

void Predictor::Clear() {
  AINFO<<"(DMCZP) EnteringMethod: Predictor::Clear";

  AINFO<<"(DMCZP) LeaveMethod: Predictor::Clear";
 }

void Predictor::TrimTrajectories(
    const ADCTrajectoryContainer& adc_trajectory_container,
    Obstacle* obstacle) {
  AINFO<<"(DMCZP) EnteringMethod: Predictor::TrimTrajectories";

  for (auto& predicted_trajectory :
       *obstacle->mutable_latest_feature()->mutable_predicted_trajectory()) {
    TrimTrajectory(adc_trajectory_container, obstacle, &predicted_trajectory);
  }

  AINFO<<"(DMCZP) LeaveMethod: Predictor::TrimTrajectories";
 }

bool Predictor::TrimTrajectory(
    const ADCTrajectoryContainer& adc_trajectory_container, Obstacle* obstacle,
    Trajectory* trajectory) {
  AINFO<<"(DMCZP) EnteringMethod: Predictor::TrimTrajectory";

  if (!adc_trajectory_container.IsProtected()) {
    ADEBUG << "Not in protection mode.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::TrimTrajectory";
  return false;
  }
  if (obstacle == nullptr || obstacle->history_size() == 0) {
    AERROR << "Invalid obstacle.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::TrimTrajectory";
  return false;
  }
  int num_of_point = trajectory->trajectory_point_size();
  if (num_of_point == 0) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::TrimTrajectory";
  return false;
  }
  const Feature& feature = obstacle->latest_feature();
  double vehicle_length = feature.length();
  double vehicle_heading = feature.velocity_heading();
  double forward_length =
      std::fmax(vehicle_length / 2.0 - FLAGS_distance_beyond_junction, 0.0);

  double front_x = trajectory->trajectory_point(0).path_point().x() +
                   forward_length * std::cos(vehicle_heading);
  double front_y = trajectory->trajectory_point(0).path_point().y() +
                   forward_length * std::sin(vehicle_heading);
  PathPoint front_point;
  front_point.set_x(front_x);
  front_point.set_y(front_y);
  bool front_in_junction =
      adc_trajectory_container.IsPointInJunction(front_point);

  const PathPoint& start_point = trajectory->trajectory_point(0).path_point();
  bool start_in_junction =
      adc_trajectory_container.IsPointInJunction(start_point);

  if (front_in_junction || start_in_junction) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::TrimTrajectory";
  return false;
  }

  int index = 0;
  while (index < num_of_point) {
    const PathPoint& point = trajectory->trajectory_point(index).path_point();
    if (adc_trajectory_container.IsPointInJunction(point)) {
      break;
    }
    ++index;
  }

  // if no intersect
  if (index == num_of_point) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::TrimTrajectory";
  return false;
  }

  for (int i = index; i < num_of_point; ++i) {
    trajectory->mutable_trajectory_point()->RemoveLast();
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::TrimTrajectory";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: Predictor::TrimTrajectory";
 }

bool Predictor::SupposedToStop(const Feature& feature,
                               const double stop_distance,
                               double* acceleration) {
  AINFO<<"(DMCZP) EnteringMethod: Predictor::SupposedToStop";

  if (stop_distance < std::max(feature.length() * 0.5, 1.0)) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::SupposedToStop";
  return false;
  }
  if (stop_distance > FLAGS_distance_to_slow_down_at_stop_sign) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::SupposedToStop";
  return false;
  }
  double speed = feature.speed();
  *acceleration = -speed * speed / (2.0 * stop_distance);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::SupposedToStop";
  return *acceleration <= -FLAGS_double_precision &&
         *acceleration >= FLAGS_vehicle_min_linear_acc;

  AINFO<<"(DMCZP) LeaveMethod: Predictor::SupposedToStop";
 }

const ObstacleConf::PredictorType& Predictor::predictor_type() {
  AINFO<<"(DMCZP) EnteringMethod: Predictor::predictor_type";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: Predictor::predictor_type";
  return predictor_type_;

  AINFO<<"(DMCZP) LeaveMethod: Predictor::predictor_type";
 }

}  // namespace prediction
}  // namespace apollo
