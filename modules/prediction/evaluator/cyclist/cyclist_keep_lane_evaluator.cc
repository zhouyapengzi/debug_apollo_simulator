#include "cyber/common/log.h"
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

#include "modules/prediction/evaluator/cyclist/cyclist_keep_lane_evaluator.h"
#include <thread>
namespace apollo {
namespace prediction {

CyclistKeepLaneEvaluator::CyclistKeepLaneEvaluator() {
  AINFO<<"(DMCZP) EnteringMethod: CyclistKeepLaneEvaluator::CyclistKeepLaneEvaluator";

  evaluator_type_ = ObstacleConf::CYCLIST_KEEP_LANE_EVALUATOR;

  AINFO<<"(DMCZP) LeaveMethod: CyclistKeepLaneEvaluator::CyclistKeepLaneEvaluator";
 }

bool CyclistKeepLaneEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  AINFO<<"(DMCZP) EnteringMethod: CyclistKeepLaneEvaluator::Evaluate";

  AINFO<<"(pengzi) begin prediction-evaluator of CyclistKeepLaneEvaluator. thread: "<<std::this_thread::get_id();
  CHECK_NOTNULL(obstacle_ptr);

  obstacle_ptr->SetEvaluatorType(evaluator_type_);

  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: CyclistKeepLaneEvaluator::Evaluate";
  return false;
  }

  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);
  if (!latest_feature_ptr->has_lane() ||
      !latest_feature_ptr->lane().has_lane_graph() ||
      !latest_feature_ptr->lane().has_lane_feature()) {
    ADEBUG << "Obstacle [" << id << "] has no lane graph.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: CyclistKeepLaneEvaluator::Evaluate";
  return false;
  }

  LaneGraph* lane_graph_ptr =
      latest_feature_ptr->mutable_lane()->mutable_lane_graph();
  CHECK_NOTNULL(lane_graph_ptr);
  if (lane_graph_ptr->lane_sequence_size() == 0) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: CyclistKeepLaneEvaluator::Evaluate";
  return false;
  }

  std::string curr_lane_id =
      latest_feature_ptr->lane().lane_feature().lane_id();

  for (auto& lane_sequence : *lane_graph_ptr->mutable_lane_sequence()) {
    const double probability = ComputeProbability(curr_lane_id, lane_sequence);
    lane_sequence.set_probability(probability);
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: CyclistKeepLaneEvaluator::Evaluate";
  return true;
 }

double CyclistKeepLaneEvaluator::ComputeProbability(
    const std::string& curr_lane_id, const LaneSequence& lane_sequence) {
  AINFO<<"(DMCZP) EnteringMethod: CyclistKeepLaneEvaluator::ComputeProbability";

  AINFO<<"(pengzi) computer probaility for CyclistKeepLaneEvaluator. thread:"<<std::this_thread::get_id();
  if (lane_sequence.lane_segment_size() == 0) {
    AWARN << "Empty lane sequence.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: CyclistKeepLaneEvaluator::ComputeProbability";
  return 0.0;
  }
  std::string lane_seq_first_id = lane_sequence.lane_segment(0).lane_id();
  if (curr_lane_id == lane_seq_first_id) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: CyclistKeepLaneEvaluator::ComputeProbability";
  return 1.0;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: CyclistKeepLaneEvaluator::ComputeProbability";
  return 0.0;
 }

}  // namespace prediction
}  // namespace apollo
