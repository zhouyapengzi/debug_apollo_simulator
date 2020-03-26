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

#include "modules/prediction/common/feature_output.h"

#include <vector>

#include "cyber/common/file.h"
#include "modules/common/util/string_util.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

using apollo::common::TrajectoryPoint;
using apollo::common::util::StrCat;

Features FeatureOutput::features_;
ListDataForLearning FeatureOutput::list_data_for_learning_;
ListPredictionResult FeatureOutput::list_prediction_result_;
ListFrameEnv FeatureOutput::list_frame_env_;
ListDataForTuning FeatureOutput::list_data_for_tuning_;
std::size_t FeatureOutput::idx_feature_ = 0;
std::size_t FeatureOutput::idx_learning_ = 0;
std::size_t FeatureOutput::idx_prediction_result_ = 0;
std::size_t FeatureOutput::idx_frame_env_ = 0;
std::size_t FeatureOutput::idx_tuning_ = 0;

void FeatureOutput::Close() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::Close";

  ADEBUG << "Close feature output";
  switch (FLAGS_prediction_offline_mode) {
    case 1: {
      WriteFeatureProto();
      break;
    }
    case 2: {
      WriteDataForLearning();
      break;
    }
    case 3: {
      WritePredictionResult();
      break;
    }
    case 4: {
      WriteFrameEnv();
      break;
    }
    case 5: {
      WriteDataForTuning();
      break;
    }
    default: {
      // No data dump
      break;
    }
  }
  Clear();

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::Close";
 }

void FeatureOutput::Clear() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::Clear";

  idx_feature_ = 0;
  idx_learning_ = 0;
  features_.Clear();
  list_data_for_learning_.Clear();

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::Clear";
 }

bool FeatureOutput::Ready() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::Ready";

  Clear();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: FeatureOutput::Ready";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::Ready";
 }

void FeatureOutput::InsertFeatureProto(const Feature& feature) {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::InsertFeatureProto";

  features_.add_feature()->CopyFrom(feature);

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::InsertFeatureProto";
 }

void FeatureOutput::InsertDataForLearning(
    const Feature& feature, const std::vector<double>& feature_values,
    const std::string& category, const LaneSequence* lane_sequence_ptr) {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::InsertDataForLearning";

  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::InsertDataForLearning";

  const std::vector<std::string> dummy_string_feature_values;
  InsertDataForLearning(feature, feature_values, dummy_string_feature_values,
                        category, lane_sequence_ptr);

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::InsertDataForLearning";
 
  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::InsertDataForLearning";
 }

void FeatureOutput::InsertDataForLearning(
    const Feature& feature, const std::vector<double>& feature_values,
    const std::vector<std::string>& string_feature_values,
    const std::string& category, const LaneSequence* lane_sequence_ptr) {
  DataForLearning* data_for_learning =
      list_data_for_learning_.add_data_for_learning();
  data_for_learning->set_id(feature.id());
  data_for_learning->set_timestamp(feature.timestamp());
  *(data_for_learning->mutable_features_for_learning()) = {
      feature_values.begin(), feature_values.end()};
  *(data_for_learning->mutable_string_features_for_learning()) = {
      string_feature_values.begin(), string_feature_values.end()};
  data_for_learning->set_category(category);
  ADEBUG << "Insert [" << category
         << "] data for learning with size = " << feature_values.size();
  if (lane_sequence_ptr != nullptr) {
    data_for_learning->set_lane_sequence_id(
        lane_sequence_ptr->lane_sequence_id());
  }
}

void FeatureOutput::InsertPredictionResult(
    const int obstacle_id, const PredictionObstacle& prediction_obstacle,
    const ObstacleConf& obstacle_conf) {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::InsertPredictionResult";

  PredictionResult* prediction_result =
      list_prediction_result_.add_prediction_result();
  prediction_result->set_id(obstacle_id);
  prediction_result->set_timestamp(prediction_obstacle.timestamp());
  for (int i = 0; i < prediction_obstacle.trajectory_size(); ++i) {
    prediction_result->add_trajectory()->CopyFrom(
        prediction_obstacle.trajectory(i));
    prediction_result->mutable_obstacle_conf()->CopyFrom(obstacle_conf);
  }

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::InsertPredictionResult";
 }

void FeatureOutput::InsertFrameEnv(const FrameEnv& frame_env) {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::InsertFrameEnv";

  list_frame_env_.add_frame_env()->CopyFrom(frame_env);

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::InsertFrameEnv";
 }

void FeatureOutput::InsertDataForTuning(
    const Feature& feature, const std::vector<double>& feature_values,
    const std::string& category, const LaneSequence& lane_sequence,
    const std::vector<TrajectoryPoint>& adc_trajectory) {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::InsertDataForTuning";

  DataForTuning* data_for_tuning = list_data_for_tuning_.add_data_for_tuning();
  data_for_tuning->set_id(feature.id());
  data_for_tuning->set_timestamp(feature.timestamp());
  *data_for_tuning->mutable_values_for_tuning() = {feature_values.begin(),
                                                   feature_values.end()};
  data_for_tuning->set_category(category);
  ADEBUG << "Insert [" << category
         << "] data for tuning with size = " << feature_values.size();
  data_for_tuning->set_lane_sequence_id(lane_sequence.lane_sequence_id());
  for (const auto& adc_traj_point : adc_trajectory) {
    data_for_tuning->add_adc_trajectory_point()->CopyFrom(adc_traj_point);
  }

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::InsertDataForTuning";
 }

void FeatureOutput::WriteFeatureProto() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::WriteFeatureProto";

  if (features_.feature_size() <= 0) {
    ADEBUG << "Skip writing empty feature.";
  } else {
    const std::string file_name = StrCat(FLAGS_prediction_data_dir, "/feature.",
                                         std::to_string(idx_feature_), ".bin");
    cyber::common::SetProtoToBinaryFile(features_, file_name);
    features_.Clear();
    ++idx_feature_;
  }

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::WriteFeatureProto";
 }

void FeatureOutput::WriteDataForLearning() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::WriteDataForLearning";

  if (list_data_for_learning_.data_for_learning().empty()) {
    ADEBUG << "Skip writing empty data_for_learning.";
  } else {
    const std::string file_name =
        StrCat(FLAGS_prediction_data_dir, "/datalearn.",
               std::to_string(idx_learning_), ".bin");
    cyber::common::SetProtoToBinaryFile(list_data_for_learning_, file_name);
    list_data_for_learning_.Clear();
    ++idx_learning_;
  }

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::WriteDataForLearning";
 }

void FeatureOutput::WritePredictionResult() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::WritePredictionResult";

  if (list_prediction_result_.prediction_result().empty()) {
    ADEBUG << "Skip writing empty prediction_result.";
  } else {
    const std::string file_name =
        StrCat(FLAGS_prediction_data_dir, "/prediction_result.",
               std::to_string(idx_prediction_result_), ".bin");
    cyber::common::SetProtoToBinaryFile(list_prediction_result_, file_name);
    list_prediction_result_.Clear();
    ++idx_prediction_result_;
  }

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::WritePredictionResult";
 }

void FeatureOutput::WriteFrameEnv() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::WriteFrameEnv";

  if (list_frame_env_.frame_env().empty()) {
    ADEBUG << "Skip writing empty prediction_result.";
  } else {
    const std::string file_name =
        StrCat(FLAGS_prediction_data_dir, "/frame_env.",
               std::to_string(idx_frame_env_), ".bin");
    cyber::common::SetProtoToBinaryFile(list_frame_env_, file_name);
    list_frame_env_.Clear();
    ++idx_frame_env_;
  }

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::WriteFrameEnv";
 }

void FeatureOutput::WriteDataForTuning() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::WriteDataForTuning";

  if (list_data_for_tuning_.data_for_tuning().empty()) {
    ADEBUG << "Skip writing empty data_for_tuning.";
    
  AINFO<<"(DMCZP) (return) LeaveMethod: FeatureOutput::WriteDataForTuning";
  return;
  }
  const std::string file_name =
      StrCat(FLAGS_prediction_data_dir, "/datatuning.",
             std::to_string(idx_tuning_), ".bin");
  cyber::common::SetProtoToBinaryFile(list_data_for_tuning_, file_name);
  list_data_for_tuning_.Clear();
  ++idx_tuning_;

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::WriteDataForTuning";
 }

int FeatureOutput::Size() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::Size";
 
  AINFO<<"(DMCZP) (return) Leave
  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::Size";
 Method: FeatureOutput::Size";
  return features_.feature_size(); }

int FeatureOutput::SizeOfDataForLearning() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::SizeOfDataForLearning";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: FeatureOutput::SizeOfDataForLearning";
  return list_data_for_learning_.data_for_learning_size();

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::SizeOfDataForLearning";
 }

int FeatureOutput::SizeOfPredictionResult() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::SizeOfPredictionResult";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: FeatureOutput::SizeOfPredictionResult";
  return list_prediction_result_.prediction_result_size();

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::SizeOfPredictionResult";
 }

int FeatureOutput::SizeOfFrameEnv() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::SizeOfFrameEnv";
 
  AINFO<<"(DMCZP) (return) LeaveMethod: 
  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::SizeOfFrameEnv";
 FeatureOutput::SizeOfFrameEnv";
  return list_frame_env_.frame_env_size(); }

int FeatureOutput::SizeOfDataForTuning() {
  AINFO<<"(DMCZP) EnteringMethod: FeatureOutput::SizeOfDataForTuning";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: FeatureOutput::SizeOfDataForTuning";
  return list_data_for_tuning_.data_for_tuning_size();

  AINFO<<"(DMCZP) LeaveMethod: FeatureOutput::SizeOfDataForTuning";
 }

}  // namespace prediction
}  // namespace apollo
