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
#include "modules/perception/fusion/lib/data_association/hm_data_association/probabilities.h"

namespace apollo {
namespace perception {
namespace fusion {

// @brief: scale input prob within input range
// @return bounded & scaled prob
// @NOTE: original method name is bound_scale_probability
double BoundedScalePositiveProbability(double p, double max_p, double min_p) {
AINFO<<"(DMCZP) EnteringMethod: BoundedScalePositiveProbability";
  p = std::max(p, min_p);
  p = (p - min_p) * (max_p - min_p) / (1 - min_p) + min_p;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: BoundedScalePositiveProbability";
  return p;

  AINFO<<"(DMCZP) LeaveMethod: BoundedScalePositiveProbability";
 }
// @brief: scale input prob
// @return scaled prob
// @NOTE: original method name is scale_positive_probability
double ScalePositiveProbability(double p, double max_p, double th_p) {
AINFO<<"(DMCZP) EnteringMethod: ScalePositiveProbability";
  if (p <= th_p) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: ScalePositiveProbability";
  return p;
  }
  p = (p - th_p) * (max_p - th_p) / (1 - th_p) + th_p;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: ScalePositiveProbability";
  return p;

  AINFO<<"(DMCZP) LeaveMethod: ScalePositiveProbability";
 }
// @brief: calculate the Welsh Loss
// @return Welsh Loss of input dist
// @NOTE: original method name is welsh_var_loss_fun
double WelshVarLossFun(double dist, double th, double scale) {
AINFO<<"(DMCZP) EnteringMethod: WelshVarLossFun";
  double p = 1e-6;
  if (dist < th) {
    p = 1 - 1e-6;
  } else {
    dist -= th;
    dist /= scale;
    p = std::exp(-dist * dist);
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: WelshVarLossFun";
  return p;

  AINFO<<"(DMCZP) LeaveMethod: WelshVarLossFun";
 }
// @brief: fuse two probabilities, fused prob is greater than 0.5, if
// the sum of input prob pair is greater than 1, otherwise, fused prob
// is less than 0.5
// @return fused prob of input prob pair
// @NOTE: original method name is fused_tow_probabilities
double FuseTwoProbabilities(double prob1, double prob2) {
AINFO<<"(DMCZP) EnteringMethod: FuseTwoProbabilities";
  double prob = (prob1 * prob2) / (2 * prob1 * prob2 + 1 - prob1 - prob2);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: FuseTwoProbabilities";
  return prob;

  AINFO<<"(DMCZP) LeaveMethod: FuseTwoProbabilities";
 }
// @brief: fuse multiple probabilities
// @return fsued probability of input multiple probabilities
// @NOTE: original method name is fused_multiple_probabilities
double FuseMultipleProbabilities(const std::vector<double>& probs) {
AINFO<<"(DMCZP) EnteringMethod: FuseMultipleProbabilities";
  std::vector<double> log_odd_probs = probs;
  auto prob_to_log_odd = [](double p) {
    p = std::max(std::min(p, 1 - 1e-6), 1e-6);
    
  AINFO<<"(DMCZP) (return) LeaveMethod: FuseMultipleProbabilities";
  return std::log(p / (1 - p));
  };
  auto log_odd_to_prob = [](double log_odd_p) {
    double tmp = std::exp(log_odd_p);
    
  AINFO<<"(DMCZP) (return) LeaveMethod: FuseMultipleProbabilities";
  return tmp / (tmp + 1);
  };
  for (auto& log_odd_prob : log_odd_probs) {
    log_odd_prob = prob_to_log_odd(log_odd_prob);
  }
  double log_odd_probs_sum =
      std::accumulate(log_odd_probs.begin(), log_odd_probs.end(), 0.0);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: FuseMultipleProbabilities";
  return log_odd_to_prob(log_odd_probs_sum);

  AINFO<<"(DMCZP) LeaveMethod: FuseMultipleProbabilities";
 }

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
