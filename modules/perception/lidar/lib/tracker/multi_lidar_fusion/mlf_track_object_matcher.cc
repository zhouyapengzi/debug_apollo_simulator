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

#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_track_object_matcher.h"

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

bool MlfTrackObjectMatcher::Init(
    const MlfTrackObjectMatcherInitOptions &options) {
AINFO<<"(DMCZP) EnteringMethod: MlfTrackObjectMatcher::Init";
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig *model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = cyber::common::GetAbsolutePath(work_root, root_path);
  config_file = cyber::common::GetAbsolutePath(config_file,
                                               "mlf_track_object_matcher.conf");
  MlfTrackObjectMatcherConfig config;
  CHECK(cyber::common::GetProtoFromFile(config_file, &config));

  foreground_matcher_.reset(
      BaseBipartiteGraphMatcherRegisterer::GetInstanceByName(
          config.foreground_mathcer_method()));
  CHECK(foreground_matcher_ != nullptr);
  AINFO << "MlfTrackObjectMatcher, fg: " << foreground_matcher_->Name();
  background_matcher_.reset(
      BaseBipartiteGraphMatcherRegisterer::GetInstanceByName(
          config.background_matcher_method()));
  CHECK(background_matcher_ != nullptr);
  AINFO << "MlfTrackObjectMatcher, bg: " << background_matcher_->Name();
  foreground_matcher_->cost_matrix()->Reserve(1000, 1000);
  background_matcher_->cost_matrix()->Reserve(1000, 1000);

  track_object_distance_.reset(new MlfTrackObjectDistance);
  MlfTrackObjectDistanceInitOptions distance_init_options;
  CHECK(track_object_distance_->Init(distance_init_options));

  bound_value_ = config.bound_value();
  max_match_distance_ = config.max_match_distance();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MlfTrackObjectMatcher::Init";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: MlfTrackObjectMatcher::Init";
 }

void MlfTrackObjectMatcher::Match(
    const MlfTrackObjectMatcherOptions &options,
    const std::vector<TrackedObjectPtr> &objects,
    const std::vector<MlfTrackDataPtr> &tracks,
    std::vector<std::pair<size_t, size_t> > *assignments,
    std::vector<size_t> *unassigned_tracks,
    std::vector<size_t> *unassigned_objects) {
AINFO<<"(DMCZP) EnteringMethod: MlfTrackObjectMatcher::Match";
  assignments->clear();
  unassigned_objects->clear();
  unassigned_tracks->clear();
  if (objects.empty() || tracks.empty()) {
    unassigned_objects->resize(objects.size());
    unassigned_tracks->resize(tracks.size());
    std::iota(unassigned_objects->begin(), unassigned_objects->end(), 0);
    std::iota(unassigned_tracks->begin(), unassigned_tracks->end(), 0);
    
  AINFO<<"(DMCZP) (return) LeaveMethod: MlfTrackObjectMatcher::Match";
  return;
  }

  BipartiteGraphMatcherOptions matcher_options;
  matcher_options.cost_thresh = max_match_distance_;
  matcher_options.bound_value = bound_value_;

  BaseBipartiteGraphMatcher *matcher = objects[0]->is_background
                                           ? background_matcher_.get()
                                           : foreground_matcher_.get();

  common::SecureMat<float> *association_mat = matcher->cost_matrix();

  association_mat->Resize(tracks.size(), objects.size());
  ComputeAssociateMatrix(tracks, objects, association_mat);
  matcher->Match(matcher_options, assignments, unassigned_tracks,
                 unassigned_objects);
  for (size_t i = 0; i < assignments->size(); ++i) {
    objects[assignments->at(i).second]->association_score =
        (*association_mat)(assignments->at(i).first,
                           assignments->at(i).second) /
        max_match_distance_;
  }

  AINFO<<"(DMCZP) LeaveMethod: MlfTrackObjectMatcher::Match";
 }

void MlfTrackObjectMatcher::ComputeAssociateMatrix(
    const std::vector<MlfTrackDataPtr> &tracks,
    const std::vector<TrackedObjectPtr> &new_objects,
    common::SecureMat<float> *association_mat) {
AINFO<<"(DMCZP) EnteringMethod: MlfTrackObjectMatcher::ComputeAssociateMatrix";
  for (size_t i = 0; i < tracks.size(); ++i) {
    for (size_t j = 0; j < new_objects.size(); ++j) {
      (*association_mat)(i, j) =
          track_object_distance_->ComputeDistance(new_objects[j], tracks[i]);
    }
  }

  AINFO<<"(DMCZP) LeaveMethod: MlfTrackObjectMatcher::ComputeAssociateMatrix";
 }

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
