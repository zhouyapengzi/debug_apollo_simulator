/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera/app/lane_camera_perception.h"

#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/object.h"
#include "modules/perception/camera/app/debug_info.h"
#include "modules/perception/camera/common/global_config.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/inference/utils/cuda_util.h"
#include "modules/perception/lib/utils/perf.h"
#include <thread>

namespace apollo {
namespace perception {
namespace camera {

using apollo::cyber::common::GetAbsolutePath;
using cyber::common::EnsureDirectory;

bool LaneCameraPerception::Init(const CameraPerceptionInitOptions &options) {
AINFO<<"(DMCZP) EnteringMethod: LaneCameraPerception::Init";
  std::string work_root = "";
  if (options.use_cyber_work_root) {
    work_root = GetCyberWorkRoot();
  }
  std::string config_file =
      GetAbsolutePath(options.root_dir, options.conf_file);
  config_file = GetAbsolutePath(work_root, config_file);
  CHECK(cyber::common::GetProtoFromFile(config_file, &perception_param_))
      << "Read config failed: " << config_file;
  CHECK(inference::CudaUtil::set_device_id(perception_param_.gpu_id()));

  lane_calibration_working_sensor_name_ =
      options.lane_calibration_working_sensor_name;

  // Init lane
  base::BaseCameraModelPtr model;
  InitLane(work_root, model, perception_param_);

  // Init calibration service
  InitCalibrationService(work_root, model, perception_param_);

  
  AINFO<<"(DMCZP) (return) LeaveMethod: LaneCameraPerception::Init";
  return true;
}

void LaneCameraPerception::InitLane(
    const std::string &work_root, base::BaseCameraModelPtr &model,
    const app::PerceptionParam &perception_param) {
AINFO<<"(DMCZP) EnteringMethod: LaneCameraPerception::InitLane";
AINFO<<"(DMCZP) EnteringMethod: LaneCameraPerception::InitCalibrationService";
  // Init lane
  CHECK(perception_param.has_lane_param()) << "Failed to include lane_param";
  {
    // Initialize lane detector
    auto lane_param = perception_param.lane_param();
    CHECK(lane_param.has_lane_detector_param())
        << "Failed to include lane_detector_param";
    LaneDetectorInitOptions lane_detector_init_options;
    auto lane_detector_param = lane_param.lane_detector_param();
    auto lane_detector_plugin_param = lane_detector_param.plugin_param();
    lane_detector_init_options.conf_file =
        lane_detector_plugin_param.config_file();

    AINFO << "(pengzi) initial lane detector " <<". thread:"<< std::this_thread::get_id();
    
    lane_detector_init_options.root_dir =
        GetAbsolutePath(work_root, lane_detector_plugin_param.root_dir());
    lane_detector_init_options.gpu_id = perception_param_.gpu_id();
    model = common::SensorManager::Instance()->GetUndistortCameraModel(
        lane_detector_param.camera_name());
    auto pinhole = static_cast<base::PinholeCameraModel *>(model.get());
    name_intrinsic_map_.insert(std::pair<std::string, Eigen::Matrix3f>(
        lane_detector_param.camera_name(), pinhole->get_intrinsic_params()));
    lane_detector_init_options.base_camera_model = model;
    AINFO << "lane detector name: " << lane_detector_plugin_param.name();
    lane_detector_.reset(BaseLaneDetectorRegisterer::GetInstanceByName(
        lane_detector_plugin_param.name()));
    CHECK(lane_detector_ != nullptr);
    CHECK(lane_detector_->Init(lane_detector_init_options))
        << "Failed to init " << lane_detector_plugin_param.name();
    AINFO << "Detector: " << lane_detector_->Name();
    AINFO << "(pengzi)initialize Lane Detector: " << lane_detector_->Name()<<".thread:"<< std::this_thread::get_id();



    //  initialize lane postprocessor
    auto lane_postprocessor_param =
        perception_param_.lane_param().lane_postprocessor_param();
    LanePostprocessorInitOptions postprocessor_init_options;
    postprocessor_init_options.detect_config_root =
        GetAbsolutePath(work_root, lane_detector_plugin_param.root_dir());
    postprocessor_init_options.detect_config_name =
        lane_detector_plugin_param.config_file();
    postprocessor_init_options.root_dir =
        GetAbsolutePath(work_root, lane_postprocessor_param.root_dir());
    postprocessor_init_options.conf_file =
        lane_postprocessor_param.config_file();
    lane_postprocessor_.reset(
        BaseLanePostprocessorRegisterer::GetInstanceByName(
            lane_postprocessor_param.name()));
    CHECK(lane_postprocessor_ != nullptr);
    CHECK(lane_postprocessor_->Init(postprocessor_init_options))
        << "Failed to init " << lane_postprocessor_param.name();
    AINFO << "Lane postprocessor: " << lane_postprocessor_->Name();
    AINFO << "(pengzi)initialize Lane postprocessor: " << lane_postprocessor_->Name()<<std::this_thread::get_id();;


    // Init output file folder
    if (perception_param_.has_debug_param() &&
        perception_param_.debug_param().has_lane_out_dir()) {
      write_out_lane_file_ = true;
      out_lane_dir_ = perception_param_.debug_param().lane_out_dir();
      EnsureDirectory(out_lane_dir_);
    }

    if (perception_param_.has_debug_param() &&
        perception_param_.debug_param().has_calibration_out_dir()) {
      write_out_calib_file_ = true;
      out_calib_dir_ = perception_param_.debug_param().calibration_out_dir();
      EnsureDirectory(out_calib_dir_);
    }
  }

  AINFO<<"(DMCZP) LeaveMethod: LaneCameraPerception::InitLane";
 }

void LaneCameraPerception::InitCalibrationService(
    const std::string &work_root, const base::BaseCameraModelPtr model,
    const app::PerceptionParam &perception_param) {
  // Init calibration service
  CHECK(perception_param.has_calibration_service_param())
      << "Failed to include calibration_service_param";
  {
    auto calibration_service_param =
        perception_param.calibration_service_param();
    CalibrationServiceInitOptions calibration_service_init_options;
    calibration_service_init_options.calibrator_working_sensor_name =
        lane_calibration_working_sensor_name_;
    calibration_service_init_options.name_intrinsic_map = name_intrinsic_map_;
    calibration_service_init_options.calibrator_method =
        calibration_service_param.calibrator_method();
    calibration_service_init_options.image_height =
        static_cast<int>(model->get_height());
    calibration_service_init_options.image_width =
        static_cast<int>(model->get_width());
    calibration_service_.reset(
        BaseCalibrationServiceRegisterer::GetInstanceByName(
            calibration_service_param.plugin_param().name()));
    CHECK(calibration_service_ != nullptr);
    CHECK(calibration_service_->Init(calibration_service_init_options))
        << "Failed to init " << calibration_service_param.plugin_param().name();
    AINFO << "Calibration service: " << calibration_service_->Name();

      AINFO <<"(pengzi) initialize Lane Calibration service: " << calibration_service_->Name() 
    <<".thread:"<< std::this_thread::get_id();

  }

  AINFO<<"(DMCZP) LeaveMethod: LaneCameraPerception::InitCalibrationService";
 }

void LaneCameraPerception::SetCameraHeightAndPitch(
    const std::map<std::string, float> name_camera_ground_height_map,
    const std::map<std::string, float> name_camera_pitch_angle_diff_map,
    const float &pitch_angle_calibrator_working_sensor) {
AINFO<<"(DMCZP) EnteringMethod: LaneCameraPerception::SetCameraHeightAndPitch";
  CHECK(calibration_service_ != nullptr);
  calibration_service_->SetCameraHeightAndPitch(
      name_camera_ground_height_map, name_camera_pitch_angle_diff_map,
      pitch_angle_calibrator_working_sensor);
AINFO <<"(pengzi))set camera height and pitch to detect lane. thread:"<< std::this_thread::get_id();
  

  AINFO<<"(DMCZP) LeaveMethod: LaneCameraPerception::SetCameraHeightAndPitch";
 }

void LaneCameraPerception::SetIm2CarHomography(
    Eigen::Matrix3d homography_im2car) {
AINFO<<"(DMCZP) EnteringMethod: LaneCameraPerception::SetIm2CarHomography";
  CHECK(calibration_service_ != nullptr);
  lane_postprocessor_->SetIm2CarHomography(homography_im2car);

  AINFO<<"(DMCZP) LeaveMethod: LaneCameraPerception::SetIm2CarHomography";
 }

bool LaneCameraPerception::GetCalibrationService(
    BaseCalibrationService **calibration_service) {
AINFO<<"(DMCZP) EnteringMethod: LaneCameraPerception::GetCalibrationService";
  *calibration_service = calibration_service_.get();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: LaneCameraPerception::GetCalibrationService";
  return true;
}

bool LaneCameraPerception::Perception(const CameraPerceptionOptions &options,
                                      CameraFrame *frame) {
AINFO<<"(DMCZP) EnteringMethod: LaneCameraPerception::Perception";

AINFO << "(pengzi) lane camera perception begin.thread:"<< std::this_thread::get_id();
  
  PERCEPTION_PERF_FUNCTION();
  inference::CudaUtil::set_device_id(perception_param_.gpu_id());
  PERCEPTION_PERF_BLOCK_START();

  CHECK(frame->calibration_service != nullptr);

  // Lane detector and postprocessor: work on front_6mm only
  if (lane_calibration_working_sensor_name_ ==
      frame->data_provider->sensor_name()) {
    
    AINFO << "(pengzi) lane detector and postprocess on front_6mm camera. thread:"<< std::this_thread::get_id();

    frame->camera_k_matrix =
        name_intrinsic_map_.at(frame->data_provider->sensor_name());
    LaneDetectorOptions lane_detetor_options;
    LanePostprocessorOptions lane_postprocessor_options;
    if (!lane_detector_->Detect(lane_detetor_options, frame)) {
      AERROR << "Failed to detect lane.";
      
  AINFO<<"(DMCZP) (return) LeaveMethod: LaneCameraPerception::Perception";
  return false;
    }
    PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
        frame->data_provider->sensor_name(), "LaneDetector");

    if (!lane_postprocessor_->Process2D(lane_postprocessor_options, frame)) {
      AERROR << "Failed to postprocess lane 2D.";
      
  AINFO<<"(DMCZP) (return) LeaveMethod: LaneCameraPerception::Perception";
  return false;
    }
    AINFO <<"(pengzi)lane post processor 2d.thread:"<<std::this_thread::get_id();
    PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
        frame->data_provider->sensor_name(), "LanePostprocessor2D");

    // Calibration service
    frame->calibration_service->Update(frame);
    PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
        frame->data_provider->sensor_name(), "CalibrationService");

    if (!lane_postprocessor_->Process3D(lane_postprocessor_options, frame)) {
      AERROR << "Failed to postprocess lane 3D.";
      
  AINFO<<"(DMCZP) (return) LeaveMethod: LaneCameraPerception::Perception";
  return false;
    }
    PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
        frame->data_provider->sensor_name(), "LanePostprocessor3D");
    
    AINFO <<"(pengzi)lane post processor 3d.thread:"<<std::this_thread::get_id();
   

    if (write_out_lane_file_) {
      std::string lane_file_path =
          out_lane_dir_ + "/" + std::to_string(frame->frame_id) + ".txt";
      WriteLanelines(write_out_lane_file_, lane_file_path, frame->lane_objects);
       
    AINFO <<"(pengzi)write lane camera perception result to: "<<lane_file_path<<" thread:"<<std::this_thread::get_id();
  
    }
  } else {
    AINFO << "Skip lane detection & calibration due to sensor mismatch.";
    AINFO << "Will use service sync from obstacle camera instead.";
    // fill the frame using previous estimates
    frame->calibration_service->Update(frame);
    PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
        frame->data_provider->sensor_name(), "CalibrationService");
    AINFO <<"(pengzi)use service sync from obstacle camera instead.thread: "<<std::this_thread::get_id();
   
  }

  if (write_out_calib_file_) {
    std::string calib_file_path =
        out_calib_dir_ + "/" + std::to_string(frame->frame_id) + ".txt";
    WriteCalibrationOutput(write_out_calib_file_, calib_file_path, frame);
  }
  AINFO <<"(pengzi))Finish lane camera perception.thread:"<< std::this_thread::get_id();
  
  AINFO<<"(DMCZP) (return) LeaveMethod: LaneCameraPerception::Perception";
  return true;
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
