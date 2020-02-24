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

#include "modules/localization/msf/local_integ/localization_integ.h"

#include "modules/common/time/time_util.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/msf/local_integ/lidar_msg_transfer.h"
#include "modules/localization/msf/local_integ/localization_integ_impl.h"

namespace apollo {
namespace localization {
namespace msf {

using common::Status;
using common::time::TimeUtil;

LocalizationInteg::LocalizationInteg()
    : localization_integ_impl_(new LocalizationIntegImpl()) {}
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::LocalizationInteg";

LocalizationInteg::~LocalizationInteg() {
  delete localization_integ_impl_;
  localization_integ_impl_ = nullptr;
}

Status LocalizationInteg::Init(const LocalizationIntegParam &params) {
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::Init";
  return localization_integ_impl_->Init(params);
}

void LocalizationInteg::PcdProcess(const drivers::PointCloud &message) {
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::PcdProcess";
  LidarFrame lidar_frame;
  LidarMsgTransfer transfer;
  transfer.Transfer(message, &lidar_frame);
  localization_integ_impl_->PcdProcess(lidar_frame);
  return;
}

void LocalizationInteg::RawImuProcessFlu(const drivers::gnss::Imu &imu_msg) {
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::RawImuProcessFlu";
  ImuData imu;
  TransferImuFlu(imu_msg, &imu);
  localization_integ_impl_->RawImuProcessRfu(imu);
  return;
}

void LocalizationInteg::RawImuProcessRfu(const drivers::gnss::Imu &imu_msg) {
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::RawImuProcessRfu";
  ImuData imu;
  TransferImuRfu(imu_msg, &imu);
  localization_integ_impl_->RawImuProcessRfu(imu);
  return;
}

void LocalizationInteg::RawObservationProcess(
    const drivers::gnss::EpochObservation &raw_obs_msg) {
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::RawObservationProcess";
  localization_integ_impl_->RawObservationProcess(raw_obs_msg);
  return;
}

void LocalizationInteg::RawEphemerisProcess(
    const drivers::gnss::GnssEphemeris &gnss_orbit_msg) {
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::RawEphemerisProcess";
  localization_integ_impl_->RawEphemerisProcess(gnss_orbit_msg);
  return;
}

void LocalizationInteg::GnssBestPoseProcess(
    const drivers::gnss::GnssBestPose &bestgnsspos_msg) {
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::GnssBestPoseProcess";
  localization_integ_impl_->GnssBestPoseProcess(bestgnsspos_msg);
  return;
}

void LocalizationInteg::GnssHeadingProcess(
    const drivers::gnss::Heading &gnssheading_msg) {
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::GnssHeadingProcess";
  localization_integ_impl_->GnssHeadingProcess(gnssheading_msg);
  return;
}

const LocalizationResult &LocalizationInteg::GetLastestLidarLocalization()
    const {
AINFO<<"(DMCZP) EnteringMethod: &LocalizationInteg::GetLastestLidarLocalization";
AINFO<<"(DMCZP) EnteringMethod: &LocalizationInteg::GetLastestIntegLocalization";
AINFO<<"(DMCZP) EnteringMethod: &LocalizationInteg::GetLastestGnssLocalization";
  return localization_integ_impl_->GetLastestLidarLocalization();
}

const LocalizationResult &LocalizationInteg::GetLastestIntegLocalization()
    const {
  return localization_integ_impl_->GetLastestIntegLocalization();
}

const LocalizationResult &LocalizationInteg::GetLastestGnssLocalization()
    const {
  return localization_integ_impl_->GetLastestGnssLocalization();
}

void LocalizationInteg::TransferImuRfu(const drivers::gnss::Imu &imu_msg,
                                       ImuData *imu_rfu) {
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::TransferImuRfu";
  CHECK_NOTNULL(imu_rfu);

  double measurement_time = TimeUtil::Gps2unix(imu_msg.measurement_time());
  imu_rfu->measurement_time = measurement_time;
  imu_rfu->fb[0] = imu_msg.linear_acceleration().x() * FLAGS_imu_rate;
  imu_rfu->fb[1] = imu_msg.linear_acceleration().y() * FLAGS_imu_rate;
  imu_rfu->fb[2] = imu_msg.linear_acceleration().z() * FLAGS_imu_rate;

  imu_rfu->wibb[0] = imu_msg.angular_velocity().x() * FLAGS_imu_rate;
  imu_rfu->wibb[1] = imu_msg.angular_velocity().y() * FLAGS_imu_rate;
  imu_rfu->wibb[2] = imu_msg.angular_velocity().z() * FLAGS_imu_rate;
  return;
}

void LocalizationInteg::TransferImuFlu(const drivers::gnss::Imu &imu_msg,
                                       ImuData *imu_flu) {
AINFO<<"(DMCZP) EnteringMethod: LocalizationInteg::TransferImuFlu";
  CHECK_NOTNULL(imu_flu);

  double measurement_time = TimeUtil::Gps2unix(imu_msg.measurement_time());
  imu_flu->measurement_time = measurement_time;
  imu_flu->fb[0] = -imu_msg.linear_acceleration().y() * FLAGS_imu_rate;
  imu_flu->fb[1] = imu_msg.linear_acceleration().x() * FLAGS_imu_rate;
  imu_flu->fb[2] = imu_msg.linear_acceleration().z() * FLAGS_imu_rate;

  imu_flu->wibb[0] = -imu_msg.angular_velocity().y() * FLAGS_imu_rate;
  imu_flu->wibb[1] = imu_msg.angular_velocity().x() * FLAGS_imu_rate;
  imu_flu->wibb[2] = imu_msg.angular_velocity().z() * FLAGS_imu_rate;
  return;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
