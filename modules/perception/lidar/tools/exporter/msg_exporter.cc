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
#include "modules/perception/lidar/tools/exporter/msg_exporter.h"

#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <memory>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/string_util.h"
#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

MsgExporter::MsgExporter(std::shared_ptr<apollo::cyber::Node> node,
                         const std::vector<std::string> channels,
                         const std::vector<std::string> child_frame_ids) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::MsgExporter";
  _cyber_node = node;
  _channels = channels;
  _child_frame_ids = child_frame_ids;
  _localization_method = "perception_localization_100hz";
  auto create_folder = [](const std::string& folder) {
    if (cyber::common::DirectoryExists(folder)) {
      cyber::common::DeleteFile(folder);
    }
    cyber::common::CreateDir(folder);
  };
  // bind channels
  for (std::size_t i = 0; i < _channels.size(); ++i) {
    const auto& channel = _channels[i];
    const auto& child_frame_id = _child_frame_ids[i];
    std::string folder = TransformChannelToFolder(channel);
    if (IsLidar(channel)) {
      _cyber_node->CreateReader<PcMsg>(
          channel,
          std::bind(&MsgExporter::PointCloudMessageHandler, this,
                    std::placeholders::_1, channel, child_frame_id, folder));
      create_folder(folder);
      std::cout << "Bind lidar channel: " << channel << std::endl;
    } else if (IsCamera(channel)) {
      _cyber_node->CreateReader<ImgMsg>(
          channel,
          std::bind(&MsgExporter::ImageMessageHandler, this,
                    std::placeholders::_1, channel, child_frame_id, folder));
      create_folder(folder);
      std::cout << "Bind camera channel: " << channel << std::endl;
    } else {
      std::cout << "Unknown channel type: " << channel << std::endl;
    }
  }

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::MsgExporter";
 }
void MsgExporter::ImageMessageHandler(
    const std::shared_ptr<const ImgMsg>& img_msg, const std::string& channel,
    const std::string& child_frame_id, const std::string& folder) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::ImageMessageHandler";
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::PointCloudMessageHandler";
  double timestamp = img_msg->measurement_time();
  // std::cout << "Receive image message from channel: " << channel <<
  //    " at time: " << GLOG_TIMESTAMP(timestamp) << std::endl;
  const unsigned char* data =
      reinterpret_cast<const unsigned char*>(img_msg->data().data());
  const unsigned char* range_data = nullptr;
  // const unsigned char* range_data = is_stereo_camera(channel) ?
  //    reinterpret_cast<const unsigned char*>(img_msg->disparity_data().data())
  //    : nullptr;
  std::size_t width = img_msg->width();
  std::size_t height = img_msg->height();
  Eigen::Matrix4d pose;
  if (QuerySensorToWorldPose(timestamp, child_frame_id, &pose)) {
    SaveImage(data, range_data, width, height, timestamp, folder);
    SavePose(pose, timestamp, folder);
  } else {
    std::cout << "Failed to query camera pose, child_frame_id "
              << child_frame_id << std::endl;
  }

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::ImageMessageHandler";
 }
void MsgExporter::PointCloudMessageHandler(
    const std::shared_ptr<const PcMsg>& cloud_msg, const std::string& channel,
    const std::string& child_frame_id, const std::string& folder) {
  double timestamp = cloud_msg->measurement_time();
  // std::cout << "Receive point cloud message from channel: " << channel <<
  //    " at time: " << GLOG_TIMESTAMP(timestamp) << std::endl;
  pcl::PointCloud<PCLPointXYZIT> cloud;
  if (cloud_msg->point_size() > 0) {
    cloud.resize(cloud_msg->point_size());
    for (int i = 0; i < cloud_msg->point_size(); ++i) {
      const apollo::drivers::PointXYZIT& pt = cloud_msg->point(i);
      if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
        continue;
      }
      if (std::fabs(pt.x()) > 1e3 || std::fabs(pt.y()) > 1e3 ||
          std::fabs(pt.z()) > 1e3) {
        continue;
      }
      cloud[i].x = pt.x();
      cloud[i].y = pt.y();
      cloud[i].z = pt.z();
      cloud[i].intensity = static_cast<unsigned char>(pt.intensity());
      cloud[i].timestamp = static_cast<double>(pt.timestamp()) * 1e-9;
    }
  }
  Eigen::Matrix4d pose;
  if (QuerySensorToWorldPose(timestamp, child_frame_id, &pose)) {
    SavePointCloud(cloud, timestamp, folder);
    SavePose(pose, timestamp, folder);
  } else {
    std::cout << "Failed to query lidar pose, child_frame_id " << child_frame_id
              << std::endl;
  }

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::PointCloudMessageHandler";
 }
bool MsgExporter::SavePointCloud(
    const pcl::PointCloud<PCLPointXYZIT>& point_cloud, double timestamp,
    const std::string& folder) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::SavePointCloud";
  static char path[500];
  snprintf(path, sizeof(path), "%s/%.6f.pcd", folder.c_str(), timestamp);
  pcl::PCDWriter writer;
  writer.writeBinaryCompressed(path, point_cloud);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::SavePointCloud";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::SavePointCloud";
 }
bool MsgExporter::SaveImage(const unsigned char* color_image,
                            const unsigned char* range_image, std::size_t width,
                            std::size_t height, double timestamp,
                            const std::string& folder) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::SaveImage";
  cv::Mat image(static_cast<int>(height), static_cast<int>(width), CV_8UC1,
                const_cast<unsigned char*>(color_image));
  static char path[500];
  snprintf(path, sizeof(path), "%s/%.6f.jpg", folder.c_str(), timestamp);
  cv::imwrite(path, image);
  if (range_image != nullptr) {
    snprintf(path, sizeof(path), "%s/%.6f.data", folder.c_str(), timestamp);
    std::ofstream fout(path, std::ofstream::binary);
    if (fout.is_open()) {
      fout.write(reinterpret_cast<const char*>(range_image), width * height);
      fout.close();
    }
    // cv::Mat range_image_mat(height, width, CV_8UC1, const_cast<unsigned
    // char*>(range_image));
    // snprintf(path, 500, "%s/%.6f_range_snapshot.jpg", folder.c_str(),
    // timestamp);
    // cv::imwrite(path, range_image_mat);
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::SaveImage";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::SaveImage";
 }
bool MsgExporter::QuerySensorToWorldPose(double timestamp,
                                         const std::string& child_frame_id,
                                         Eigen::Matrix4d* pose) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::QuerySensorToWorldPose";
  Eigen::Matrix4d sensor2novatel_extrinsics = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d novatel2world_pose;
  // query sensor to novatel extrinsics
  if (child_frame_id != "null" &&
      !QueryPose(timestamp, "novatel", child_frame_id,
                 &sensor2novatel_extrinsics)) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::QuerySensorToWorldPose";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::QuerySensorToWorldPose";
  return false;
  }
  // query novatel to world pose
  if (!QueryPose(timestamp, "world", _localization_method,
                 &novatel2world_pose)) {
    return false;
  }
  *pose = novatel2world_pose * sensor2novatel_extrinsics;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::QuerySensorToWorldPose";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::QuerySensorToWorldPose";
 }
bool MsgExporter::QueryPose(double timestamp, const std::string& frame_id,
                            const std::string& child_frame_id,
                            Eigen::Matrix4d* pose) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::QueryPose";
  cyber::Time query_time(timestamp);
  std::string err_string;
  if (!tf2_buffer_->canTransform(frame_id, child_frame_id, query_time, 0.05f,
                                 &err_string)) {
    AERROR << "Can not find transform. "  //<< GLOG_TIMESTAMP(timestamp)
           << " frame_id: " << frame_id << " child_frame_id: " << child_frame_id
           << " Error info: " << err_string;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::QueryPose";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::QueryPose";
  return false;
  }
  apollo::transform::TransformStamped stamped_transform;
  try {
    stamped_transform =
        tf2_buffer_->lookupTransform(frame_id, child_frame_id, query_time);
    Eigen::Translation3d translation(
        stamped_transform.transform().translation().x(),
        stamped_transform.transform().translation().y(),
        stamped_transform.transform().translation().z());
    Eigen::Quaterniond rotation(stamped_transform.transform().rotation().qw(),
                                stamped_transform.transform().rotation().qx(),
                                stamped_transform.transform().rotation().qy(),
                                stamped_transform.transform().rotation().qz());
    *pose = (translation * rotation).matrix();
  } catch (tf2::TransformException& ex) {
    AERROR << ex.what();
    return false;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::QueryPose";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::QueryPose";
 }
bool MsgExporter::SavePose(const Eigen::Matrix4d& pose, double timestamp,
                           const std::string& folder) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::SavePose";
  Eigen::Affine3d affine(pose);
  Eigen::Quaterniond quat = (Eigen::Quaterniond)affine.linear();
  static char path[500];
  snprintf(path, sizeof(path), "%s/%.6f.pose", folder.c_str(), timestamp);
  std::ofstream fout(path);
  if (fout.is_open()) {
    fout << 0 << " " << std::setprecision(14) << timestamp << " "
         << affine.translation().transpose() << " " << quat.x() << " "
         << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
    fout.close();
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::SavePose";
  return true;

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::SavePose";
 }
bool MsgExporter::IsStereoCamera(const std::string& channel) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::IsStereoCamera";
  std::vector<std::string> strs;
  apollo::common::util::Split(channel, '/', &strs);
  if (strs.size() > 2 && strs[2] == "smartereye") {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::IsStereoCamera";
  return true;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::IsStereoCamera";
  return false;

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::IsStereoCamera";
 }
bool MsgExporter::IsCamera(const std::string& channel) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::IsCamera";
  std::vector<std::string> strs;
  apollo::common::util::Split(channel, '/', &strs);
  if (strs.size() > 1 && strs[1] == "camera") {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::IsCamera";
  return true;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::IsCamera";
  return false;

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::IsCamera";
 }
bool MsgExporter::IsLidar(const std::string& channel) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::IsLidar";
  std::vector<std::string> strs;
  apollo::common::util::Split(channel, '/', &strs);
  if (strs.size() > 0 && strs.back() == "PointCloud2") {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::IsLidar";
  return true;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::IsLidar";
  return false;

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::IsLidar";
 }
std::string MsgExporter::TransformChannelToFolder(const std::string& channel) {
AINFO<<"(DMCZP) EnteringMethod: MsgExporter::TransformChannelToFolder";
  std::vector<std::string> strs;
  apollo::common::util::Split(channel, '/', &strs);
  std::string target = "";
  for (auto& str : strs) {
    target += str + "_";
  }
  target += "data";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: MsgExporter::TransformChannelToFolder";
  return target;

  AINFO<<"(DMCZP) LeaveMethod: MsgExporter::TransformChannelToFolder";
 }
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
