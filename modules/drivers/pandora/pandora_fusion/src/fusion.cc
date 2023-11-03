/******************************************************************************
 * Copyright 2017 The JmcAuto Authors. All Rights Reserved.
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

#include <limits>
#include <string>

#include "pandora_fusion/fusion.h"
#include "ros/this_node.h"

namespace jmc_auto {
namespace drivers {
namespace pandora {

Fusion::Fusion(ros::NodeHandle node, ros::NodeHandle private_nh)
    : tf2_transform_listener_(tf2_buffer_, node) {
  private_nh.param("fused_lidar_frame_id", fused_lidar_frame_id_,
                   std::string("velodyne64"));
  private_nh.param("left_lidar_frame_id", left_lidar_frame_id_,
                   std::string("hesai40_left"));
  private_nh.param("right_lidar_frame_id", right_lidar_frame_id_,
                   std::string("hesai40_right"));
  private_nh.param("back_lidar_frame_id", back_lidar_frame_id_,
                   std::string("hesai40_back"));
  private_nh.param(
      "topic_fused_pointcloud", topic_fused_pointcloud_,
      std::string("/jmc_auto/sensor/pandora/velodyne64/PointCloud2"));
  private_nh.param("topic_left_pointcloud", topic_left_pointcloud_,
                   std::string("/jmc_auto/sensor/hesai40/left/PointCloud2"));
  private_nh.param("topic_right_pointcloud", topic_right_pointcloud_,
                   std::string("/jmc_auto/sensor/hesai40/right/PointCloud2"));
  private_nh.param("topic_back_pointcloud", topic_back_pointcloud_,
                   std::string("/jmc_auto/sensor/hesai40/back/PointCloud2"));
  private_nh.param("queue_size", queue_size_, 10);
  private_nh.param("tf_query_timeout", tf_timeout_, 0.1f);

  // advertise output point cloud (before subscribing to input data)
  fusion_pub_ = node.advertise<sensor_msgs::PointCloud2>(
      topic_fused_pointcloud_, queue_size_);

  left_lidar_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>
                                                        (node,topic_left_pointcloud_, 1);
  right_lidar_sub_= new message_filters::Subscriber<sensor_msgs::PointCloud2>
                                                        (node,topic_right_pointcloud_, 1);
  back_lidar_sub_= new message_filters::Subscriber<sensor_msgs::PointCloud2>
                                                        (node,topic_back_pointcloud_, 1);

  lidar_msg_sync_ = new message_filters::Synchronizer<MySyncPolicy>(
                        MySyncPolicy(10), *left_lidar_sub_, *right_lidar_sub_, *back_lidar_sub_);
  
  lidar_msg_sync_->registerCallback(boost::bind(&Fusion::LidarFusionCallback,this, _1, _2, _3));
  
}

void Fusion::LidarFusionCallback(sensor_msgs::PointCloud2ConstPtr left_msg,
                                 sensor_msgs::PointCloud2ConstPtr right_msg,
                                 sensor_msgs::PointCloud2ConstPtr back_msg) {
  //pcl::PointCloud<PPoint>::Ptr left_pointcloud(new pcl::PointCloud<PPoint>());
  //pcl::PointCloud<PPoint>::Ptr right_pointcloud(new pcl::PointCloud<PPoint>());
  //pcl::PointCloud<PPoint>::Ptr back_pointcloud(new pcl::PointCloud<PPoint>());
  boost::shared_ptr<PPointCloud> left_pointcloud(new PPointCloud());
  boost::shared_ptr<PPointCloud> right_pointcloud(new PPointCloud());
  boost::shared_ptr<PPointCloud> back_pointcloud(new PPointCloud());
  boost::shared_ptr<PPointCloud> fused_pointcloud(new PPointCloud());
  pcl::fromROSMsg(*left_msg, *left_pointcloud);
  pcl::fromROSMsg(*right_msg, *right_pointcloud);
  pcl::fromROSMsg(*back_msg, *back_pointcloud);

  Process(left_pointcloud,right_pointcloud,back_pointcloud,fused_pointcloud);

  sensor_msgs::PointCloud2::Ptr q_msg(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*fused_pointcloud,*q_msg);
  q_msg->header.frame_id = fused_lidar_frame_id_;
  fusion_pub_.publish(*q_msg);
  }

bool Fusion::QueryPoseAffine(const std::string& target_frame_id,
                            const std::string& source_frame_id,Eigen::Affine3d* pose) {
  std::string err_string;
  if (!tf2_buffer_.canTransform(target_frame_id, source_frame_id,ros::Time(0),
                                ros::Duration(tf_timeout_), &err_string)) {
    ROS_WARN_STREAM("Can not find transform. "
           << "target_id:" << target_frame_id << " frame_id:" << source_frame_id
           << " Error info: " << err_string);
    return false;
  }
  geometry_msgs::TransformStamped stamped_transform;
  try {
    stamped_transform = tf2_buffer_.lookupTransform(
        target_frame_id, source_frame_id, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }
  tf::transformMsgToEigen(stamped_transform.transform, *pose);
  // ROS_DEBUG_STREAM("pose matrix : " << pose);
  return true;
}

void Fusion::AppendPointCloud(boost::shared_ptr<PPointCloud> point_cloud,
    boost::shared_ptr<PPointCloud> point_cloud_add, const Eigen::Affine3d& pose) {
    boost::shared_ptr<PPointCloud> transformed_pointcloud(new PPointCloud());
    pcl::transformPointCloud(*point_cloud_add, *transformed_pointcloud, pose);
    *point_cloud = *point_cloud + *transformed_pointcloud;
}


bool Fusion::Process(boost::shared_ptr<PPointCloud> left,boost::shared_ptr<PPointCloud> right,
              boost::shared_ptr<PPointCloud> back,boost::shared_ptr<PPointCloud> fused) {
  Eigen::Affine3d pose;
  if (!QueryPoseAffine(fused_lidar_frame_id_,left_lidar_frame_id_,
                      &pose)) {return false;}
    AppendPointCloud(fused,left,pose);
  if (!QueryPoseAffine(fused_lidar_frame_id_, right_lidar_frame_id_,
                      &pose)) {return false;}
    AppendPointCloud(fused,right,pose);
  if (!QueryPoseAffine(fused_lidar_frame_id_, back_lidar_frame_id_,
                      &pose)) {return false;}
    AppendPointCloud(fused,back,pose);
  
  return true;
}


}  // namespace pandora
}  // namespace drivers
}  // namespace jmc_auto
