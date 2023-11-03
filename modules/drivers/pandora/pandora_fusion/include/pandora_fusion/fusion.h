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

#ifndef MODULES_DRIVERS_PANDORA_PANDORA_FUSION_FUSION_H_
#define MODULES_DRIVERS_PANDORA_PANDORA_FUSION_FUSION_H_

#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Eigen>
#include <string>
#include <boost/function.hpp>

#include "pandora_fusion/point_types.h"

namespace jmc_auto {
namespace drivers {
namespace pandora {

class Fusion {
 public:
  Fusion(ros::NodeHandle node, ros::NodeHandle private_nh);
  virtual ~Fusion() {}

 private:
  /**
  * @brief get pointcloud2 msg, fused it,publish pointcloud2 after
  * fusion
  */
  void LidarFusionCallback(sensor_msgs::PointCloud2ConstPtr left_msg,
                            sensor_msgs::PointCloud2ConstPtr right_msg,
                            sensor_msgs::PointCloud2ConstPtr back_msg);
  /**
  * @brief get pose affine from tf2 by gps timestamp
  *   novatel-preprocess broadcast the tf2 transfrom.
  */
  bool QueryPoseAffine(const std::string& target_frame_id,
                       const std::string& source_frame_id,Eigen::Affine3d* pose);
  /**
   * @brief get pointcloud from diffrent lidar
   * add right and back cloud to the left
   */
  void AppendPointCloud(
    boost::shared_ptr<PPointCloud> point_cloud,
    boost::shared_ptr<PPointCloud> point_cloud_add, const Eigen::Affine3d& pose);
  /**
  * @brief check if message is valid, check width, height, timesatmp.
  *   set timestamp_offset and point data type
  */
  bool Process(boost::shared_ptr<PPointCloud> left,boost::shared_ptr<PPointCloud> right,
               boost::shared_ptr<PPointCloud> back,boost::shared_ptr<PPointCloud> fused);
                                  
  // publish point cloud2 after fusion
  ros::Publisher fusion_pub_;
  // ros::Publisher metastatus_publisher_;
  // tf2 buffer
  tf2_ros::Buffer tf2_buffer_;
  // tf2 transform listener to get transform by gps timestamp.
  tf2_ros::TransformListener tf2_transform_listener_;
  // transform frame id
  std::string fused_lidar_frame_id_;
  std::string left_lidar_frame_id_;
  std::string right_lidar_frame_id_;
  std::string back_lidar_frame_id_;
  //point cloud

  // time 
  float tf_timeout_;
  // topic names
  std::string topic_fused_pointcloud_;
  std::string topic_left_pointcloud_;
  std::string topic_right_pointcloud_;
  std::string topic_back_pointcloud_;
  // ros queue size for publisher and subscriber
  int queue_size_;
  // message synchronizer
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, 
                                                          sensor_msgs::PointCloud2,
                                                          sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy>* lidar_msg_sync_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* left_lidar_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* right_lidar_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* back_lidar_sub_;

};

}  // namespace pandora
}  // namespace drivers
}  // namespace jmc_auto

#endif  // MODULES_DRIVERS_PANDORA_PANDORA_FUSION_FUSION_H_
