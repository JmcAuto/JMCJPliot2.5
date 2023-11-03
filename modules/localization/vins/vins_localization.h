/******************************************************************************
 * Copyright 2017 The jmc_auto Authors. All Rights Reserved.
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

/**
 * @file rtk_localization.h
 * @brief The class of VINSLocalization
 */

#ifndef MODULES_LOCALIZATION_VINS_VINS_LOCALIZATION_H_
#define MODULES_LOCALIZATION_VINS_VINS_LOCALIZATION_H_

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

// #include "modules/localization/proto/gps.pb.h"
// #include "modules/localization/proto/imu.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "glog/logging.h"
#include "gtest/gtest_prod.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/localization/localization_base.h"
#include <proj_api.h>

#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "modules/localization/vins/apriltag/apriltag_pose.h"

/**
 * @namespace jmc_auto::localization
 * @brief jmc_auto::localization
 */
namespace jmc_auto {
namespace localization {

/**
 * @class VINSLocalization
 *
 * @brief generate localization info based on VINS
 */
class VINSLocalization : public LocalizationBase {
 public:
  VINSLocalization();
  virtual ~VINSLocalization();

  /**
   * @brief module start function
   * @return start status
   */
  jmc_auto::common::Status Start() override;

  /**
   * @brief module stop function
   * @return stop status
   */
  jmc_auto::common::Status Stop() override;

 private:
  void OnTimer(const ros::TimerEvent &event);
  // void OnChassis(const jmc_auto::canbus::Chassis &Chassis);
  void PublishLocalization();
  void RunWatchDog(LocalizationEstimate *localization);

  void PrepareLocalizationMsg(LocalizationEstimate *localization);
  // void ComposeLocalizationMsg(const localization::Gps &gps,
  //                             const localization::CorrectedImu &imu,
  //                             LocalizationEstimate *localization);
  // bool FindMatchingIMU(const double gps_timestamp_sec, CorrectedImu *imu_msg);
  // bool InterpolateIMU(const CorrectedImu &imu1, const CorrectedImu &imu2,
  //                     const double timestamp_sec, CorrectedImu *msgbuf);
  // template <class T>
  // T InterpolateXYZ(const T &p1, const T &p2, const double frac1);

 private:
  ros::Timer timer_;
  jmc_auto::common::monitor::MonitorLogger monitor_logger_;
  const std::vector<double> map_offset_;
  double last_received_timestamp_sec_ = 0.0;
  double last_reported_timestamp_sec_ = 0.0; 
  // bool service_started_ = false;
  projPJ wgs84pj_source_;
  projPJ utm_target_;
  jmc_auto::canbus::Chassis chassis_;
  // sensor_msgs::Image Image_msg_;//ros msg 格式的类型
  boost::shared_ptr<const sensor_msgs::Image> Image_msg_;
  cv::Mat img_;//opencv mat的数据类型
  apriltag_detector_t *td;
  apriltag_family_t *tf = NULL;


  const double DEG_TO_RAD_LOCAL = M_PI / 180.0;
  const char *WGS84_TEXT = "+proj=latlong +ellps=WGS84";
  // FRIEND_TEST(VINSLocalizationTest, InterpolateIMU);
  // FRIEND_TEST(VINSLocalizationTest, ComposeLocalizationMsg);
};

}  // namespace localization
}  // namespace jmc_auto

#endif  // MODULES_LOCALIZATION_VINS_VINS_LOCALIZATION_H_
