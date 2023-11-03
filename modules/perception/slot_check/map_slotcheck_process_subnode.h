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
//210702-根据测试情况,将队列q1设置为Vector来进行编译说明。

#ifndef MODULES_PERCEPTION_SLOT_CHECK_MAP_SLOTCHECK_PROCESS_SUBNODE_H_
#define MODULES_PERCEPTION_SLOT_CHECK_MAP_SLOTCHECK_PROCESS_SUBNODE_H_
#include <boost/circular_buffer.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <queue> 

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/slotcheck_Info.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/drivers/proto/tte_conti_radar.pb.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
// #include "modules/control/proto/pad_msg.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"

namespace jmc_auto {
namespace perception {

class MapSlotCheckProcessSubnode : public Subnode 
{
 public:
 MapSlotCheckProcessSubnode() = default;
 ~MapSlotCheckProcessSubnode() = default;

 jmc_auto::common::Status ProcEvents() override 
  {return jmc_auto::common::Status::OK();}

 void OnPad(const jmc_auto::control::PadMessage &pad_msg);
 void OnPad(const jmc_auto::planning::PadMessage &pad_msg);
 void SearchUnvalidParkingSpace();

 private:
 //typedef std::pair<double, jmc_auto::localization::LocalizationEstimate> LocalizationPair;
 bool InitInternal(void) override;
 void PublishValidSlot(void); 
 void OnTimer(const ros::TimerEvent&);
 bool CheckRrsObstacle(double rrs_distance);
 double CalculateaAbDistance(const Eigen::Vector3d &point_a, const Eigen::Vector3d &point_b);
 Eigen::Vector3d PointAPositionEstimate(double distance, bool obstacle_appear);
 void SlotJudge(const std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> &parking_spaces, const Eigen::Vector3d &point_a, const Eigen::Vector3d &point_b, const Eigen::Vector3d &point_c, const Eigen::Vector3d &point_d);

bool Judge_vector_pair_id(std::vector<std::pair<std::string, jmc_auto::perception::ValidMapSlot>> vector_pair,std::string find_id);

 ros::Timer timer_;
 const jmc_auto::hdmap::HDMap *hdmap_ = nullptr; 
 jmc_auto::canbus::Chassis chassis_; 
 jmc_auto::drivers::TteContiRadar tte_;
 jmc_auto::localization::LocalizationEstimate localization_;
 jmc_auto::perception::PerceptionObstacles perception_obstacles_;
// jmc_auto::common::Quaternion quaternion;
 //jmc_auto::perception::ValidMapSlot slot_;   //在package jmc_auto::perception目录下，自动在proto内搜寻匹配的message名字
 //jmc_auto::hdmap::ParkingSpaceInfoConstPtr parking_spaces;
 common::ErrorCode error_code_ = common::OK;
//  std::unordered_map<std::string, jmc_auto::perception::ValidMapSlot> slots_;//有效车位
 std::vector<std::pair<std::string, jmc_auto::perception::ValidMapSlot>> slots_;//有效车位
 std::vector<std::string> unvalid_slots_;//无效车位
 //std::vector<jmc_auto::perception::ValidMapSlot> slots_;//将q1设置为Vector类型，而不是队列形式。

 //const double PI = 3.1415926535897932;
 bool Need_Check_ = false; // 是否需要检测车位
 double last_distance_; // 上一帧障碍物距离
 int obs_count_;  //超声波障碍物检测次数
 
// bool initialized_ = false;

// Eigen::Vector3d vehicle_pose;

  //车辆坐标系转换为大地坐标系
 Eigen::Vector3d CoordinateTrans(const jmc_auto::common::Quaternion &quater, const Eigen::Vector3d &vec, const Eigen::Vector3d &vehicle_pose) 
  {
      Eigen::Quaternion<double> q(quater.qw(),quater.qx(), quater.qy(), quater.qz());
    //return quaternion.toRotationMatrix().inverse() * p + vehpose;
      return q.toRotationMatrix() * vec + vehicle_pose;
  }

  enum SlotCheckObstacleStatus {
    HAS_OBS = 1,
    NO_OBS = 2,
    OBS_APPEAR = 3,
    OBS_DISAPPEAR = 4,
  };
//  SlotCheckObstacleStatus obstacle_status_;

//  bool has_obstacle_now_ = false;  //当前计算循环是否有障碍物
  bool has_obstacle_last_ = false; //上一计算循环是否有障碍物

  const double m1_b_x_rfu_ = 1.0;      //以RRS传感器在车辆坐标系点作为初始可泊车区域的范围B点和C点起点。参考涉及RRS探测距离为5.5m。
  const double m1_b_y_rfu_ = -0.5; 
  const double m1_c_x_rfu_ = 6.5;
  const double m1_c_y_rfu_ = -0.5;
  Eigen::Vector3d m1_pointB_enu_;
  Eigen::Vector3d m1_pointC_enu_;
  Eigen::Vector3d m1_pointA_enu_;
  Eigen::Vector3d m1_pointD_enu_;
};

REGISTER_SUBNODE(MapSlotCheckProcessSubnode);

} // namespace perception
} // namespace jmc_auto

#endif // MODULES_PERCEPTION_SLOT_CHECK_SLOTCHECK_PROCESS_SUBNODE_H_
