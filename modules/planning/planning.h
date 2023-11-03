/******************************************************************************
 * Copyright 2018 The jmc_auto Authors. All Rights Reserved.
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

#pragma once

#include <memory>

// #include "cyber/class_loader/class_loader.h"
// #include "cyber/component/component.h"
// #include "cyber/message/raw_message.h"

#include "modules/common/jmc_auto_app.h"
#include "modules/common/status/status.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
//#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planning_base.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_mode_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
//#include "modules/planning/common/planning_util.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/drivers/proto/tte_conti_radar.pb.h"
#include "modules/perception/proto/slotcheck_Info.pb.h"

namespace jmc_auto {
namespace planning {

class Planning : public jmc_auto::common::JmcAutoApp {
 public:
  Planning() = default;

  virtual ~Planning();

 public:
  jmc_auto::common::Status Init() override;

  std::string Name() const override;

  jmc_auto::common::Status Start() override;

  bool IsNewParkingSpace();

  void RoutingCallback(const routing::RoutingResponse &routing);
  
  //void PadCallback(const control::PadMessage &pad_msg);
  
  void Stop() override;
//   bool Proc(const std::shared_ptr<prediction::PredictionObstacles>&
//                 prediction_obstacles,
//             const std::shared_ptr<canbus::Chassis>& chassis,
//             const std::shared_ptr<localization::LocalizationEstimate>&
//                 localization_estimate) override;

 private:
  void CheckRerouting();
  bool CheckInput();
  void CheckParkingSpace();

 private:
 void OnTimer(const ros::TimerEvent&);
//   std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
//       traffic_light_reader_;
//   std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
//   std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
//   std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;

//   std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
//   std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;

  std::mutex mutex_;
  ros::Timer timer_;
//  perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;
  planning::PadMessage pad_msg_;
  routing::RoutingRequest  routing_request_;
  canbus::Chassis chassis_;
  localization::LocalizationEstimate localization_;
  prediction::PredictionObstacles prediction_obstacles_;
  drivers::TteContiRadar tte_radar_;
  perception::MulvalidSlot valid_slots_;

//  perception::ParkingSpaceDetection parkingspace_;
//  planning::PadMessage pad_msg_;
//  relative_map::MapMsg relative_map_;

  LocalView local_view_;
  double parking_position_ = -1;
  bool parking_to_parking_status_ = false;  //false为泊出阶段   true为泊入阶段

  std::unique_ptr<PlanningBase> planning_base_;

//测试计数用
//  int count_ = 0;

  PlanningConfig config_;
  //PlanningModeConfig mode_config_;
};

//CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace jmc_auto
