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
#include "modules/planning/planning.h"
//#define _GNU_SOURCE
//#include "cyber/common/file.h"
#include "modules/common/util/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
//#include "modules/common/util/threadpool.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
//#include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"

#include "modules/common/time/time.h"
#include "modules/common/adapters/adapter_manager.h"

namespace jmc_auto {
namespace planning {

using jmc_auto::hdmap::HDMapUtil;
using jmc_auto::hdmap::ParkingSpaceInfoConstPtr;
using jmc_auto::common::math::Vec2d;
//using jmc_auto::perception::TrafficLightDetection;
//using jmc_auto::relative_map::MapMsg;
using jmc_auto::routing::RoutingRequest;
using jmc_auto::routing::RoutingResponse;
//using jmc_auto::control::PadMessage;

using jmc_auto::common::adapter::AdapterManager;
using jmc_auto::common::Status;
using jmc_auto::common::ErrorCode;

Planning::~Planning() { Stop(); }

std::string Planning::Name() const { return "planning"; }

#define CHECK_ADAPTER(NAME)                                               \
  if (AdapterManager::Get##NAME() == nullptr) {                           \
    AERROR << #NAME << " is not registered";                              \
    return Status(ErrorCode::PLANNING_ERROR, #NAME " is not registered"); \
  }


Status Planning::Init() {
//  cpu_set_t mask;
//  CPU_ZERO(&mask);
//  CPU_SET(0, &mask);
//  CPU_SET(1, &mask);
//  CPU_SET(2, &mask);
//  CPU_SET(3, &mask);
//  CPU_SET(4, &mask);
//  CPU_SET(5, &mask);

//  sched_setaffinity(0, sizeof(cpu_set_t), &mask);
  if (FLAGS_use_navigation_mode) {
    //planning_base_ = std::make_unique<NaviPlanning>();
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>();
  }

  CHECK(jmc_auto::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                                 &config_))
      << "failed to load planning config file " << FLAGS_planning_config_file;
  // CHECK(jmc_auto::common::util::GetProtoFromFile(FLAGS_planning_mode_config_file,
  //                                                &mode_config_))
  //     << "failed to load planning mode config file " << FLAGS_planning_mode_config_file;

  planning_base_->Init(config_);

  //判断需要启动的模式
  // if(mode_config_.calldriving().enabled()){
  //     routing_request_ = mode_config_.calldriving().routingrequest();
  //     AdapterManager::FillRoutingRequestHeader(Name(), &routing_request_);
  //     AdapterManager::PublishRoutingRequest(routing_request_);
  //     AINFO << "planning mode is call driving";
  // }else if(mode_config_.parking().enabled()){
  //   if(mode_config_.parking().ownparking().enabled()){
  //     routing_request_ = mode_config_.parking().ownparking().routingrequest();
  //     AdapterManager::FillRoutingRequestHeader(Name(), &routing_request_);
  //     AdapterManager::PublishRoutingRequest(routing_request_);
  //     AINFO << "planning mode is own parking";
  //   }else if(mode_config_.parking().flowparking().enabled()){
  //     routing_request_ = mode_config_.parking().ownparking().routingrequest();
  //     AdapterManager::FillRoutingRequestHeader(Name(), &routing_request_);
  //     AdapterManager::PublishRoutingRequest(routing_request_);
  //     AINFO << "planning mode is flow parking";
  //   }else{
  //     AERROR << "parking mode is error";
  //     return Status(ErrorCode::PLANNING_ERROR, "parking mode is error");
  //   }
  // }else{
  //   AERROR << "mode is not set";
  //   return Status(ErrorCode::PLANNING_ERROR, "mode is not set");
  // }


  // routing_reader_ = node_->CreateReader<RoutingResponse>(
  //     FLAGS_routing_response_topic,
  //     [this](const std::shared_ptr<RoutingResponse>& routing) {
  //       AINFO << "Received routing data: run routing callback."
  //             << routing->header().DebugString();
  //       std::lock_guard<std::mutex> lock(mutex_);
  //       routing_.CopyFrom(*routing);
  //     });
  // traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
  //     FLAGS_traffic_light_detection_topic,
  //     [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
  //       ADEBUG << "Received traffic light data: run traffic light callback.";
  //       std::lock_guard<std::mutex> lock(mutex_);
  //       traffic_light_.CopyFrom(*traffic_light);
  //     });

  // pad_msg_reader_ = node_->CreateReader<PadMessage>(
  //     FLAGS_planning_pad_topic,
  //     [this](const std::shared_ptr<PadMessage>& pad_msg) {
  //       ADEBUG << "Received pad data: run pad callback.";
  //       std::lock_guard<std::mutex> lock(mutex_);
  //       pad_msg_.CopyFrom(*pad_msg);
  //     });

  // initialize planning thread pool
  // PlanningThreadPool::instance()->Init();

//  if (!AdapterManager::Initialized()) {
    AdapterManager::Init(FLAGS_planning_adapter_config_filename);
//  }
  CHECK_ADAPTER(Localization);
  CHECK_ADAPTER(Chassis);
  CHECK_ADAPTER(RoutingResponse);
  CHECK_ADAPTER(RoutingRequest);
  CHECK_ADAPTER(Prediction);
  //CHECK_ADAPTER(Pad);
  CHECK_ADAPTER(PlanningPad);
  CHECK_ADAPTER(TteContiRadar);
  CHECK_ADAPTER(ValidSlot);

  AdapterManager::AddRoutingResponseCallback(&Planning::RoutingCallback, this);
  //AdapterManager::AddPadCallback(&Planning::PadCallback, this);

  // if (FLAGS_use_navigation_mode) {
  //   relative_map_reader_ = node_->CreateReader<MapMsg>(
  //       FLAGS_relative_map_topic,
  //       [this](const std::shared_ptr<MapMsg>& map_message) {
  //         ADEBUG << "Received relative map data: run relative map callback.";
  //         std::lock_guard<std::mutex> lock(mutex_);
  //         relative_map_.CopyFrom(*map_message);
  //       });
  // }
  // planning_writer_ =
  //     node_->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);

  // rerouting_writer_ =
  //     node_->CreateWriter<RoutingRequest>(FLAGS_routing_request_topic);

  return Status::OK();
}

void Planning::RoutingCallback(const routing::RoutingResponse &routing){
  ADEBUG << "Routing msg";
  routing_.CopyFrom(routing);
 // local_view_.routing = std::make_shared<routing::RoutingResponse>(routing);
}

/*
void Planning::PadCallback(const control::PadMessage &pad_msg){
  ADEBUG << "parkingnoid pad msg debug : " << pad_msg.DebugString();
  if(pad_msg.has_point()){
   pad_msg_.CopyFrom(pad_msg);
   ADEBUG << "parkingnoid pad msg has point.";
   parking_position_ = -1;
   parking_to_parking_status_ = false;
  }
// local_view_.pad_msg_ = std::make_shared<control::PadMessage>(pad_msg);
}*/

Status Planning::Start() {
  timer_ = AdapterManager::CreateTimer(
      ros::Duration(1.0 / FLAGS_planning_loop_rate), &Planning::OnTimer, this);
  // The "reference_line_provider_" may not be created yet in navigation mode.
  // It is necessary to check its existence.
  // if (reference_line_provider_) {
  //   reference_line_provider_->Start();
  // }
  //start_time_ = Clock::NowInSeconds();
  AINFO << "Planning started";
  return Status::OK();
}

void Planning::OnTimer(const ros::TimerEvent&) {
  AdapterManager::Observe();
  auto chassis_adapter = AdapterManager::GetChassis();
  if (chassis_adapter->Empty()){
    AINFO<< "not chassis msg";
    return;
  }
  //ACHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  CheckRerouting();

  //流动车位模式获取空闲车位信息
  // if(mode_config_.parking().flowparking().enabled()){
  //   local_view_.parkingspace = AdapterManager::GetParkingSpaceDetection()->GetLatestObserved();
  //   CheckRerouting();
  // }
  // process fused input data
  //local_view_.prediction_obstacles = prediction_obstacles;
  chassis_ = chassis_adapter->GetLatestObserved();
  auto localization_adapter = AdapterManager::GetLocalization();
  if (localization_adapter->Empty()){
    AINFO<< "not localization msg";
    return;
  }
  localization_ = localization_adapter->GetLatestObserved();
  local_view_.chassis = std::make_shared<canbus::Chassis>(chassis_);
  local_view_.localization_estimate = std::make_shared<localization::LocalizationEstimate>(localization_);
  //routing_ = AdapterManager::GetRoutingResponse()->GetLatestObservedStdPtr();
  auto prediction_adapter = AdapterManager::GetPrediction();
  if (!prediction_adapter->Empty()){
    prediction_obstacles_ = prediction_adapter->GetLatestObserved();
    local_view_.prediction_obstacles = std::make_shared<prediction::PredictionObstacles>(prediction_obstacles_);
//    if(prediction_obstacles_.prediction_obstacle().size() > 0){
//      ADEBUG << "PREDICTION OBSTACLES";
//    }
//    ADEBUG << "Prediction obstacles nums: " << local_view_.prediction_obstacles->prediction_obstacle().size();
  }

  auto tte_radar_adapter = AdapterManager::GetTteContiRadar();
  if (!tte_radar_adapter->Empty()){
    tte_radar_ = tte_radar_adapter->GetLatestObserved();
    local_view_.tte_radar = std::make_shared<drivers::TteContiRadar>(tte_radar_);
  }


  auto valid_slot_adapter = AdapterManager::GetValidSlot();
  if (!valid_slot_adapter->Empty()){
    valid_slots_ = valid_slot_adapter->GetLatestObserved();
    local_view_.valid_slots = std::make_shared<perception::MulvalidSlot>(valid_slots_);
    ADEBUG << "parkingnoid SLOTS HAS num : " << valid_slots_.slotsum_size();
  }

  auto planning_pad_adapter = AdapterManager::GetPlanningPad();
  if (!planning_pad_adapter->Empty()){
    auto pad_msg = planning_pad_adapter->GetLatestObserved();
    if(!common::util::IsProtoEqual(pad_msg, pad_msg_)){
      pad_msg_ = pad_msg;
      ADEBUG << "parkingnoid receive summon debug : " << pad_msg_.DebugString();
      //ADEBUG << "parkingnoid pad msg has point.";
      parking_position_ = -1;
      parking_to_parking_status_ = false;
    }
  }
  
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing = std::make_shared<routing::RoutingResponse>(routing_);
          //std::make_shared<routing::RoutingResponse>(routing_);
    }
  }
  // {
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   local_view_.traffic_light =
  //       std::make_shared<TrafficLightDetection>(traffic_light_);
  //   local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  // }
   
  // ADEBUG << "pad_msg source x:" << pad_msg_.point().end().x();
   {
     std::lock_guard<std::mutex> lock(mutex_);
     local_view_.pad_msg = std::make_shared<planning::PadMessage>(pad_msg_);
   }


  // if(local_view_.routing->waypoint().pose() == local_view_.localization_estimate->position() && 
  //    local_view_.chassis->speed() <= 1e-2){
  //   if(mode_config_.parking().enabled()){
  //     //切换泊车系统

  //   }else if(mode_config_.calldriving().enabled()){
  //     //上报召唤结果

  //   }
  // }
  // else{
  if (!CheckInput()) {
    AERROR << "Input check failed";
    return;
  }
 
 // count_++;
//  ADEBUG << "COUNT:" << count_;
//  if(count_ > 30){
//  ADEBUG << "parking_id:" << parking_id_;
  CheckParkingSpace();
//  parking_id_.set_id(local_view_.parkingspace_id);
//  }

  ADCTrajectory adc_trajectory_pb;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  AdapterManager::FillPlanningHeader(Name(), &adc_trajectory_pb);

  // modify trajectory relative time due to the timestamp change in header
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
   // if(p.path_point().kappa() > -0.03 && p.path_point().kappa() < 0){
   //   p.mutable_path_point()->set_kappa(p.path_point().kappa()*0.1);
//      p.path_point().kappa() *= 0.1;
   // }
  }
  AdapterManager::PublishPlanning(adc_trajectory_pb);

  // record in history
  auto* history = History::Instance();
  history->Add(adc_trajectory_pb);
//  }
}

void Planning::CheckParkingSpace(){
  if(pad_msg_.appmode() == planning::PARKING || pad_msg_.appmode() == planning::SUMMON_INPARKING){
    local_view_.parkingspace_id = pad_msg_.parking_space_info().id();
    ADEBUG << "APPMODE: " << pad_msg_.appmode() << " parking_ID: " << pad_msg_.parking_space_info().id();
    //parking_position_ = -1;
//    NOID_parking_id_ = false;
  }
  else if(pad_msg_.appmode() == planning::PARKING_NOID && valid_slots_.slotsum_size() > 0){
//    bool update_parking = true;
   ADEBUG << "parkingnoid slotsum num : " << valid_slots_.slotsum_size();
//   int count = 0;
   for(auto slot : valid_slots_.slotsum()){
//     std::string parking_id = slot.id_v().id();
//     ADEBUG << "parkingnoid slotsum id : " << slot.id_v().id() << " count : " << count;
     ADEBUG << "parkingnoid slotsum id : " << slot.id_v().id();
     if(std::abs(parking_position_ - slot.polygon_v().point(0).x()) < 1){
       double distance = ((slot.polygon_v().point(0).x() - localization_.pose().position().x())*(slot.polygon_v().point(0).x() - localization_.pose().position().x()))
                       + ((slot.polygon_v().point(0).y() - localization_.pose().position().y())*(slot.polygon_v().point(0).y() - localization_.pose().position().y()));
       ADEBUG << "parkingnoid target distance : " << distance;
       ADEBUG << "parkingnoid target id : " << slot.id_v().id();
       if(distance > 60){
         parking_position_ = -1;
         ADEBUG << "parkingnoid target too far, resume.";
         break;
       }
      //  update_parking = false;
      //  break;
       local_view_.parkingspace_id = slot.id_v().id();
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_left_up()->set_x(slot.polygon_v().point(0).x());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_left_up()->set_y(slot.polygon_v().point(0).y());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_left_down()->set_x(slot.polygon_v().point(1).x());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_left_down()->set_y(slot.polygon_v().point(1).y());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_right_down()->set_x(slot.polygon_v().point(2).x());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_right_down()->set_y(slot.polygon_v().point(2).y());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_right_up()->set_x(slot.polygon_v().point(3).x());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_right_up()->set_y(slot.polygon_v().point(3).y());
      // local_view_.pad_msg->mutable_parking_space_info()->mutable_point_left_up()->set_x(slot.polygon_v().point(0).x());
      // local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_left_up(slot.polygon_v().point(0));
      // local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_left_down(slot.polygon_v().point(1));
      // local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_right_down(slot.polygon_v().point(2));
      // local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_right_up(slot.polygon_v().point(3));
       break;
     }
//     ++count;
   }
//    for(auto slot : valid_slots_.slotsum()){
    //  std::string parking_id = slot.id_v().id();
//      if(parking_id_.id() == slot.id_v().id()){
//        double distance = ((slot.polygon_v().point(0).x() - localization_.pose().position().x())*(slot.polygon_v().point(0).x() - localization_.pose().position().x()))
//                        + ((slot.polygon_v().point(0).y() - localization_.pose().position().y())*(slot.polygon_v().point(0).y() - localization_.pose().position().y()));
//        ADEBUG << "PARKINGNOID Distance:" << distance;
//        if(distance > 144){
//          break;
//        }
//        update_parking = false;
//        break;
//        local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_left_up(slot.polygon_v().point(0));
//        local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_left_down(slot.polygon_v().point(1));
//        local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_right_down(slot.polygon_v().point(2));
//        local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_right_up(slot.polygon_v().point(3));
//        break;
//      }
//    }
    if(parking_position_ < 0){
      auto size = valid_slots_.slotsum_size();
      ADEBUG << "parkingnoid init valid slots num : " << size;
      auto Slot = valid_slots_.slotsum(0);
      double distance = ((Slot.polygon_v().point(0).x() - localization_.pose().position().x())*(Slot.polygon_v().point(0).x() - localization_.pose().position().x()))
                        + ((Slot.polygon_v().point(0).y() - localization_.pose().position().y())*(Slot.polygon_v().point(0).y() - localization_.pose().position().y()));
      ADEBUG << "parkingnoid init distance : " << distance;
      ADEBUG << "parkingnoid init candidate id : " << Slot.id_v().id();
      if(distance <= 60){
        //滤除弯道车位
       // const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
       // char *data_ptr;
       // char *temp;
       // std::vector<std::string> road_id;
       // data_ptr = (char*)Slot.id_v().id().c_str();
       // const char * split = "_";
       // temp = strtok(data_ptr, split);
       // while (temp != NULL)
       // {
       //   road_id.push_back(temp);
       //   temp = strtok(NULL,split);
       //}
       // hdmap::Id id;
       // id.set_id(road_id[1]);
       // auto target_parking_road_ptr = hdmap->GetRoadById(id);
       // if(target_parking_road_ptr != nullptr && target_parking_road_ptr->sections()[0].lane_id_size() == 1){
       //   ADEBUG << "The Parking on the turn lane";
       //   local_view_.parkingspace_id = "";
       //   return;
       // }
        ADEBUG << "parkingnoid init target id : " << Slot.id_v().id();
        local_view_.parkingspace_id = Slot.id_v().id();
        parking_position_ = Slot.polygon_v().point(0).x();
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_left_up()->set_x(Slot.polygon_v().point(0).x());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_left_up()->set_y(Slot.polygon_v().point(0).y());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_left_down()->set_x(Slot.polygon_v().point(1).x());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_left_down()->set_y(Slot.polygon_v().point(1).y());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_right_down()->set_x(Slot.polygon_v().point(2).x());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_right_down()->set_y(Slot.polygon_v().point(2).y());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_right_up()->set_x(Slot.polygon_v().point(3).x());
       local_view_.pad_msg->mutable_parking_space_info()->mutable_point_right_up()->set_y(Slot.polygon_v().point(3).y());

       // local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_left_up(Slot.polygon_v().point(0));
       // local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_left_down(Slot.polygon_v().point(1));
       // local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_right_down(Slot.polygon_v().point(2));
       // local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_right_up(Slot.polygon_v().point(3));
      }
//      else{
//        NOID_parking_id_ = false;
//      }
        //local_view_.parkingspace_id = Slot.id_v().id();
     // }                  
//      local_view_.parkingspace_id = Slot.id_v().id();
//      local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_left_up(Slot.polygon_v().point(0));
//      local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_left_down(Slot.polygon_v().point(1));
//      local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_right_down(Slot.polygon_v().point(2));
//      local_view_.pad_msg->mutable_parking_space_info()->set_allocated_point_right_up(Slot.polygon_v().point(3));
    }
  }
  //车位到车位
  else if(pad_msg_.appmode() == planning::PARKING_TO_PARKING){
    if(parking_to_parking_status_){
      local_view_.pad_msg->set_appmode(planning::PARKING);
      local_view_.parkingspace_id = pad_msg_.parking_space_info().id();
      ADEBUG << "to parking summon. parking in stage";
    }else{
      local_view_.pad_msg->set_appmode(planning::SUMMON_INPARKING);
      local_view_.parkingspace_id = pad_msg_.to_parking_space_info().id();
      local_view_.pad_msg->mutable_parking_space_info()->CopyFrom(pad_msg_.to_parking_space_info());
      ADEBUG << "to parking. parking out stage";
    }
  }
  else{
    ADEBUG << "parkingnoid : else appmode or no slots. clear parkingspace id.";
    local_view_.parkingspace_id = "";
    parking_position_ = -1;
  //  NOID_parking_id_ = false;
  }

//  parking_id = local_view_.parkingspace_id;
//  ParkingSpaceInfoConstPtr target_parking_spot_ptr;
//  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
      // TODO(Jinyun) parking overlap s are wrong on map, not usable
      // target_area_center_s =
      //     (parking_overlap.start_s + parking_overlap.end_s) / 2.0;
//      hdmap::Id id;
//      std::string park_id = FLAGS_test_parkingspace_id;
      //std::string park_id = "1262";
//      id.set_id(park_id);
      //金融厂区地图
      //Vec2d right_top_point = (hdmap->GetParkingSpaceById(id))->polygon().points().at(1);
      //晶众厂区地图
//      Vec2d right_top_point = (hdmap->GetParkingSpaceById(id))->polygon().points().at(3);
//      AERROR << "Find ParkingSpace";
      // Vec2d left_bottom_point =
      //     target_parking_spot_ptr->polygon().points().at(0);
//      double distance = ((right_top_point.x() - localization_.pose().position().x())*(right_top_point.x() - localization_.pose().position().x()))
//                       + ((right_top_point.y() - localization_.pose().position().y())*(right_top_point.y() - localization_.pose().position().y()));
//      if(distance < 1000){
//        local_view_.parkingspace_id = park_id;
//      }else{
//        AERROR << "ParkingSpace too far";
//      }
}


//检查是否需要再routing
void Planning::CheckRerouting() {
  // if(!local_view_.parkingspace){
  //   if(!common::util::IsProtoEqual(local_view_.parkingspace, parkingspace_)){
  //     local_parkingspace = local_view_.parkingspace;
  //     //不能发布车位，还需要修改
  //     AdapterManager::FillPlanningHeader(Name(), &parkingspace_);
  //     AdapterManager::PublishRoutingRequest(parkingspace_);
  //   }
  // }

   auto* rerouting = PlanningContext::Instance()
                         ->mutable_planning_status()
                         ->mutable_rerouting();
   if (!rerouting->need_rerouting()) {
     return;
   }
   AdapterManager::FillRoutingRequestHeader(Name(), rerouting->mutable_routing_request());
   rerouting->set_need_rerouting(false);
   parking_to_parking_status_ = true;
   AdapterManager::PublishRoutingRequest(rerouting->routing_request());
   ADEBUG << "rerouting publish";
// rerouting_writer_->Write(rerouting->routing_request());
}

bool Planning::CheckInput() {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  } else {
    // nothing
  }

//  if (FLAGS_use_navigation_mode) {
    // if (!local_view_.relative_map->has_header()) {
    //   not_ready->set_reason("relative map not ready");
    // }
//  } else {
    if (!local_view_.routing->has_header()) {
      not_ready->set_reason("routing not ready");
    }
//  }

  if (not_ready->has_reason()) {
    AERROR << not_ready->reason() << "; skip the planning cycle.";
    AdapterManager::FillPlanningHeader(Name(), &trajectory_pb);
    AdapterManager::PublishPlanning(trajectory_pb);
    return false;
  }
  return true;
}

void Planning::Stop() {
  AERROR << "Planning Stop is called";
  // PlanningThreadPool::instance()->Stop();
  // if (reference_line_provider_) {
  //   reference_line_provider_->Stop();
  // }
  // last_publishable_trajectory_.reset(nullptr);
  // frame_.reset(nullptr);
  planning_base_.reset(nullptr);
 // FrameHistory::instance()->Clear();
}

}  // namespace planning
}  // namespace jmc_auto
