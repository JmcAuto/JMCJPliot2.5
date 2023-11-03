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
#include <typeinfo>
#include "modules/dreamview/backend/simulation_world/simulation_world_updater.h"
#include "google/protobuf/util/json_util.h"
#include "modules/common/util/json_util.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/common/proto/geometry.pb.h"

namespace jmc_auto {
namespace dreamview {

using jmc_auto::common::adapter::AdapterManager;
using jmc_auto::common::monitor::MonitorMessageItem;
using jmc_auto::common::util::ContainsKey;
using jmc_auto::common::util::GetProtoFromASCIIFile;
using jmc_auto::common::util::JsonUtil;
using jmc_auto::hdmap::EndWayPointFile;
using jmc_auto::routing::RoutingRequest;
using Json = nlohmann::json;
using google::protobuf::util::JsonStringToMessage;
using google::protobuf::util::MessageToJsonString;
using jmc_auto::common::PointENU;
using jmc_auto::control::DrivingAction;
using jmc_auto::hdmap::ParkingSpaceInfoConstPtr;
using jmc_auto::planning::AppMode;

SimulationWorldUpdater::SimulationWorldUpdater(WebSocketHandler *websocket,
                                               WebSocketHandler *map_ws,
                                               SimControl *sim_control,
                                               const MapService *map_service,
                                               bool routing_from_file)
    : sim_world_service_(map_service, routing_from_file),
      map_service_(map_service),
      websocket_(websocket),
      map_ws_(map_ws),
      sim_control_(sim_control) {
  RegisterMessageHandlers();
}

void SimulationWorldUpdater::RegisterMessageHandlers() {
  // Send current sim_control status to the new client.
  websocket_->RegisterConnectionReadyHandler(
      [this](WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "SimControlStatus";
        response["enabled"] = sim_control_->IsEnabled();
        websocket_->SendData(conn, response.dump());
      });

  map_ws_->RegisterMessageHandler(
      "RetrieveMapData",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto iter = json.find("elements");
        if (iter != json.end()) {
          MapElementIds map_element_ids;
          if (JsonStringToMessage(iter->dump(), &map_element_ids).ok()) {
            auto retrieved = map_service_->RetrieveMapElements(map_element_ids);

            std::string retrieved_map_string;
            retrieved.SerializeToString(&retrieved_map_string);

            map_ws_->SendBinaryData(conn, retrieved_map_string, true);
          } else {
            AERROR << "Failed to parse MapElementIds from json";
          }
        }
      });

//  map_ws_->RegisterMessageHandler(
//      "RetrieveRelativeMapData",
//      [this](const Json &json, WebSocketHandler::Connection *conn) {
//        std::string to_send;
//        {
//          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
//          to_send = relative_map_string_;
//        }
//        map_ws_->SendBinaryData(conn, to_send, true);
//      });

//  websocket_->RegisterMessageHandler(
//      "Binary",
//      [this](const std::string &data, WebSocketHandler::Connection *conn) {
//        // Navigation info in binary format
//        jmc_auto::relative_map::NavigationInfo navigation_info;
//        if (navigation_info.ParseFromString(data)) {
//          AdapterManager::FillNavigationHeader(FLAGS_dreamview_module_name,
//                                               &navigation_info);
//          AdapterManager::PublishNavigation(navigation_info);
//        } else {
//          AERROR << "Failed to parse navigation info from string. String size: "
//                 << data.size();
//        }
//      });

  websocket_->RegisterMessageHandler(
      "RetrieveMapElementIdsByRadius",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto radius = json.find("radius");
        if (radius == json.end()) {
          AERROR << "Cannot retrieve map elements with unknown radius.";
          return;
        }

        if (!radius->is_number()) {
          AERROR << "Expect radius with type 'number', but was "
                 << radius->type_name();
          return;
        }

        Json response;
        response["type"] = "MapElementIds";
        response["mapRadius"] = *radius;

        MapElementIds ids;
        sim_world_service_.GetMapElementIds(*radius, &ids);
        std::string elementIds;
        MessageToJsonString(ids, &elementIds);
        response["mapElementIds"] = Json::parse(elementIds);

        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "SendRoutingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        flag_appmode = json["parking_status"];
        RoutingRequest routing_request;

        bool succeed = ConstructRoutingRequest(json, &routing_request);
        if (succeed) {
          AdapterManager::FillRoutingRequestHeader(FLAGS_dreamview_module_name,
                                                   &routing_request);
          AdapterManager::PublishRoutingRequest(routing_request);
        }

        // Publish monitor message.
        if (succeed) {
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::INFO,
                                                   "Routing request sent.");
        } else {
          sim_world_service_.PublishMonitorMessage(
              MonitorMessageItem::ERROR, "Failed to send a routing request.");
        }
      });

//对召唤指令通过topic发送出去
  websocket_->RegisterMessageHandler(
      "SendSummonRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto start = json["start"];
        auto end = json["end"];
        planning::PadMessage pad;
        jmc_auto::common::PointENU point;
        std::string id = "";
        std::vector<jmc_auto::common::PointENU> points;
        pad.set_appmode(AppMode::SUMMON);
        pad.mutable_point() -> mutable_start() -> set_x(start["x"]);
        pad.mutable_point() -> mutable_start() -> set_y(start["y"]);
        pad.mutable_point() -> mutable_end() -> set_x(end["x"]);
        pad.mutable_point() -> mutable_end() -> set_y(end["y"]);
        point.set_x(start["x"]);
        point.set_y(start["y"]);
        //AWARN << "your ids is: " << ids.DebugString();
        //如果点在车位内，将ID和坐标位置传给pad消息；点不在车位内，不作处理。
        if(map_service_ -> CheckPointInParkingSpace(point, id, points)){
            pad.set_appmode(AppMode::SUMMON_INPARKING);
            pad.mutable_parking_space_info() -> set_id(id);
            pad.mutable_parking_space_info() -> mutable_point_left_up() -> set_x(points[0].x());
            pad.mutable_parking_space_info() -> mutable_point_left_up() -> set_y(points[0].y());
            pad.mutable_parking_space_info() -> mutable_point_left_down() -> set_x(points[1].x());
            pad.mutable_parking_space_info() -> mutable_point_left_down() -> set_y(points[1].y());
            pad.mutable_parking_space_info() -> mutable_point_right_down() -> set_x(points[2].x());
            pad.mutable_parking_space_info() -> mutable_point_right_down() -> set_y(points[2].y());
            pad.mutable_parking_space_info() -> mutable_point_right_up() -> set_x(points[3].x());
            pad.mutable_parking_space_info() -> mutable_point_right_up() -> set_y(points[3].y());
        }
        AdapterManager::FillPlanningPadHeader("HMI", &pad);
        AdapterManager::PublishPlanningPad(pad);
        AINFO << "send Pad summon Msg:" << pad.DebugString();
        return;
      });

      //对召唤指令通过topic发送出去
  websocket_->RegisterMessageHandler(
      "SendParkingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto start = json["start"];
        auto end = json["end"];
        planning::PadMessage pad;
        jmc_auto::common::PointENU point;
        std::string id = "";
        std::vector<jmc_auto::common::PointENU> points;
        point.set_x(end["x"]);
        point.set_y(end["y"]);
        AINFO << "the parking end point is: " << point.DebugString();
        Json response;
        response["type"] = "ParkingPointCheck";
        response["start"] = start;
        response["end"] = end;
        response["status"] = 0;
        if(map_service_ -> CheckPointInParkingSpace(point, id, points)){
          //将状态发送给前端，绘制routing线
            response["status"] = 1;
            end["x"] = (points[0].x() +  points[3].x()) / 2;
            end["y"] = (points[0].y() +  points[3].y()) / 2;
            response["end"] = end;
 //           response["parking_id"] = id;
            websocket_->SendData(conn, response.dump());
            //pad.set_appmode(AppMode::PARKING);
            pad.mutable_parking_space_info() -> set_id(id);
            pad.mutable_parking_space_info() -> mutable_point_left_up() -> set_x(points[0].x());
            pad.mutable_parking_space_info() -> mutable_point_left_up() -> set_y(points[0].y());
            pad.mutable_parking_space_info() -> mutable_point_left_down() -> set_x(points[1].x());
            pad.mutable_parking_space_info() -> mutable_point_left_down() -> set_y(points[1].y());
            pad.mutable_parking_space_info() -> mutable_point_right_down() -> set_x(points[2].x());
            pad.mutable_parking_space_info() -> mutable_point_right_down() -> set_y(points[2].y());
            pad.mutable_parking_space_info() -> mutable_point_right_up() -> set_x(points[3].x());
            pad.mutable_parking_space_info() -> mutable_point_right_up() -> set_y(points[3].y());

            jmc_auto::common::PointENU start_point;
            std::string start_id = "";
            std::vector<jmc_auto::common::PointENU> start_points;
            start_point.set_x(start["x"]);
            start_point.set_y(start["y"]);
            if(map_service_ -> CheckPointInParkingSpace(start_point, start_id, start_points)){
              pad.set_appmode(AppMode::PARKING_TO_PARKING);
              pad.mutable_to_parking_space_info() -> set_id(start_id);
              pad.mutable_to_parking_space_info() -> mutable_point_left_up() -> set_x(start_points[0].x());
              pad.mutable_to_parking_space_info() -> mutable_point_left_up() -> set_y(start_points[0].y());
              pad.mutable_to_parking_space_info() -> mutable_point_left_down() -> set_x(start_points[1].x());
              pad.mutable_to_parking_space_info() -> mutable_point_left_down() -> set_y(start_points[1].y());
              pad.mutable_to_parking_space_info() -> mutable_point_right_down() -> set_x(start_points[2].x());
              pad.mutable_to_parking_space_info() -> mutable_point_right_down() -> set_y(start_points[2].y());
              pad.mutable_to_parking_space_info() -> mutable_point_right_up() -> set_x(start_points[3].x());
              pad.mutable_to_parking_space_info() -> mutable_point_right_up() -> set_y(start_points[3].y());
            }else{
              pad.set_appmode(AppMode::PARKING);
            }

        }
        else{
          websocket_->SendData(conn, response.dump());
          pad.set_appmode(AppMode::PARKING_NOID);
        }


            pad.mutable_point() -> mutable_start() -> set_x(start["x"]);
            pad.mutable_point() -> mutable_start() -> set_y(start["y"]);
            if(pad.appmode() == AppMode::PARKING_TO_PARKING){
              routing::LaneWaypoint laneWayPoint;
               if (map_service_->ConstructLaneWayPoint(end["x"], end["y"],
                                          &laneWayPoint, 1)){
                 pad.mutable_point() -> mutable_end() -> set_x(laneWayPoint.pose().x());
                  pad.mutable_point() -> mutable_end() -> set_y(laneWayPoint.pose().y());
                }else{
                  AERROR << "cannot locate end point on map";
                  pad.mutable_point() -> mutable_end() -> set_x(end["x"]);
                  pad.mutable_point() -> mutable_end() -> set_y(end["y"]);
                }
            }else{
              pad.mutable_point() -> mutable_end() -> set_x(end["x"]);
              pad.mutable_point() -> mutable_end() -> set_y(end["y"]);
            }
            // pad.mutable_point() -> mutable_end() -> set_x(end["x"]);
            // pad.mutable_point() -> mutable_end() -> set_y(end["y"]);
            //AWARN << "your ids is: " << ids.DebugString();
            //如果点在车位内，将ID和坐标位置传给pad消息；点不在车位内，不作处理。
            AINFO << "send parking Pad Msg:" << pad.DebugString();
            AdapterManager::FillPlanningPadHeader("HMI", &pad);
            AdapterManager::PublishPlanningPad(pad);
        return;
      });


  websocket_->RegisterMessageHandler(
      "app_SendParkingRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        AINFO << "i am in app_parking";
        auto start = json["start"];
        auto end = json["end"];
        auto parking_id = json["parking_id"];
        std::vector<jmc_auto::common::PointENU> points;
        planning::PadMessage pad;
        AINFO << "the parking_id is: " << parking_id;
        if(map_service_ -> getParkingspaceBy_id(parking_id, points)){
          //pad.set_appmode(AppMode::PARKING);
          pad.mutable_parking_space_info() -> set_id(parking_id);
          pad.mutable_parking_space_info() -> mutable_point_left_up() -> set_x(points[0].x());
          pad.mutable_parking_space_info() -> mutable_point_left_up() -> set_y(points[0].y());
          pad.mutable_parking_space_info() -> mutable_point_left_down() -> set_x(points[1].x());
          pad.mutable_parking_space_info() -> mutable_point_left_down() -> set_y(points[1].y());
          pad.mutable_parking_space_info() -> mutable_point_right_down() -> set_x(points[2].x());
          pad.mutable_parking_space_info() -> mutable_point_right_down() -> set_y(points[2].y());
          pad.mutable_parking_space_info() -> mutable_point_right_up() -> set_x(points[3].x());
          pad.mutable_parking_space_info() -> mutable_point_right_up() -> set_y(points[3].y());

          jmc_auto::common::PointENU start_point;
          std::string start_id = "";
          std::vector<jmc_auto::common::PointENU> start_points;
          start_point.set_x(start["x"]);
          start_point.set_y(start["y"]);
          if(map_service_ -> CheckPointInParkingSpace(start_point, start_id, start_points)){
            pad.set_appmode(AppMode::PARKING_TO_PARKING);
            pad.mutable_to_parking_space_info() -> set_id(start_id);
            pad.mutable_to_parking_space_info() -> mutable_point_left_up() -> set_x(start_points[0].x());
            pad.mutable_to_parking_space_info() -> mutable_point_left_up() -> set_y(start_points[0].y());
            pad.mutable_to_parking_space_info() -> mutable_point_left_down() -> set_x(start_points[1].x());
            pad.mutable_to_parking_space_info() -> mutable_point_left_down() -> set_y(start_points[1].y());
            pad.mutable_to_parking_space_info() -> mutable_point_right_down() -> set_x(start_points[2].x());
            pad.mutable_to_parking_space_info() -> mutable_point_right_down() -> set_y(start_points[2].y());
            pad.mutable_to_parking_space_info() -> mutable_point_right_up() -> set_x(start_points[3].x());
            pad.mutable_to_parking_space_info() -> mutable_point_right_up() -> set_y(start_points[3].y());
          }else{
            pad.set_appmode(AppMode::PARKING);
          }
          pad.mutable_point() -> mutable_start() -> set_x(start["x"]);
          pad.mutable_point() -> mutable_start() -> set_y(start["y"]);
          // pad.mutable_point() -> mutable_end() -> set_x(end["x"]);
          // pad.mutable_point() -> mutable_end() -> set_y(end["y"]);
          if(pad.appmode() == AppMode::PARKING_TO_PARKING){
            routing::LaneWaypoint laneWayPoint;
            if (map_service_->ConstructLaneWayPoint(end["x"], end["y"],
                                      &laneWayPoint, 1)){
              pad.mutable_point() -> mutable_end() -> set_x(laneWayPoint.pose().x());
              pad.mutable_point() -> mutable_end() -> set_y(laneWayPoint.pose().y());
            }else{
              AERROR << "cannot locate end point on map";
              pad.mutable_point() -> mutable_end() -> set_x(end["x"]);
              pad.mutable_point() -> mutable_end() -> set_y(end["y"]);
            }
          }else{
            pad.mutable_point() -> mutable_end() -> set_x(end["x"]);
            pad.mutable_point() -> mutable_end() -> set_y(end["y"]);
          }
          //AWARN << "your ids is: " << ids.DebugString();
          //如果点在车位内，将ID和坐标位置传给pad消息；点不在车位内，不作处理。
          AINFO << "send app_parking Pad Msg:" << pad.DebugString();
          AdapterManager::FillPlanningPadHeader("HMI", &pad);
          AdapterManager::PublishPlanningPad(pad);
        }
        else{
          AWARN << "the parking_id dose not exist!" << parking_id;
        }
        return;
  });

        //对召唤指令通过topic发送出去
  websocket_->RegisterMessageHandler(
      "SendParking_NOIDRequest",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto start = json["start"];
        auto end = json["end"];
        planning::PadMessage pad;
        pad.set_appmode(AppMode::PARKING_NOID);
        pad.mutable_point() -> mutable_start() -> set_x(start["x"]);
        pad.mutable_point() -> mutable_start() -> set_y(start["y"]);
        pad.mutable_point() -> mutable_end() -> set_x(end["x"]);
        pad.mutable_point() -> mutable_end() -> set_y(end["y"]);
        //AWARN << "your ids is: " << ids.DebugString();
        //如果点在车位内，将ID和坐标位置传给pad消息；点不在车位内，不作处理。
        AdapterManager::FillPlanningPadHeader("HMI", &pad);
        AdapterManager::PublishPlanningPad(pad);
        AINFO << "send Parking_NOID Pad Msg:" << pad.DebugString();
        return;
      });

  websocket_->RegisterMessageHandler(
      "RequestSimulationWorld",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        if (!sim_world_service_.ReadyToPush()) {
          AWARN_EVERY(100)
              << "Not sending simulation world as the data is not ready!";
          return;
        }

        bool enable_pnc_monitor = false;
        auto planning = json.find("planning");
        if (planning != json.end() && planning->is_boolean()) {
          enable_pnc_monitor = json["planning"];
        }
        std::string to_send;
        {
          // Pay the price to copy the data instead of sending data over the
          // wire while holding the lock.
          boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
          to_send = enable_pnc_monitor ? simulation_world_with_planning_data_
                                       : simulation_world_;
        }
        if (FLAGS_enable_update_size_check && !enable_pnc_monitor &&
            to_send.size() > FLAGS_max_update_size) {
          AWARN << "update size is too big:" << to_send.size();
          return;
        }
        //simulationWorld是以二进制数据发送给前端
        websocket_->SendBinaryData(conn, to_send, true);
      });

  websocket_->RegisterMessageHandler(
      "RequestRoutePath",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response = sim_world_service_.GetRoutePathAsJson();
        response["type"] = "RoutePath";
        //RoutePath是以Json格式发送给前端
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "GetDefaultEndPoint",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        Json response;
        response["type"] = "DefaultEndPoint";

        Json poi_list = Json::array();
        if (LoadPOI()) {
          for (const auto &landmark : poi_.landmark()) {
            Json place;
            place["name"] = landmark.name();
            Json waypoint_list;
            for (const auto &waypoint : landmark.waypoint()) {
              Json point;
              point["x"] = waypoint.pose().x();
              point["y"] = waypoint.pose().y();
              waypoint_list.push_back(point);
            }
            place["waypoint"] = waypoint_list;
            poi_list.push_back(place);
          }
        } else {
          sim_world_service_.PublishMonitorMessage(MonitorMessageItem::ERROR,
                                                   "Failed to load default "
                                                   "POI. Please make sure the "
                                                   "file exists at " +
                                                       EndWayPointFile());
        }
        response["poi"] = poi_list;
        websocket_->SendData(conn, response.dump());
      });

  websocket_->RegisterMessageHandler(
      "Reset", [this](const Json &json, WebSocketHandler::Connection *conn) {
        sim_world_service_.SetToClear();
        sim_control_->Reset();
      });

  websocket_->RegisterMessageHandler(
      "Dump", [this](const Json &json, WebSocketHandler::Connection *conn) {
        DumpMessage(AdapterManager::GetChassis(), "Chassis");
        DumpMessage(AdapterManager::GetPrediction(), "Prediction");
        DumpMessage(AdapterManager::GetRoutingRequest(), "RoutingRequest");
        DumpMessage(AdapterManager::GetRoutingResponse(), "RoutingResponse");
        DumpMessage(AdapterManager::GetLocalization(), "Localization");
        DumpMessage(AdapterManager::GetPlanning(), "Planning");
        DumpMessage(AdapterManager::GetControlCommand(), "Control");
        DumpMessage(AdapterManager::GetPerceptionObstacles(), "Perception");
        DumpMessage(AdapterManager::GetTrafficLightDetection(), "TrafficLight");
        //DumpMessage(AdapterManager::GetRelativeMap(), "RelativeMap");
        //DumpMessage(AdapterManager::GetNavigation(), "Navigation");
        DumpMessage(AdapterManager::GetContiRadar(), "ContiRadar");
        DumpMessage(AdapterManager::GetMobileye(), "Mobileye");
      });

  websocket_->RegisterMessageHandler(
      "ToggleSimControl",
      [this](const Json &json, WebSocketHandler::Connection *conn) {
        auto enable = json.find("enable");
        if (enable != json.end() && enable->is_boolean()) {
          if (*enable) {
            sim_control_->Start();
          } else {
            sim_control_->Stop();
          }
        }
      });
}

bool SimulationWorldUpdater::ConstructRoutingRequest(
    const Json &json, RoutingRequest *routing_request) {
  routing_request->clear_waypoint();
  // set start point
  if (!ContainsKey(json, "start")) {
    AERROR << "Failed to prepare a routing request: start point not found.";
    return false;
  }

  auto start = json["start"];
  if (!ValidateCoordinate(start)) {
    AERROR << "Failed to prepare a routing request: invalid start point.";
    return false;
  }
  if (!map_service_->ConstructLaneWayPoint(start["x"], start["y"],
                                           routing_request->add_waypoint(), 0)) {
    AERROR << "Failed to prepare a routing request:"
           << " cannot locate start point on map.";
    return false;
  }

  // set way point(s) if any
  auto iter = json.find("waypoint");
  if (iter != json.end() && iter->is_array()) {
    auto *waypoint = routing_request->mutable_waypoint();
    for (size_t i = 0; i < iter->size(); ++i) {
      auto &point = (*iter)[i];
      if (!ValidateCoordinate(point)) {
        AERROR << "Failed to prepare a routing request: invalid waypoint.";
        return false;
      }

      if (!map_service_->ConstructLaneWayPoint(point["x"], point["y"],
                                               waypoint->Add(), 0)) {
        waypoint->RemoveLast();
      }
    }
  }

  // set end point
  if (!ContainsKey(json, "end")) {
    AERROR << "Failed to prepare a routing request: end point not found.";
    return false;
  }

  auto end = json["end"];
  if (!ValidateCoordinate(end)) {
    AERROR << "Failed to prepare a routing request: invalid end point.";
    return false;
  }

  if(flag_appmode == 1){
    if (!map_service_->ConstructParkingLaneWayPoint(end["x"], end["y"],
                                            routing_request, flag_appmode)) {
      AERROR << "Failed to prepare a parking routing request:"
            << " cannot locate end point on map.";
      return false;
    }
  }
  else if (!map_service_->ConstructLaneWayPoint(end["x"], end["y"],
                                          routing_request->add_waypoint(), 0)) {
    AERROR << "Failed to prepare a routing request:"
          << " cannot locate end point on map.";
    return false;
  }


//  if (!map_service_->ConstructLaneWayPoint(end["x"], end["y"],
//                                          routing_request->add_waypoint(), flag_appmode)) {
//    AERROR << "Failed to prepare a routing request:"
//          << " cannot locate end point on map.";
//    return false;
//  }

  AINFO << "Constructed RoutingRequest to be sent:\n"
        << routing_request->DebugString();

  return true;
}

bool SimulationWorldUpdater::ValidateCoordinate(const nlohmann::json &json) {
  if (!ContainsKey(json, "x") || !ContainsKey(json, "y")) {
    AERROR << "Failed to find x or y coordinate.";
    return false;
  }
  if (json.find("x")->is_number() && json.find("y")->is_number()) {
    return true;
  }
  AERROR << "Both x and y coordinate should be a number.";
  return false;
}

void SimulationWorldUpdater::Start() {
  // start ROS timer, one-shot = false, auto-start = true
  timer_ =
      AdapterManager::CreateTimer(ros::Duration(kSimWorldTimeIntervalMs / 1000),
                                  &SimulationWorldUpdater::OnTimer, this);
}

void SimulationWorldUpdater::OnTimer(const ros::TimerEvent &event) {
  sim_world_service_.Update();

  {
    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
    sim_world_service_.GetWireFormatString(
        FLAGS_sim_map_radius, &simulation_world_,
        &simulation_world_with_planning_data_);
    //sim_world_service_.GetRelativeMap().SerializeToString(
    //    &relative_map_string_);
  }
}

bool SimulationWorldUpdater::LoadPOI() {
  if (GetProtoFromASCIIFile(EndWayPointFile(), &poi_)) {
    return true;
  }

  AWARN << "Failed to load default list of POI from " << EndWayPointFile();
  return false;
}

}  // namespace dreamview
}  // namespace jmc_auto
