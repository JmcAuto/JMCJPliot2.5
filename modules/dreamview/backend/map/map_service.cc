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

#include "modules/dreamview/backend/map/map_service.h"

#include <algorithm>
#include <fstream>

#include "modules/common/util/json_util.h"
#include "modules/common/util/string_util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/common/proto/geometry.pb.h"

namespace jmc_auto {
namespace dreamview {

using jmc_auto::common::PointENU;
using jmc_auto::common::util::JsonUtil;
using jmc_auto::hdmap::ClearAreaInfoConstPtr;
using jmc_auto::hdmap::CrosswalkInfoConstPtr;
using jmc_auto::hdmap::ParkingSpaceInfoConstPtr;
using jmc_auto::hdmap::HDMapUtil;
using jmc_auto::hdmap::Id;
using jmc_auto::hdmap::JunctionInfoConstPtr;
using jmc_auto::hdmap::LaneInfoConstPtr;
using jmc_auto::hdmap::Map;
using jmc_auto::hdmap::Path;
using jmc_auto::hdmap::PncMap;
using jmc_auto::hdmap::RoadInfoConstPtr;
using jmc_auto::hdmap::RouteSegments;
using jmc_auto::hdmap::SignalInfoConstPtr;
using jmc_auto::hdmap::SimMapFile;
using jmc_auto::hdmap::StopSignInfoConstPtr;
using jmc_auto::hdmap::YieldSignInfoConstPtr;
using jmc_auto::routing::RoutingRequest;
using jmc_auto::routing::RoutingResponse;
using google::protobuf::RepeatedPtrField;
//using jmc_auto::control::PadMessage;

namespace {
//RepeatedPtrField为protobuf repeated类型
template <typename MapElementInfoConstPtr>
void ExtractIds(const std::vector<MapElementInfoConstPtr> &items,
                RepeatedPtrField<std::string> *ids) {
  //为容器预留空间
  ids->Reserve(items.size());
  for (const auto &item : items) {
    ids->Add()->assign(item->id().id());
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(ids->begin(), ids->end());
}

void ExtractOverlapIds(const std::vector<SignalInfoConstPtr> &items,
                       RepeatedPtrField<std::string> *ids) {
  for (const auto &item : items) {
    for (auto &overlap_id : item->signal().overlap_id()) {
      ids->Add()->assign(overlap_id.id());
    }
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(ids->begin(), ids->end());
}

void ExtractOverlapIds(const std::vector<StopSignInfoConstPtr> &items,
                       RepeatedPtrField<std::string> *ids) {
  for (const auto &item : items) {
    for (auto &overlap_id : item->stop_sign().overlap_id()) {
      ids->Add()->assign(overlap_id.id());
    }
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(ids->begin(), ids->end());
}

void ExtractRoadAndLaneIds(const std::vector<LaneInfoConstPtr> &lanes,
                           RepeatedPtrField<std::string> *lane_ids,
                           RepeatedPtrField<std::string> *road_ids) {
  lane_ids->Reserve(lanes.size());
  road_ids->Reserve(lanes.size());

  for (const auto &lane : lanes) {
    lane_ids->Add()->assign(lane->id().id());
    if (!lane->road_id().id().empty()) {
      road_ids->Add()->assign(lane->road_id().id());
    }
  }
  // The output is sorted so that the calculated hash will be
  // invariant to the order of elements.
  std::sort(lane_ids->begin(), lane_ids->end());
  std::sort(road_ids->begin(), road_ids->end());
}

}  // namespace

const char MapService::kMetaFileName[] = "/metaInfo.json";

MapService::MapService(bool use_sim_map) : use_sim_map_(use_sim_map) {
  ReloadMap(false);
}

bool MapService::ReloadMap(bool force_reload) {
  boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
  bool ret = true;
  if (force_reload) {
    ret = HDMapUtil::ReloadMaps();
  }

  // Update the x,y-offsets if present.
  UpdateOffsets();
  return ret;
}

void MapService::UpdateOffsets() {
  x_offset_ = 0.0;
  y_offset_ = 0.0;
  std::ifstream ifs(FLAGS_map_dir + kMetaFileName);
  if (!ifs.is_open()) {
    AINFO << "Failed to open map meta file: " << kMetaFileName;
  } else {
    nlohmann::json json;
    ifs >> json;
    ifs.close();

    for (auto it = json.begin(); it != json.end(); ++it) {
      auto val = it.value();
      if (val.is_object()) {
        auto x_offset = val.find("xoffset");
        if (x_offset == val.end()) {
          AWARN << "Cannot find x_offset for this map " << it.key();
          continue;
        }

        if (!x_offset->is_number()) {
          AWARN << "Expect x_offset with type 'number', but was "
                << x_offset->type_name();
          continue;
        }
        x_offset_ = x_offset.value();

        auto y_offset = val.find("yoffset");
        if (y_offset == val.end()) {
          AWARN << "Cannot find y_offset for this map " << it.key();
          continue;
        }

        if (!y_offset->is_number()) {
          AWARN << "Expect y_offset with type 'number', but was "
                << y_offset->type_name();
          continue;
        }
        y_offset_ = y_offset.value();
      }
    }
  }
  AINFO << "Updated with map: x_offset " << x_offset_ << ", y_offset "
        << y_offset_;
}

const hdmap::HDMap *MapService::HDMap() const {
  return HDMapUtil::BaseMapPtr();
}

const hdmap::HDMap *MapService::SimMap() const {
  return use_sim_map_ ? HDMapUtil::SimMapPtr() : HDMapUtil::BaseMapPtr();
}

bool MapService::MapReady() const { return HDMap() && SimMap(); }

void MapService::CollectMapElementIds(const PointENU &point, double radius,
                                      MapElementIds *ids) const {
  if (!MapReady()) {
    return;
  }
  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  std::vector<LaneInfoConstPtr> lanes;
  if (SimMap()->GetLanes(point, radius, &lanes) != 0) {
    AERROR << "Fail to get lanes from sim_map.";
  }
  ExtractRoadAndLaneIds(lanes, ids->mutable_lane(), ids->mutable_road());

  std::vector<ClearAreaInfoConstPtr> clear_areas;
  if (SimMap()->GetClearAreas(point, radius, &clear_areas) != 0) {
    AERROR << "Fail to get clear areas from sim_map.";
  }
  ExtractIds(clear_areas, ids->mutable_clear_area());

  std::vector<CrosswalkInfoConstPtr> crosswalks;
  if (SimMap()->GetCrosswalks(point, radius, &crosswalks) != 0) {
    AERROR << "Fail to get crosswalks from sim_map.";
  }
  ExtractIds(crosswalks, ids->mutable_crosswalk());

  std::vector<JunctionInfoConstPtr> junctions;
  if (SimMap()->GetJunctions(point, radius, &junctions) != 0) {
    AERROR << "Fail to get junctions from sim_map.";
  }
  ExtractIds(junctions, ids->mutable_junction());

  std::vector<SignalInfoConstPtr> signals;
  if (SimMap()->GetSignals(point, radius, &signals) != 0) {
    AERROR << "Failed to get signals from sim_map.";
  }

  ExtractIds(signals, ids->mutable_signal());
  ExtractOverlapIds(signals, ids->mutable_overlap());

  std::vector<StopSignInfoConstPtr> stop_signs;
  if (SimMap()->GetStopSigns(point, radius, &stop_signs) != 0) {
    AERROR << "Failed to get stop signs from sim_map.";
  }
  ExtractIds(stop_signs, ids->mutable_stop_sign());
  ExtractOverlapIds(stop_signs, ids->mutable_overlap());

  std::vector<YieldSignInfoConstPtr> yield_signs;
  if (SimMap()->GetYieldSigns(point, radius, &yield_signs) != 0) {
    AERROR << "Failed to get yield signs from sim_map.";
  }
  ExtractIds(yield_signs, ids->mutable_yield());

  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  if (SimMap()->GetParkingSpaces(point, radius, &parking_spaces) != 0) {
    AERROR << "Failed to get parking spaces from sim_map.";
  }
  //将找到的对应元素的id取出来
  ExtractIds(parking_spaces, ids->mutable_parking_space());
  //AWARN << "The parking_spaces is: " << parking_spaces;
}

Map MapService::RetrieveMapElements(const MapElementIds &ids) const {
  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  Map result;
  if (!MapReady()) {
    return result;
  }
  Id map_id;

  for (const auto &id : ids.lane()) {
    map_id.set_id(id);
    auto element = SimMap()->GetLaneById(map_id);
    if (element) {
      auto lane = element->lane();
      lane.clear_left_sample();
      lane.clear_right_sample();
      lane.clear_left_road_sample();
      lane.clear_right_road_sample();
      *result.add_lane() = lane;
    }
  }

  for (const auto &id : ids.clear_area()) {
    map_id.set_id(id);
    auto element = SimMap()->GetClearAreaById(map_id);
    if (element) {
      *result.add_clear_area() = element->clear_area();
    }
  }

  for (const auto &id : ids.crosswalk()) {
    map_id.set_id(id);
    auto element = SimMap()->GetCrosswalkById(map_id);
    if (element) {
      *result.add_crosswalk() = element->crosswalk();
    }
  }

  for (const auto &id : ids.junction()) {
    map_id.set_id(id);
    auto element = SimMap()->GetJunctionById(map_id);
    if (element) {
      *result.add_junction() = element->junction();
    }
  }

  for (const auto &id : ids.signal()) {
    map_id.set_id(id);
    auto element = SimMap()->GetSignalById(map_id);
    if (element) {
      *result.add_signal() = element->signal();
    }
  }

  for (const auto &id : ids.stop_sign()) {
    map_id.set_id(id);
    auto element = SimMap()->GetStopSignById(map_id);
    if (element) {
      *result.add_stop_sign() = element->stop_sign();
    }
  }

  for (const auto &id : ids.yield()) {
    map_id.set_id(id);
    auto element = SimMap()->GetYieldSignById(map_id);
    if (element) {
      *result.add_yield() = element->yield_sign();
    }
  }

  for (const auto &id : ids.road()) {
    map_id.set_id(id);
    auto element = SimMap()->GetRoadById(map_id);
    if (element) {
      *result.add_road() = element->road();
    }
  }

  for (const auto &id : ids.overlap()) {
    map_id.set_id(id);
    auto element = SimMap()->GetOverlapById(map_id);
    if (element) {
      *result.add_overlap() = element->overlap();
    }
  }

  for (const auto &id : ids.parking_space()) {
    map_id.set_id(id);
    auto element = SimMap()->GetParkingSpaceById(map_id);
    if (element) {
      *result.add_parking_space() = element->parking_space();
    }
  }

  return result;
}

bool MapService::GetNearestLane(const double x, const double y,
                                LaneInfoConstPtr *nearest_lane,
                                double *nearest_s, double *nearest_l) const {
  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  PointENU point;
  point.set_x(x);
  point.set_y(y);
  if (!MapReady() ||
      HDMap()->GetNearestLane(point, nearest_lane, nearest_s, nearest_l) < 0) {
    AERROR << "Failed to get nearest lane!";
    return false;
  }
  return true;
}

bool MapService::GetPathsFromRouting(const RoutingResponse &routing,
                                     std::vector<Path> *paths) const {
  if (!CreatePathsFromRouting(routing, paths)) {
    AERROR << "Unable to get paths from routing!";
    return false;
  }
  return true;
}

bool MapService::GetPoseWithRegardToLane(const double x, const double y,
                                         double *theta, double *s) const {
  double l;
  LaneInfoConstPtr nearest_lane;
  if (!GetNearestLane(x, y, &nearest_lane, s, &l)) {
    return false;
  }

  *theta = nearest_lane->Heading(*s);
  return true;
}

/*bool MapService::ConstructLaneWayPoint(
    const double x, const double y, routing::LaneWaypoint *laneWayPoint, const short int flag_appmode) const {
  double s, l;
  LaneInfoConstPtr lane;
  PointENU summon_point;
  if (!GetNearestLane(x, y, &lane, &s, &l)) {
    return false;
  }
  laneWayPoint->set_id(lane->id().id());
  auto *pose = laneWayPoint->mutable_pose();
  if (flag_appmode == 1){
      s = s + 10 >= lane -> total_length() ? lane -> total_length():s + 10;
      summon_point = lane -> GetSmoothPoint(s);
      laneWayPoint->set_s(s);
      pose -> CopyFrom(summon_point);
  }
  else {
    laneWayPoint->set_s(s);
    pose->set_x(x);
    pose->set_y(y);
  }
  AINFO << "the flag_appmode is: " << flag_appmode;
  return true;
}
*/

bool MapService::ConstructParkingLaneWayPoint(
    const double x, const double y, jmc_auto::routing::RoutingRequest *routing_request, const short int flag_appmode) const {
  double s, l;
  LaneInfoConstPtr lane;
  PointENU summon_point;
  //summon_point.set_x(x);
  //summon_point.set_y(y);
  ADEBUG << "Routing x:" << x << " y:" << y;
  //std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  //auto statu = SimMap()->GetParkingSpaces(summon_point, 0.1, &parking_spaces);
 // if(statu != 0){
 //   ADEBUG << "Find parking failed";
 //   return false;
 // }
 // double parking_cx = (parking_spaces[0]->polygon().points()[0].x() + parking_spaces[0]->polygon().points()[3].x())/2;
 // double parking_cy = (parking_spaces[0]->polygon().points()[0].y() + parking_spaces[0]->polygon().points()[3].y())/2;
  auto* laneWayPoint = routing_request->add_waypoint();
  if (!GetNearestLane(x, y, &lane, &s, &l)) {
      return false;
  }
  if(lane->lane().turn() != 1){
    ADEBUG << "Find No_turn lane";
    std::vector<hdmap::LaneInfoConstPtr> lanes;
    summon_point.set_x(x);
    summon_point.set_y(y);
    if(HDMap()->GetLanes(summon_point, 3, &lanes) == 0){
      for(const auto lane_ptr : lanes){
        if(lane_ptr->lane().turn() == 1){
          ADEBUG << "has no_turn lane";
          lane = lane_ptr;
        //  jmc_auto::common::math::Vec2d point(x, y);
          if(!lane->GetProjection({x, y}, &s, &l)){
             AERROR << "projection failed";
          }
          break;
        }
      }
    }
  }
  laneWayPoint->set_id(lane->id().id());
  auto *pose = laneWayPoint->mutable_pose();
  laneWayPoint->set_s(s);
  pose->set_x(x);
  pose->set_y(y);
  auto* laneWayEndPoint = routing_request->add_waypoint();
  auto *pose_end = laneWayEndPoint->mutable_pose();
  if (flag_appmode == 1){
      s = s + 12;
//NEW ADD
      int left_turn = -1;
      if(s >= lane -> total_length() && lane->lane().successor_id_size() != 0){
        LaneInfoConstPtr next_lane;
        for (int i = 0; i < lane->lane().successor_id_size(); ++i){
          auto next_i_lane_id = lane -> lane().successor_id(i);
          next_lane = HDMap()->GetLaneById(next_i_lane_id);
          if(next_lane -> lane().turn() == 1){ //直行
            break;
          }
          if(next_lane -> lane().turn() == 2) left_turn = i;
        }

      if(next_lane -> lane().turn() != 1 && next_lane->lane().successor_id_size() != 0){
        if(left_turn != -1){
           next_lane = HDMap()->GetLaneById(lane -> lane().successor_id(left_turn));
        }
        auto next_l_lane_id = next_lane -> lane().successor_id(0);
        next_lane = HDMap()->GetLaneById(next_l_lane_id);
      }
      //if(s >= lane -> total_length()){
       // auto next_lane_id = lane -> lane().successor_id(0);
       // auto next_lane = HDMap()->GetLaneById(next_lane_id);
        summon_point = next_lane->GetSmoothPoint(5);
        AINFO << "NEXT LANE PARKING";
        laneWayEndPoint->set_id(next_lane->id().id());

      }else{
        summon_point = lane -> GetSmoothPoint(s);
        laneWayEndPoint->set_id(lane->id().id());
      }
      laneWayEndPoint->set_s(s);
      pose_end -> CopyFrom(summon_point);
  }
  AINFO << "the flag_appmode is: " << flag_appmode;
  return true;
}

bool MapService::ConstructLaneWayPoint(
    const double x, const double y, routing::LaneWaypoint *laneWayPoint, const short int flag_appmode) const {
  double s, l;
  LaneInfoConstPtr lane;
  PointENU summon_point;
  if (!GetNearestLane(x, y, &lane, &s, &l)) {
    return false;
  }
  //laneWayPoint->set_id(lane->id().id());
  auto *pose = laneWayPoint->mutable_pose();
  if (flag_appmode == 1){
      s = s + 10;
      if(s >= lane -> total_length() && lane->lane().successor_id_size() != 0){
        auto next_lane_id = lane -> lane().successor_id(0);
        auto next_lane = HDMap()->GetLaneById(next_lane_id);
        summon_point = next_lane->GetSmoothPoint(5);
        AINFO << "NEXT LANE PARKING";
        laneWayPoint->set_id(next_lane->id().id());
      }else{
        summon_point = lane -> GetSmoothPoint(s);
        laneWayPoint->set_id(lane->id().id());
      }
      laneWayPoint->set_s(s);
      pose -> CopyFrom(summon_point);
  }
  else {
    laneWayPoint->set_s(s);
    pose->set_x(x);
    pose->set_y(y);
  }
  AINFO << "the flag_appmode is: " << flag_appmode;
  return true;
}

bool MapService::GetStartPoint(jmc_auto::common::PointENU *start_point) const {
  // Start from origin to find a lane from the map.
  double s, l;
  LaneInfoConstPtr lane;
  if (!GetNearestLane(0.0, 0.0, &lane, &s, &l)) {
    return false;
  }

  *start_point = lane->GetSmoothPoint(0.0);
  return true;
}

bool MapService::CreatePathsFromRouting(const RoutingResponse &routing,
                                        std::vector<Path> *paths) const {
  for (const auto &road : routing.road()) {
    for (const auto &passage_region : road.passage()) {
      // Each passage region in a road forms a path
      // 每一个在路上的过道区域组成路线
      if (!AddPathFromPassageRegion(passage_region, paths)) {
        return false;
      }
    }
  }
  return true;
}

bool MapService::AddPathFromPassageRegion(
    const routing::Passage &passage_region, std::vector<Path> *paths) const {
  if (!MapReady()) {
    return false;
  }
  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  RouteSegments segments;
  for (const auto &segment : passage_region.segment()) {
    auto lane_ptr = HDMap()->GetLaneById(hdmap::MakeMapId(segment.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << segment.id();
      return false;
    }
    segments.emplace_back(lane_ptr, segment.start_s(), segment.end_s());
  }

  paths->emplace_back();
  if (!PncMap::CreatePathFromLaneSegments(segments, &paths->back())) {
    return false;
  }

  return true;
}

size_t MapService::CalculateMapHash(const MapElementIds &ids) const {
  static std::hash<std::string> hash_function;
  return hash_function(ids.DebugString());
}

//判断点是否在凸四边形内部
//采用向量叉积法，同向为正
bool isPointInRect(jmc_auto::common::PointENU &point0, jmc_auto::common::PointENU &point1, jmc_auto::common::PointENU &point2, jmc_auto::common::PointENU &point3, jmc_auto::common::PointENU &point4){
  double a, b, c, d;//c存储四个向量的计算结果
  a = (point2.x() - point1.x()) * (point0.y() - point1.y()) - (point2.y() - point1.y()) * (point0.x() - point1.x());
  b = (point3.x() - point2.x()) * (point0.y() - point2.y()) - (point3.y() - point2.y()) * (point0.x() - point2.x());
  c = (point4.x() - point3.x()) * (point0.y() - point3.y()) - (point4.y() - point3.y()) * (point0.x() - point3.x());
  d = (point1.x() - point4.x()) * (point0.y() - point4.y()) - (point1.y() - point4.y()) * (point0.x() - point4.x());
  if ((a > 0 && b > 0 && c > 0 && d > 0) || (a < 0 && b < 0 && c < 0 && d < 0)){
    return true;
  }
  return false;
}

bool MapService::getParkingspaceBy_id(const std::string &id, std::vector<jmc_auto::common::PointENU> &points) const{
  Id map_id;
  map_id.set_id(id);
  auto element = SimMap()->GetParkingSpaceById(map_id);
  jmc_auto::common::PointENU point_left_down;
  jmc_auto::common::PointENU point_right_down;
  jmc_auto::common::PointENU point_right_up;
  jmc_auto::common::PointENU point_left_up;
  if (element) {
    point_left_down.set_x(element -> polygon().points()[1].x());
    point_left_down.set_y(element -> polygon().points()[1].y());
    point_right_down.set_x(element -> polygon().points()[2].x());
    point_right_down.set_y(element -> polygon().points()[2].y());
    point_right_up.set_x(element -> polygon().points()[3].x());
    point_right_up.set_y(element -> polygon().points()[3].y());
    point_left_up.set_x(element -> polygon().points()[0].x());
    point_left_up.set_y(element -> polygon().points()[0].y());
    points.emplace_back(point_left_up);
    points.emplace_back(point_left_down);
    points.emplace_back(point_right_down);
    points.emplace_back(point_right_up);
    return true;
  }
  return false;
}

//点在车位内，将车位信息输出
bool MapService::CheckPointInParkingSpace(jmc_auto::common::PointENU &point, std::string &id, std::vector<jmc_auto::common::PointENU> &points) const{

  boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);

  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  SimMap()->GetParkingSpaces(point, 1, &parking_spaces);
  if (parking_spaces.empty()){
    AINFO << "point is not in!";
    return false;
  }
  for (auto parking_space : parking_spaces){
    AINFO << "point is in" << parking_space -> parking_space().DebugString();
    jmc_auto::common::PointENU point_left_down;
    jmc_auto::common::PointENU point_right_down;
    jmc_auto::common::PointENU point_right_up;
    jmc_auto::common::PointENU point_left_up;
    point_left_down.set_x(parking_space -> polygon().points()[1].x());
    point_left_down.set_y(parking_space -> polygon().points()[1].y());
    point_right_down.set_x(parking_space -> polygon().points()[2].x());
    point_right_down.set_y(parking_space -> polygon().points()[2].y());
    point_right_up.set_x(parking_space -> polygon().points()[3].x());
    point_right_up.set_y(parking_space -> polygon().points()[3].y());
    point_left_up.set_x(parking_space -> polygon().points()[0].x());
    point_left_up.set_y(parking_space -> polygon().points()[0].y());
    if(isPointInRect(point, point_left_down, point_right_down, point_right_up, point_left_up)){
      points.emplace_back(point_left_up);
      points.emplace_back(point_left_down);
      points.emplace_back(point_right_down);
      points.emplace_back(point_right_up);
      id = parking_space -> id().id();
      return true;
    }
  }
  return false;
}

}  // namespace dreamview
}  // namespace jmc_auto
