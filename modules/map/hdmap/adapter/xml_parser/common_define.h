/* Copyright 2017 The JmcAuto Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/
#ifndef MODULES_MAP_HDMAP_ADAPTER_XML_PARSER_COMMON_DEFINE_H_
#define MODULES_MAP_HDMAP_ADAPTER_XML_PARSER_COMMON_DEFINE_H_

#include <string>
#include <unordered_set>
#include <vector>
#include "modules/common/log.h"
#include "modules/map/proto/map.pb.h"

namespace jmc_auto {
namespace hdmap {
namespace adapter {

using PbHeader = jmc_auto::hdmap::Header;
using PbRoad = jmc_auto::hdmap::Road;
using PbRoadSection = jmc_auto::hdmap::RoadSection;
using PbLane = jmc_auto::hdmap::Lane;
using PbJunction = jmc_auto::hdmap::Junction;
using PbSignal = jmc_auto::hdmap::Signal;
using PbSubSignal = jmc_auto::hdmap::Subsignal;
using PbCrosswalk = jmc_auto::hdmap::Crosswalk;
using PbParkingSpace = jmc_auto::hdmap::ParkingSpace;
using PbSpeedBump = jmc_auto::hdmap::SpeedBump;
using PbStopSign = jmc_auto::hdmap::StopSign;
using PbYieldSign = jmc_auto::hdmap::YieldSign;
using PbObjectOverlapInfo = jmc_auto::hdmap::ObjectOverlapInfo;
using PbOverlap = jmc_auto::hdmap::Overlap;
using PbClearArea = jmc_auto::hdmap::ClearArea;
using PbLineSegment = jmc_auto::hdmap::LineSegment;
using PbCurveSegment = jmc_auto::hdmap::CurveSegment;
using PbCurve = jmc_auto::hdmap::Curve;
using PbPoint3D = jmc_auto::common::PointENU;
using PbLaneType = jmc_auto::hdmap::Lane_LaneType;
using PbTurnType = jmc_auto::hdmap::Lane_LaneTurn;
using PbID = jmc_auto::hdmap::Id;
using PbLaneBoundary = jmc_auto::hdmap::LaneBoundary;
using PbLaneBoundaryTypeType = jmc_auto::hdmap::LaneBoundaryType_Type;
using PbPolygon = jmc_auto::hdmap::Polygon;
using PbBoundaryPolygon = jmc_auto::hdmap::BoundaryPolygon;
using PbBoundaryEdge = jmc_auto::hdmap::BoundaryEdge;

using PbLaneDirection = jmc_auto::hdmap::Lane_LaneDirection;
using PbSignalType = jmc_auto::hdmap::Signal_Type;
using PbSubSignalType = jmc_auto::hdmap::Subsignal_Type;
using PbBoundaryEdgeType = jmc_auto::hdmap::BoundaryEdge_Type;

struct StopLineInternal {
  std::string id;
  PbCurve curve;
};

struct StopSignInternal {
  std::string id;
  PbStopSign stop_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct YieldSignInternal {
  std::string id;
  PbYieldSign yield_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct TrafficLightInternal {
  std::string id;
  PbSignal traffic_light;
  std::unordered_set<std::string> stop_line_ids;
};

struct OverlapWithLane {
  std::string object_id;
  double start_s;
  double end_s;
  bool is_merge;

  OverlapWithLane() : is_merge(false) {}
};

struct OverlapWithJunction {
  std::string object_id;
};

struct LaneInternal {
  PbLane lane;
  std::vector<OverlapWithLane> overlap_signals;
  std::vector<OverlapWithLane> overlap_objects;
  std::vector<OverlapWithLane> overlap_junctions;
  std::vector<OverlapWithLane> overlap_lanes;
};

struct JunctionInternal {
  PbJunction junction;
  std::unordered_set<std::string> road_ids;
  std::vector<OverlapWithJunction> overlap_with_junctions;
};

struct RoadSectionInternal {
  std::string id;
  PbRoadSection section;
  std::vector<LaneInternal> lanes;
};

struct RoadInternal {
  std::string id;
  PbRoad road;

  bool in_junction;
  std::string junction_id;

  std::vector<RoadSectionInternal> sections;

  std::vector<TrafficLightInternal> traffic_lights;
  std::vector<StopSignInternal> stop_signs;
  std::vector<YieldSignInternal> yield_signs;
  std::vector<PbCrosswalk> crosswalks;
  std::vector<PbClearArea> clear_areas;
  std::vector<PbSpeedBump> speed_bumps;
  std::vector<StopLineInternal> stop_lines;
  std::vector<PbParkingSpace> parking_spaces;

  RoadInternal() : in_junction(false) { junction_id = ""; }
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace jmc_auto

#endif  // MODULES_MAP_HDMAP_ADAPTER_XML_PARSER_COMMON_DEFINE_H_
