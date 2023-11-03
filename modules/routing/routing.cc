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
#include <algorithm>

#include "modules/routing/routing.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/core/navigator.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/angle.h"

namespace jmc_auto {
namespace routing {

using jmc_auto::common::adapter::AdapterManager;
using jmc_auto::common::monitor::MonitorMessageItem;
using jmc_auto::common::ErrorCode;

std::string Routing::Name() const { return FLAGS_routing_node_name; }

Routing::Routing()
    : monitor_logger_(jmc_auto::common::monitor::MonitorMessageItem::ROUTING) {}

jmc_auto::common::Status Routing::Init() {
  const auto routing_map_file = jmc_auto::hdmap::RoutingMapFile();
  AINFO << "Use routing topology graph path: " << routing_map_file;
  navigator_ptr_.reset(new Navigator(routing_map_file));
  CHECK(common::util::GetProtoFromFile(FLAGS_routing_conf_file, &routing_conf_))
      << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

  hdmap_ = jmc_auto::hdmap::HDMapUtil::BaseMapPtr();
  CHECK(hdmap_) << "Failed to load map file:" << jmc_auto::hdmap::BaseMapFile();

  AdapterManager::Init(FLAGS_routing_adapter_config_filename);
  AdapterManager::AddRoutingRequestCallback(&Routing::OnRoutingRequest, this);
  return jmc_auto::common::Status::OK();
}

jmc_auto::common::Status Routing::Start() {
  if (!navigator_ptr_->IsReady()) {
    AERROR << "Navigator is not ready!";
    return jmc_auto::common::Status(ErrorCode::ROUTING_ERROR,
                                  "Navigator not ready");
  }
  AINFO << "Routing service is ready.";

  jmc_auto::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("Routing started");
  return jmc_auto::common::Status::OK();
}

RoutingRequest Routing::FillLaneInfoIfMissing(
    const RoutingRequest& routing_request) {
  RoutingRequest fixed_request(routing_request);
  std::vector<LaneWaypoint> start_waypoints;
  ADEBUG << "request waypoint size : " << routing_request.waypoint_size();
  for (int i = 0; i < routing_request.waypoint_size(); ++i) {
    const auto& lane_waypoint = routing_request.waypoint(i);
    
    if (lane_waypoint.has_id()) {
      ADEBUG << "point has id. deal with start point.";
      if((start_waypoints.size() > 0) && !fixed_request.waypoint(0).has_id())
      {
        auto waypoint_info = fixed_request.mutable_waypoint(0);
        auto start_waypoint = start_waypoints[0];
        waypoint_info->mutable_pose()->CopyFrom(start_waypoint.pose());
        waypoint_info->set_id(start_waypoint.id());
        waypoint_info->set_s(start_waypoint.s());
        ADEBUG << "waypoint has id : start point, lane id : " << start_waypoint.id() << ", s = " << start_waypoint.s() << ", start point : " << start_waypoint.pose().DebugString();
      }else
      {
        ADEBUG << "waypoint has id, start waypoint size = 0 or deal with start has done.";
      }
      continue;
    }

    ADEBUG << "search point.";
    auto point = common::util::MakePointENU(lane_waypoint.pose().x(),
                                            lane_waypoint.pose().y(),
                                            lane_waypoint.pose().z());

    // FIXME(all): select one reasonable lane candidate for point=>lane
    // is one to many relationship.
    if(0 == i)
    {
      double start_s = 0.0;
      double start_l = 0.0;
      hdmap::LaneInfoConstPtr start_lane;
      std::vector<hdmap::LaneInfoConstPtr> start_lanes;
      AdapterManager::Observe();
      auto localization_adapter = AdapterManager::GetLocalization();
      if(localization_adapter->Empty())
      {	
        AINFO << "routing : localization is not ready.";
        return routing_request;
      }	
      const auto localization_estimate = localization_adapter->GetLatestObserved();
      auto vehicle_heading = localization_estimate.pose().heading();
      ADEBUG << "routing : start point : FillLaneInfoIfMissing() : Vehicle State Heading : " << vehicle_heading;

      if (hdmap_->GetLanesWithHeading(point, 5.0, vehicle_heading, M_PI / 3.0, &start_lanes) != 0) {
        ADEBUG << "get lane with heading error.";
        AERROR << "Failed to find lane with heading from map at position: "
              << point.DebugString() << " ,heading : " << vehicle_heading;
        //return routing_request;
      }

      if(start_lanes.size() > 1)
      {	
	      ADEBUG << "start point search, vailid lanes size : " << start_lanes.size();
        start_waypoints = GetProperWaypoints(start_lanes, point, vehicle_heading);
        ADEBUG << "GetProperWaypoints size : " <<  start_waypoints.size();
        if(0 == start_waypoints.size())
        {
          ADEBUG << "GetProperWaypoints: waypoint is null!, no lanes avg heading less than 30.";
          if(hdmap_->GetNearestLaneWithHeading(point, 10.0, vehicle_heading, M_PI / 1.5, &start_lane, &start_s, &start_l) != 0)
          {
            ADEBUG << "get nearest lane with heading error.";
            AERROR << "Failed to find nearest lane heading from map at position: "
                  << point.DebugString() << " ,heading : " << vehicle_heading;
            return routing_request;
          }
          auto start_point_img_enu = start_lane->GetSmoothPoint(start_s);
          LaneWaypoint wp;
          wp.set_id(start_lane->id().id());
          wp.set_s(start_s);
          wp.mutable_pose()->CopyFrom(start_point_img_enu);
          start_waypoints.push_back(wp);
        }
        ADEBUG << "actual start point : " << point.DebugString();
        //在终点处根据routing线长度确定起点
	     /* auto waypoint_info = fixed_request.mutable_waypoint(i);
        waypoint_info->mutable_pose()->CopyFrom(waypoint.pose());
        waypoint_info->set_id(waypoint.id());
        waypoint_info->set_s(waypoint.s());
        ADEBUG << "routing : start point : " << point.DebugString() << ", lane id : " << waypoint.id() << ", s = " << s << ", l = " << l;
      */
      }else{
        ADEBUG << "lanes size = " << start_lanes.size();
        if(hdmap_->GetNearestLaneWithHeading(point, 5.0, vehicle_heading, M_PI / 3.0, &start_lane, &start_s, &start_l) != 0)
        {
          if(hdmap_->GetNearestLaneWithHeading(point, 10.0, vehicle_heading, M_PI / 1.5, &start_lane, &start_s, &start_l) != 0)
          {
            ADEBUG << "get nearest lane with heading error.";
            AERROR << "Failed to find nearest lane heading from map at position: "
                  << point.DebugString() << " ,heading : " << vehicle_heading;
          }
        }
          auto start_point_img_enu = start_lane->GetSmoothPoint(start_s);
          LaneWaypoint wp;
          wp.set_id(start_lane->id().id());
          wp.set_s(start_s);
          wp.mutable_pose()->CopyFrom(start_point_img_enu);
          start_waypoints.push_back(wp);
       //request 的起始waypoint改为在终点处赋值
       /* auto waypoint_info = fixed_request.mutable_waypoint(i);
          waypoint_info->mutable_pose()->CopyFrom(start_point_img_enu);
          waypoint_info->set_id(lane->id().id());
          waypoint_info->set_s(s);
       */   
          ADEBUG << "actual start point : " << point.DebugString() << ", "
                << "imagination start point : " << start_point_img_enu.DebugString();
          ADEBUG << "routing : start point : " << point.DebugString() << ", lane id : " << start_lane->id().id() << ", s = " << start_s << ", l = " << start_l;
      }
    }else if((routing_request.waypoint_size() - 1) == i){		//对终点的处理
	    ADEBUG << "destination point : waypoint index : " << i;
	    std::vector<hdmap::LaneInfoConstPtr> dest_lanes;
	    if(hdmap_->GetLanes(point, 1.5, &dest_lanes) != 0)  //point是终点waypoint
	    {
		    ADEBUG << "get destination lanes error.";
		    AERROR << "Failed to find lanes nearest destination:"
			          << point.DebugString();
		    return fixed_request;
	    }

      if(0 == dest_lanes.size()){
        ADEBUG << "destination lane size = 0. refind dest lane.";
        hdmap::LaneInfoConstPtr nearest_lane;
        double nearest_s = 0.0;
        double nearest_l = 0.0;
        if(hdmap_->GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l)){
          ADEBUG << "refind destination lane failed.";
          AERROR << "Failed to find nearest destination lane at position: "
                 << point.DebugString();
        }
        dest_lanes.push_back(nearest_lane);
      }
	    ADEBUG << "destination point : candidate lanes size : " << dest_lanes.size();

	    double min_total_distance = std::numeric_limits<double>::infinity();
	    //ADEBUG << "start waypoint size : " << start_waypoints.size();	
      for(const auto& start_wp : start_waypoints)//根据routing路线的长短来确定起始和终止waypoint
      {
        //ADEBUG << "start waypoint id " << start_wp.id();
	      for(const auto& lane : dest_lanes)
	      {
		     // if(!lane->IsOnLane({point.x(), point.y()})) //滤除搜索到的无关的Lane
		     /* {
			      ADEBUG << "destination candidate lane id : " << lane->id().id() << " ---------- is not on lane," << " vehicle point{" << point.x() << ", " << point.y() << "}.";
			      continue;
		      }else{*/
			      double dest_s = 0.0;
			      double dest_l = 0.0;
			      lane->GetProjection({point.x(), point.y()}, &dest_s, &dest_l);
		
			     // ADEBUG << "destination candidate lane id : " << lane->id().id() << " ----------";
			      RoutingRequest request_tmp(fixed_request);
            
            auto start_waypoint_tmp = request_tmp.mutable_waypoint(0);
            start_waypoint_tmp->set_id(start_wp.id());
            start_waypoint_tmp->set_s(start_wp.s());
            start_waypoint_tmp->mutable_pose()->CopyFrom(start_wp.pose());
            
			      auto end_waypoint_tmp = request_tmp.mutable_waypoint(i);
			      end_waypoint_tmp->set_id(lane->id().id());
			      end_waypoint_tmp->set_s(dest_s);
			      double total_distance_tmp = navigator_ptr_->DistanceRouting(request_tmp);
			      ADEBUG << "start lane id : " << start_wp.id()  << " end lane id : " << lane->id().id() << " total distance : " << total_distance_tmp << " s = " << dest_s << ", l = " << dest_l;	
			      if((total_distance_tmp >= 0) && (total_distance_tmp < min_total_distance))
		    	  {
				      min_total_distance = total_distance_tmp;
				
              auto start_waypoint_info = fixed_request.mutable_waypoint(0);
              start_waypoint_info->set_id(start_wp.id());
              start_waypoint_info->set_s(start_wp.s());
              start_waypoint_info->mutable_pose()->CopyFrom(start_wp.pose());

				      auto end_waypoint_info = fixed_request.mutable_waypoint(i);
				      end_waypoint_info->set_id(lane->id().id());
				      end_waypoint_info->set_s(dest_s);
				      ADEBUG << "find min_total_distance : " << min_total_distance << " start lane id : " << start_wp.id() << " end lane id : " << lane->id().id();
			      }	
		     // }
	      }
      }
    }else{	
       double mid_s = 0.0;
       double mid_l = 0.0;
       hdmap::LaneInfoConstPtr mid_lane;
       if (hdmap_->GetNearestLane(point, &mid_lane, &mid_s, &mid_l) != 0) {
         ADEBUG << "get nearest lane error.";
         AERROR << "Failed to find nearest lane from map at position: "
                << point.DebugString();
          return routing_request;
       }
       auto waypoint_info = fixed_request.mutable_waypoint(i);
       waypoint_info->set_id(mid_lane->id().id());
       waypoint_info->set_s(mid_s);
       ADEBUG << "routing : else points " << i << " : {" << point.x() << " ," << point.y() << "}";
    }
  }
  AINFO << "Fixed routing request:" << fixed_request.DebugString();
  return fixed_request;
}

//根据Lane的平均heading和与车辆的heading的差值来确定一些合适的Waypoints
std::vector<LaneWaypoint> Routing::GetProperWaypoints(const std::vector<hdmap::LaneInfoConstPtr>& valid_lanes, 
                                                      const common::PointENU& vehicle_point, 
                                                      const double vehicle_heading)
{
  std::vector<LaneWaypoint> waypoints;
  std::vector<std::pair<LaneWaypoint, double>> candidate_waypoints;
  double threshold_deg = common::math::Angle8::from_rad(ROUTING_HEADING_THRESHOLD).to_deg();
  for(const auto& lane : valid_lanes)	//std::vector<hdmap::LaneInfoConstPtr>::size_typ
  {
    double ss = 0.0;
    double ll = 0.0;
    lane->GetProjection({vehicle_point.x(), vehicle_point.y()}, &ss, &ll);
    bool is_onlane = lane->IsOnLane({vehicle_point.x(), vehicle_point.y()});
    ADEBUG << "lane id : " << lane->id().id() << " s = " << ss << " l = " << ll << " total length : " << lane->total_length() << " is on lane : " << is_onlane;
    double lane_heading_avg = 0.0;//lanes[i]->Heading(ss + 2);
    common::math::Vec2d vec_avg{0, 0};
    if(!is_onlane)
    {
     // ADEBUG << "vehicle not on this lane " << " id : " << lane->id().id();
      continue;
    }

    double rest_len = lane->total_length() - ss;
    if(rest_len <= 2.0)
    {
      lane_heading_avg = lane->Heading(ss);
    }else
    {
      double cal_len = 10.0;
      if(rest_len <= cal_len)
      {
        cal_len = rest_len;
      }
      double s_segment = std::floor(cal_len / 5.0);
      for(int i = 1; i < 5; ++i)
      {	
        double segment_heading = lane->Heading(ss + s_segment * i);
        auto segment_unit = common::math::Vec2d::CreateUnitVec2d(segment_heading);
        vec_avg += segment_unit;
        vec_avg.Normalize();
      //  ADEBUG << "ss = " << ss << " s_segment = " << s_segment << " s = " << ss + s_segment * i << " heading count : " << segment_heading << " avg heading : " << vec_avg.Angle();
      }
      lane_heading_avg = vec_avg.Angle();
    }
    //ADEBUG << "lane id : " << lane->id().id() << " avg_heading = " << lane_heading_avg << " vehicle heading : " << vehicle_heading;
//    double distance = lane->DistanceTo({vehicle_point.x(), vehicle_point.y()});    
    double heading_diff = 0.0;
    if((heading_diff = std::fabs(common::math::NormalizeAngle(lane_heading_avg - vehicle_heading))) <= ROUTING_HEADING_THRESHOLD)
    {
      ADEBUG << "lane id : " << lane->id().id() << " avg_heading = " << lane_heading_avg << " vehicle heading : " << vehicle_heading << " heading diff <= " << threshold_deg << " : " << common::math::Angle8::from_rad(heading_diff).to_deg() << "(" << heading_diff << ")" << " distance to vehicle : " << ll << " rest len : " << rest_len;
      //ADEBUG << "lane id : " << lane->id().id() << " heading diff <= 30° : " << heading_diff;
      if(std::fabs(ll) >= 2.4)
      {
        ADEBUG << "lane id : " << lane->id().id() << " is too far from the vehicle, distance = " << ll;
        continue;
      }
      LaneWaypoint wp;
      wp.set_id(lane->id().id());
      wp.set_s(ss);
      wp.mutable_pose()->CopyFrom(lane->GetSmoothPoint(ss));
      candidate_waypoints.push_back(std::make_pair(wp, heading_diff));
    }else
    {
      ADEBUG << "lane id : " << lane->id().id() << " avg_heading = " << lane_heading_avg << " vehicle heading : " << vehicle_heading << " heading diff >= " << threshold_deg << " : " << common::math::Angle8::from_rad(heading_diff).to_deg() << "(" << heading_diff << ")" << " distance to vehicle : " << ll << " rest len : " << rest_len;
      //ADEBUG << "lane id : " << lane->id().id() << " heading diff > 30° : " << heading_diff;
    }
  }

  if(0 == candidate_waypoints.size())
  {
    ADEBUG << "no lanes avg heading less than " << threshold_deg << "° or lane is too far!";
    return waypoints;
  }

  auto min_iter = min_element(candidate_waypoints.begin(), candidate_waypoints.end(), [&](std::pair<LaneWaypoint, double> a, std::pair<LaneWaypoint, double> b){return a.second < b.second;});
  double min_diff = (*min_iter).second;
  ADEBUG << "min heading diff : " << min_diff;

  std::vector<std::pair<LaneWaypoint, double>> valid_waypoints;
  std::copy_if(candidate_waypoints.begin(), candidate_waypoints.end(), back_inserter(valid_waypoints), [&](std::pair<LaneWaypoint, double> a){return a.second <= min_diff * 2;});
  for(auto& p : valid_waypoints)
  {
    ADEBUG << "valid lane id : " << p.first.id() << " heading diff : " << p.second;
    waypoints.push_back(p.first);
  }

  if(0 == waypoints.size())
  {
    ADEBUG << "no lanes avg heading less than " << threshold_deg << "° or lane is too far!";
  }

  return waypoints;
}

void Routing::OnRoutingRequest(const RoutingRequest& routing_request) {
  AINFO << "Get new routing request:" << routing_request.DebugString();
  RoutingResponse routing_response;
  AERROR<< "routing 1.";
  AERROR << "outing 1";
  jmc_auto::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  AERROR<< "routing 2.";
  AERROR << "routing 2.";
  const auto& fixed_request = FillLaneInfoIfMissing(routing_request);
  if (!navigator_ptr_->SearchRoute(fixed_request, &routing_response)) {
    AERROR << "Failed to search route with navigator.";

    buffer.WARN("Routing failed! " + routing_response.status().msg());
    return;
  }
  buffer.INFO("Routing success!");
  AINFO << "routing_response:" << routing_response.DebugString();
  AdapterManager::PublishRoutingResponse(routing_response);
  return;
}

void Routing::Stop() {}

}  // namespace routing
}  // namespace jmc_auto
