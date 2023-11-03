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

#include <string>
#include <vector>

#include "gflags/gflags.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"

// #include "modules/map/hdmap/hdmap_util.h"
#include "modules/tools/map_cidi2jinglong/cidi_proto/map.pb.h"
#include "modules/tools/map_cidi2jinglong/jinglong_proto/map.pb.h"

#include "modules/common/util/points_downsampler.h"

#include "modules/routing/proto/routing_config.pb.h"
#include "modules/tools/map_cidi2jinglong/topo_creator/graph_creator.h"
#include "modules/routing/common/routing_gflags.h"

/**
 * A map tool to generate a downsampled map to be displayed by dreamview
 * frontend.
 */

DEFINE_string(output_dir, "/tmp/", "output map directory");

//sim_map
DEFINE_double(angle_threshold, 1. / 180 * M_PI, /* 1 degree */
              "Points are sampled when the accumulated direction change "
              "exceeds the threshold");
DEFINE_int32(downsample_distance, 5, "downsample rate for a normal path");
DEFINE_int32(steep_turn_downsample_distance, 1,
             "downsample rate for a steep turn path");

using jmc_auto::common::PointENU;
using jmc_auto::common::util::DownsampleByAngle;
using jmc_auto::common::util::DownsampleByDistance;
using jmc_auto::common::util::GetProtoFromFile;
using adu::common::hdmap::Curve;

//使用bin读取太过麻烦，将cidi.proto中多余的信息删除，生成txt文件，再用jingzhong.proto读取
//要保持cidi.proto和jingzhong.proto之间的变量名一致
bool cidi2jinglong(apollo::hdmap::Map& cidi_map_pb ,adu::common::hdmap::Map& jinglong_map_pb){
    // //hearder
    // jinglong_map_pb.header().set_version(cidi_map_pb.header().version());
    // jinglong_map_pb.header().set_date(cidi_map_pb.header().date());
    // jinglong_map_pb.header().set_district(cidi_map_pb.header().district());
    // //croswork
    // jinglong_map_pb.
    //删除多余、不一样的变量
    //heard，大量的不同元素，先全部删除
    
    if (cidi_map_pb.has_header())
    {
        cidi_map_pb.clear_header();
    }
    
    //crosswalk元素相同，不做修改
    //junction,cidi地图元素比jinglong少，只能提供基本元素，后续如必须，再添加
    //lane中,cidi缺少width、central_curvature、multiple_type、road_id。而LaneType在cidi中有多余的车型
    for (size_t i = 0; i < cidi_map_pb.lane_size(); i++)
    {
    if (cidi_map_pb.lane(i).has_path())
    {
        cidi_map_pb.mutable_lane(i)->clear_path();
    }
    
    if (cidi_map_pb.lane(i).has_review_status())
    {
        cidi_map_pb.mutable_lane(i)->clear_review_status();
    }

    if (cidi_map_pb.lane(i).has_city_driving_lane_type())
    {
        cidi_map_pb.mutable_lane(i)->clear_city_driving_lane_type();
    }
    
    }
    
    
    
    //stop_sign，cidi缺少location、signal_id
    //signal
    for (size_t i = 0; i < cidi_map_pb.signal_size(); i++)
    {
      if ( cidi_map_pb.signal(i).sign_info_size()!=0)
      {
          cidi_map_pb.mutable_signal(i)->clear_sign_info();
      }
    }
    
    
    //yield,cidi中缺少location
    //overlap cidi的LaneOverlapInfo中缺少has_precedence ，overlap_info中有多余的类型
    for (size_t i = 0; i < cidi_map_pb.overlap_size(); i++)
    {
      if ( cidi_map_pb.overlap(i).object_size()!=0)
      {
        for (size_t j = 0; j < cidi_map_pb.overlap(i).object_size(); j++)
        {
          if ( cidi_map_pb.overlap(i).object(j).has_mark_point_overlap_info())
          {
            cidi_map_pb.mutable_overlap(i)->mutable_object(j)->clear_mark_point_overlap_info();
          }
          
          if ( cidi_map_pb.overlap(i).object(j).has_pnc_junction_overlap_info())
          {
            cidi_map_pb.mutable_overlap(i)->mutable_object(j)->clear_pnc_junction_overlap_info();
          }
          if ( cidi_map_pb.overlap(i).object(j).has_remark_overlap_info())
          {
            cidi_map_pb.mutable_overlap(i)->mutable_object(j)->clear_remark_overlap_info();
          }
          if ( cidi_map_pb.overlap(i).object(j).has_mark_polygon_overlap_info())
          {
            cidi_map_pb.mutable_overlap(i)->mutable_object(j)->clear_mark_polygon_overlap_info();
          }
          if ( cidi_map_pb.overlap(i).object(j).has_location_overlap_info())
          {
            cidi_map_pb.mutable_overlap(i)->mutable_object(j)->clear_location_overlap_info();
          }
          if ( cidi_map_pb.overlap(i).object(j).has_mark_polygon_overlap_info())
          {
            cidi_map_pb.mutable_overlap(i)->mutable_object(j)->clear_mark_polygon_overlap_info();
          }
          if ( cidi_map_pb.overlap(i).object(j).has_mark_line_overlap_info())
          {
            cidi_map_pb.mutable_overlap(i)->mutable_object(j)->clear_mark_line_overlap_info();
          }
        }
        
          
      }
    }
    
    //ClearArea正常
    //speed_bump正常
    //parking_space
    for (size_t i = 0; i < cidi_map_pb.parking_space_size(); i++)
    {
      if (cidi_map_pb.parking_space(i).has_type())
      {
          cidi_map_pb.mutable_parking_space(i)->clear_type();
      }
    }
    //road cidi的road中多了type，多了RoadROIBoundary
    for (size_t i = 0; i < cidi_map_pb.road_size(); i++)
    {
      if (cidi_map_pb.road(i).has_type())
      {
          cidi_map_pb.mutable_road(i)->clear_type();
      }
    }
    
   

    if ( cidi_map_pb.pnc_junction_size()!=0)
    {
       cidi_map_pb.clear_pnc_junction();
    }
    if (cidi_map_pb.mark_point_size()!=0)
    {
      cidi_map_pb.clear_mark_point();
    }
    if (cidi_map_pb.remark_size()!=0)
    {
      cidi_map_pb.clear_remark();
    }
    
    if (cidi_map_pb.mark_polygon_size()!=0)
    {
      cidi_map_pb.clear_mark_polygon();
    }
    if (cidi_map_pb.mark_line_size()!=0)
    {
      cidi_map_pb.clear_mark_line();
    }
    
    std::ofstream map_txt_file(FLAGS_output_dir + "/cidi_less_base_map.txt");
    map_txt_file << cidi_map_pb.DebugString();
    map_txt_file.close();
    const auto cidi_less_base_map=FLAGS_output_dir + "/cidi_less_base_map.txt";
    CHECK(jmc_auto::common::util::GetProtoFromFile(cidi_less_base_map, &jinglong_map_pb)) << "Fail to open: " << cidi_less_base_map;


}

//生成jingzong的sim_map
void DownsampleCurve(Curve* curve) {
  auto* line_segment = curve->mutable_segment(0)->mutable_line_segment();
  std::vector<adu::common::hdmap::Point> points(line_segment->point().begin(),
                               line_segment->point().end());
  line_segment->clear_point();

  // NOTE: this not the most efficient implementation, but since this map tool
  // is only run once for each, we can probably live with that.

  // Downsample points by angle then by distance.
  auto sampled_indices = DownsampleByAngle(points, FLAGS_angle_threshold);
  std::vector<adu::common::hdmap::Point> downsampled_points;
  for (int index : sampled_indices) {
    downsampled_points.push_back(points[index]);
  }

  sampled_indices =
      DownsampleByDistance(downsampled_points, FLAGS_downsample_distance,
                           FLAGS_steep_turn_downsample_distance);

  for (int index : sampled_indices) {
    *line_segment->add_point() = downsampled_points[index];
  }
  int new_size = line_segment->point_size();
  CHECK_GT(new_size, 1);

  AINFO << "Lane curve downsampled from " << points.size() << " points to "
        << new_size << " points.";
}

void DownsampleMap(adu::common::hdmap::Map* map_pb) {
  for (int i = 0; i < map_pb->lane_size(); ++i) {
    auto* lane = map_pb->mutable_lane(i);
    lane->clear_left_sample();
    lane->clear_right_sample();
    lane->clear_left_road_sample();
    lane->clear_right_road_sample();

    AINFO << "Downsampling lane " << lane->id().id();
    DownsampleCurve(lane->mutable_central_curve());
    DownsampleCurve(lane->mutable_left_boundary()->mutable_curve());
    DownsampleCurve(lane->mutable_right_boundary()->mutable_curve());
  }
}

static void OutputSimMap(const adu::common::hdmap::Map& map_pb) {
  std::ofstream map_txt_file(FLAGS_output_dir + "/jinglong_sim_map.txt");
  map_txt_file << map_pb.DebugString();
  map_txt_file.close();

  std::ofstream map_bin_file(FLAGS_output_dir + "/jinglong_sim_map.bin");
  std::string map_str;
  map_pb.SerializeToString(&map_str);
  map_bin_file << map_str;
  map_bin_file.close();
}

static void OutputMap(apollo::hdmap::Map& map_pb) {
  
  adu::common::hdmap::Map jinglong_map_pb;
  
  
  std::ofstream cidi_map_txt_file(FLAGS_output_dir + "/cidi_base_map.txt");
  cidi_map_txt_file << map_pb.DebugString();
  cidi_map_txt_file.close();

  cidi2jinglong(map_pb,jinglong_map_pb);

  std::ofstream map_txt_file(FLAGS_output_dir + "/jinglong_base_map.txt");
  map_txt_file << jinglong_map_pb.DebugString();
  map_txt_file.close();

  std::ofstream map_bin_file(FLAGS_output_dir + "/jinglong_base_map.bin");
  std::string map_str;
  jinglong_map_pb.SerializeToString(&map_str);
  map_bin_file << map_str;
  map_bin_file.close();
  //生成jingzong_simmap
  DownsampleMap(&jinglong_map_pb);
  OutputSimMap(jinglong_map_pb);
  //生成routing_map
  jmc_auto::routing::RoutingConfig routing_conf;
   const auto jinglong_base_map=FLAGS_output_dir + "jinglong_base_map.txt";
   const auto jinglong_routing_map=FLAGS_output_dir + "jinglong_routing_map.bin";
  const auto map_file = FLAGS_map_dir + "/navi_base_map.bin";
  CHECK(jmc_auto::common::util::GetProtoFromFile(FLAGS_routing_conf_file,
                                               &routing_conf)) 
                                               << "Unable to load routing conf file: " + FLAGS_routing_conf_file;
  // jmc_auto::tools::GraphCreator creator(map_file, jinglong_routing_map, &routing_conf);
  //CHECK(creator.Create()) << "Create routing topo failed!";

}





int main(int32_t argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::hdmap::Map cidi_map_pb;
  const auto map_file = FLAGS_map_dir + "/base_map.bin";
  CHECK(jmc_auto::common::util::GetProtoFromFile(map_file, &cidi_map_pb)) << "Fail to open: " << map_file;

//   DownsampleMap(&map_pb);
  OutputMap(cidi_map_pb);
  AINFO << "sim_map generated at:" << FLAGS_output_dir;

  return 0;
}
