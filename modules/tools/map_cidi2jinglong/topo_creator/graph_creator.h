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

#ifndef MODULES_ROUTING_TOPO_CREATOR_GRAPH_CREATOR_H
#define MODULES_ROUTING_TOPO_CREATOR_GRAPH_CREATOR_H

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "modules/tools/map_cidi2jinglong/jinglong_proto/map.pb.h"
#include "modules/routing/proto/routing_config.pb.h"
#include "modules/routing/proto/topo_graph.pb.h"

namespace jmc_auto {
namespace tools {
using jmc_auto::routing::RoutingConfig;
using jmc_auto::routing::Node;
using jmc_auto::routing::Edge;
using jmc_auto::routing::Graph;
using adu::common::hdmap::Id;
class GraphCreator {
 public:
  GraphCreator(const std::string& base_map_file_path,
               const std::string& dump_topo_file_path,
               const RoutingConfig* routing_conf);

  ~GraphCreator() = default;

  bool Create();

 private:
  void InitForbiddenLanes();
  std::string GetEdgeID(const std::string& from_id, const std::string& to_id);

  void AddEdge(
      const Node& from_node,
      const ::google::protobuf::RepeatedPtrField<Id>& to_node_vec,
      const Edge::DirectionType& type);

 private:
  std::string base_map_file_path_;
  std::string dump_topo_file_path_;
  // hdmap::Map pbmap_;
  //暂时改为jinglong的map，用于生成routing文件
  adu::common::hdmap::Map pbmap_;
  Graph graph_;
  std::unordered_map<std::string, int> node_index_map_;
  std::unordered_map<std::string, std::string> road_id_map_;
  std::unordered_set<std::string> showed_edge_id_set_;
  std::unordered_set<std::string> forbidden_lane_id_set_;

  const RoutingConfig* routing_conf_ = nullptr;
};

}  // namespace routing
}  // namespace jmc_auto

#endif  // MODULES_ROUTING_TOPO_CREATOR_GRAPH_CREATOR_H
