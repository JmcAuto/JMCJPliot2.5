/******************************************************************************
 * Copyright 2018 The JmcAuto Authors. All Rights Reserved.
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

#include "modules/map/relative_map/relative_map.h"

#include "modules/map/proto/map_lane.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/relative_map/common/relative_map_gflags.h"

namespace jmc_auto {
namespace relative_map {

using jmc_auto::common::ErrorCode;
using jmc_auto::common::Status;
using jmc_auto::common::VehicleStateProvider;
using jmc_auto::common::adapter::AdapterManager;
using jmc_auto::common::util::operator+;
using jmc_auto::common::math::Vec2d;
using jmc_auto::common::monitor::MonitorLogBuffer;
using jmc_auto::common::monitor::MonitorMessageItem;
using jmc_auto::perception::PerceptionObstacles;

RelativeMap::RelativeMap()
    : monitor_logger_(MonitorMessageItem::RELATIVE_MAP) {}

Status RelativeMap::Init() {//初始化
  if (!FLAGS_use_navigation_mode) {
    AERROR << "FLAGS_use_navigation_mode is false, system is not configured "
              "for relative map mode";
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "FLAGS_use_navigation_mode is not true.");
  }
  adapter_conf_.Clear();//先清除，然后加载配置文件
  if (!common::util::GetProtoFromFile(
          FLAGS_relative_map_adapter_config_filename, &adapter_conf_)) {
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "Unable to load adapter conf file: " +
                      FLAGS_relative_map_adapter_config_filename);
  } else {
    ADEBUG << "Adapter config file is loaded into: "
           << adapter_conf_.ShortDebugString();
  }

  config_.Clear();//先清除，然后加载配置文件
  if (!common::util::GetProtoFromFile(FLAGS_relative_map_config_filename,
                                      &config_)) {
    return Status(ErrorCode::RELATIVE_MAP_ERROR,
                  "Unable to load relative map conf file: " +
                      FLAGS_relative_map_config_filename);
  }

  navigation_lane_.SetConfig(config_.navigation_lane());
  const auto& map_param = config_.map_param();//地图参数赋值
  navigation_lane_.SetDefaultWidth(map_param.default_left_width(),
                                   map_param.default_right_width());//导航车道默认宽度设置

  AdapterManager::Init(adapter_conf_);//初始化
  if (!AdapterManager::GetPerceptionObstacles()) {
    std::string error_msg(
        "Perception should be configured as dependency in adapter.conf");
    AERROR << error_msg;
    return Status(ErrorCode::RELATIVE_MAP_ERROR, error_msg);
  }
  if (!AdapterManager::GetMonitor()) {
    std::string error_msg(
        "Monitor should be configured as dependency in adapter.conf");
    AERROR << error_msg;
    return Status(ErrorCode::RELATIVE_MAP_ERROR, error_msg);
  }
  if (AdapterManager::GetLocalization() == nullptr) {
    std::string error_msg(
        "Localization should be configured as dependency in adapter.conf");
    AERROR << error_msg;
    return Status(ErrorCode::RELATIVE_MAP_ERROR, error_msg);
  }
  if (AdapterManager::GetChassis() == nullptr) {
    std::string error_msg(
        "Chassis should be configured as dependency in adapter.conf");
    AERROR << error_msg;
    return Status(ErrorCode::RELATIVE_MAP_ERROR, error_msg);
  }
  if (AdapterManager::GetNavigation() == nullptr) {
    std::string error_msg(
        "Navigation should be configured as dependency in adapter.conf");
    AERROR << error_msg;
    return Status(ErrorCode::RELATIVE_MAP_ERROR, error_msg);
  }
  return Status::OK();
}

void LogErrorStatus(MapMsg* map_msg, const std::string& error_msg) {
  auto* status = map_msg->mutable_header()->mutable_status();
  status->set_msg(error_msg);
  status->set_error_code(ErrorCode::RELATIVE_MAP_ERROR);
}//日志错误状态

jmc_auto::common::Status RelativeMap::Start() {
  MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("RelativeMap started");

  timer_ = AdapterManager::CreateTimer(
      ros::Duration(1.0 / FLAGS_relative_map_loop_rate), &RelativeMap::OnTimer,
      this);//设置发布周期，relative_map_loop_rate为10,每0.1s发布一次

  AdapterManager::AddNavigationCallback(&RelativeMap::OnReceiveNavigationInfo,
                                        this);//接收数据

  if (AdapterManager::GetPerceptionObstacles()->Empty()) {
    AWARN << "Perception is not ready.";
  }

  return Status::OK();
}

void RelativeMap::OnTimer(const ros::TimerEvent&) { RunOnce(); }
//定时发布
void RelativeMap::RunOnce() {
  AdapterManager::Observe();

  MapMsg map_msg;//
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);//锁
    CreateMapFromNavigationLane(&map_msg);//从导航车道创建地图
  }
  Publish(&map_msg);//发布
}

void RelativeMap::OnReceiveNavigationInfo(
    const NavigationInfo& navigation_info) {
  {
    std::lock_guard<std::mutex> lock(navigation_lane_mutex_);
    navigation_lane_.UpdateNavigationInfo(navigation_info);
  }
}

bool RelativeMap::CreateMapFromNavigationLane(MapMsg* map_msg) {
  CHECK_NOTNULL(map_msg);

  if (AdapterManager::GetLocalization()->Empty()) {
    LogErrorStatus(map_msg, "localization is not ready");
    return false;
  }
  if (AdapterManager::GetChassis()->Empty()) {
    LogErrorStatus(map_msg, "chassis is not ready");
    return false;
  }

  // update vehicle state from localization and chassis
  //依据定位和地盘更新车辆状态
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Get localization:" << localization.DebugString();
  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  ADEBUG << "Get chassis:" << chassis.DebugString();
  VehicleStateProvider::instance()->Update(localization, chassis);
  map_msg->mutable_localization()->CopyFrom(localization);

  // update navigation_lane from perception_obstacles (lane marker)
  //依据感知障碍物更新导航车道（车道标记）
  if (!AdapterManager::GetPerceptionObstacles()->Empty()) {
    const auto& perception =
        AdapterManager::GetPerceptionObstacles()->GetLatestObserved();
    navigation_lane_.UpdatePerception(perception);
    map_msg->mutable_lane_marker()->CopyFrom(perception.lane_marker());
  }

  if (!navigation_lane_.GeneratePath()) {//生成路径
    LogErrorStatus(map_msg, "navigation lane fails to generate path");
    return false;
  }

  if (navigation_lane_.Path().path().path_point_size() == 0) {//路径
    LogErrorStatus(map_msg, "navigation lane has no path points");
    return false;
  }

  // create map proto from navigation_path
  if (!navigation_lane_.CreateMap(config_.map_param(), map_msg)) {//生成地图
    LogErrorStatus(map_msg, "Failed to create map from navigation path");
    AERROR << "Failed to create map from navigation path";
    return false;
  }
  return true;
}

void RelativeMap::Stop() {//停止
  MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("RelativeMap stopped");
}

}  // namespace relative_map
}  // namespace jmc_auto
