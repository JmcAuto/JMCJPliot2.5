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

#ifndef MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_H_
#define MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_H_

#include <memory>
#include <mutex>
#include <string>

#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/map/relative_map/proto/relative_map_config.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/map/relative_map/navigation_lane.h"
#include "modules/map/relative_map/relative_map_interface.h"

namespace jmc_auto {
namespace relative_map {

class RelativeMap : public RelativeMapInterface {
 public:
  RelativeMap();

  /**
   * @brief module name
   */
  std::string Name() const override { return "RelativeMap"; };

  /**
   * @brief module initialization function
   * @return initialization status
   */
  jmc_auto::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  jmc_auto::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief destructor
   */
  virtual ~RelativeMap() = default;

  /**
   * @brief main logic of the relative_map module, runs periodically triggered
   * by timer.
   */
  void RunOnce() override;

  void OnTimer(const ros::TimerEvent&);

 private:
  bool CreateMapFromNavigationLane(MapMsg* map_msg);

  void OnReceiveNavigationInfo(const NavigationInfo& navigation_info);

  common::adapter::AdapterManagerConfig adapter_conf_;//adapter配置对象
  RelativeMapConfig config_;
  jmc_auto::common::monitor::MonitorLogger monitor_logger_;//日志对象

  NavigationLane navigation_lane_;//车道对象
  std::mutex navigation_lane_mutex_;//互斥体
  ros::Timer timer_;//时间周期对象
};

}  // namespace relative_map
}  // namespace jmc_auto

#endif  // MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_H_
