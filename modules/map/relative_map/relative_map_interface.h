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

#ifndef MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_INTERFACE_H_
#define MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_INTERFACE_H_

#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/jmc_auto_app.h"

/**
 * @namespace jmc_auto::relative_map
 * @brief jmc_auto::relative_map
 */

namespace jmc_auto {
namespace relative_map {

/**
 * @class RelativeMapInterface
 *
 * @brief Interface of the relative map module.
 * Relative map module is the module that can produce a relative map on the
 * fly based on observations of obstacles and round boundaries provided by
 * perception.
 */
class RelativeMapInterface : public jmc_auto::common::JmcAutoApp {
 public:
  /**
   * @brief main logic of the relative map module, triggered by timer.
   */
  virtual void RunOnce() = 0;

  /**
   * @brief Fill the header and publish the prediction message.
   */
  void Publish(MapMsg *relative_map) {//发布函数
    jmc_auto::common::adapter::AdapterManager::FillRelativeMapHeader(
        Name(), relative_map);
    jmc_auto::common::adapter::AdapterManager::PublishRelativeMap(*relative_map);
  }
};

}  // namespace relative_map
}  // namespace jmc_auto

#endif  // MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_INTERFACE_H_
