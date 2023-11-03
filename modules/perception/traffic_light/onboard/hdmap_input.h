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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_HDMAP_INPUT_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_HDMAP_INPUT_H_

#include <memory>
#include <mutex>
#include <vector>

#include "Eigen/Core"

#include "modules/common/macro.h"
#include "modules/map/hdmap/hdmap.h"

namespace jmc_auto {
namespace perception {
namespace traffic_light {
/**
 * @class HDMapInput
 * @brief HDMap wrapper
 */
class HDMapInput {
 public:
  /**
   * @brief: get roi polygon
   *         all points are in the world frame
   */
  bool GetSignals(const Eigen::Matrix4d &pointd,
                  std::vector<jmc_auto::hdmap::Signal> *signals);

 private:
  DECLARE_SINGLETON(HDMapInput);
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace jmc_auto

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_HDMAP_INPUT_H_
