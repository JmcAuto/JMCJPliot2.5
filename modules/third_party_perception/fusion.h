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

/**
 * @file
 */

#ifndef MODEULES_THIRD_PARTY_PERCEPTION_FUSION_H_
#define MODEULES_THIRD_PARTY_PERCEPTION_FUSION_H_

#include "modules/perception/proto/perception_obstacle.pb.h"

/**
 * @namespace jmc_auto::third_party_perception::fusion
 * @brief jmc_auto::third_party_perception
 */
namespace jmc_auto {
namespace third_party_perception {
namespace fusion {

jmc_auto::perception::PerceptionObstacles MobileyeRadarFusion(
    const jmc_auto::perception::PerceptionObstacles& mobileye_obstacles,
    const jmc_auto::perception::PerceptionObstacles& radar_obstacles);

}  // namespace fusion
}  // namespace third_party_perception
}  // namespace jmc_auto

#endif  // MODULES_THIRD_PARTY_PERCEPTION_FUSION_H_
