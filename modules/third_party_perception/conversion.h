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

#ifndef MODEULES_THIRD_PARTY_PERCEPTION_CONVERSION_H_
#define MODEULES_THIRD_PARTY_PERCEPTION_CONVERSION_H_

#include <cstdint>
#include <queue>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/third_party_perception/proto/radar_obstacle.pb.h"

/**
 * @namespace jmc_auto::third_party_perception::conversion
 * @brief jmc_auto::third_party_perception
 */
namespace jmc_auto {
namespace third_party_perception {
namespace conversion {

jmc_auto::perception::PerceptionObstacles MobileyeToPerceptionObstacles(
    const jmc_auto::drivers::Mobileye& mobileye,
    const jmc_auto::localization::LocalizationEstimate& localization,
    const jmc_auto::canbus::Chassis& chassis);

RadarObstacles DelphiToRadarObstacles(
    const jmc_auto::drivers::DelphiESR& delphi_esr,
    const jmc_auto::localization::LocalizationEstimate& localization,
    const RadarObstacles& last_radar_obstacles);

RadarObstacles ContiToRadarObstacles(
    const jmc_auto::drivers::ContiRadar& conti_radar,
    const jmc_auto::localization::LocalizationEstimate& localization,
    const RadarObstacles& last_radar_obstacles,
    const jmc_auto::canbus::Chassis& chassis);

jmc_auto::perception::PerceptionObstacles RadarObstaclesToPerceptionObstacles(
    const RadarObstacles& radar_obstacles);

}  // namespace conversion
}  // namespace third_party_perception
}  // namespace jmc_auto

#endif  // MODULES_THIRD_PARTY_PERCEPTION_FUSION_H_
