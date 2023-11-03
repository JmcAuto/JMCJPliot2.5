/******************************************************************************
 * Copyright 2019 The jmc_auto Authors. All Rights Reserved.
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

#pragma once

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/routing/proto/routing.pb.h"
//#include "modules/control/proto/pad_msg.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"

namespace jmc_auto {
namespace planning {
namespace util {

bool IsVehicleStateValid(const jmc_auto::common::VehicleState& vehicle_state);

bool IsDifferentRouting(const jmc_auto::routing::RoutingResponse& first,
                        const jmc_auto::routing::RoutingResponse& second);

bool IsDifferentPadMessage(const planning::PadMessage& first,
                        const planning::PadMessage& second);

double GetADCStopDeceleration(const double adc_front_edge_s,
                              const double stop_line_s);

bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id);

bool CheckTrafficLightOnReferenceLine(
    const ReferenceLineInfo& reference_line_info,
    const std::string& traffic_light_overlap_id);

bool CheckInsidePnCJunction(const ReferenceLineInfo& reference_line_info);

void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files);

}  // namespace util
}  // namespace planning
}  // namespace jmc_auto
