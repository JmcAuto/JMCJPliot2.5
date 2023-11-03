/******************************************************************************
 * Copyright 2017 The jmc_auto Authors. All Rights Reserved.
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
 **/

#pragma once

#include <string>

#include "modules/common/proto/geometry.pb.h"
#include "modules/planning/traffic_rules/traffic_rule.h"

namespace jmc_auto {
namespace planning {

/**
 * This class decides whether we should stop for destination.
 * situation.
 */
class Destination : public TrafficRule {
 public:
  explicit Destination(const TrafficRuleConfig& config);
  virtual ~Destination() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);
  bool CheckParkingSpotPreStop(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info,
                               double* target_s);
  bool SetParkingSpotStopFence(const double target_s, Frame* const frame,
                               ReferenceLineInfo* const reference_line_info);
  bool CheckADCStop(const Frame& frame);
  bool CheckParkingValid(Frame *const frame, ReferenceLineInfo* const reference_line_info);

 private:
  int MakeDecisions(Frame* const frame,
                    ReferenceLineInfo* const reference_line_info);
  int MakeParkingDecisions(Frame* const frame,
                    ReferenceLineInfo* const reference_line_info);
  std::string target_parking_spot_id_;
  int target_parking_spot_num_;
  //bool is_near_parking_ = false;
//  int count_ = 0;
};

}  // namespace planning
}  // namespace jmc_auto
