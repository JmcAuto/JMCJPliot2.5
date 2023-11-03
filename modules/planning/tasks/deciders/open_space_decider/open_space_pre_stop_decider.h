/******************************************************************************
 * Copyright 2018 The jmc_auto Authors. All Rights Reserved.
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

#include "modules/planning/proto/open_space_pre_stop_decider_config.pb.h"

//#include "cyber/common/macros.h"
#include "modules/common/macro.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace jmc_auto {
namespace planning {

class OpenSpacePreStopDecider : public Decider {
 public:
  explicit OpenSpacePreStopDecider(const TaskConfig& config);

 private:
  jmc_auto::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  bool CheckParkingSpotPreStop(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info,
                               double* target_s);

  bool CheckPullOverPreStop(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info,
                            double* target_s);

  void SetParkingSpotStopFence(const double target_s, Frame* const frame,
                               ReferenceLineInfo* const reference_line_info);

  void SetPullOverStopFence(const double target_s, Frame* const frame,
                            ReferenceLineInfo* const reference_line_info);

 private:
  static constexpr const char* OPEN_SPACE_STOP_ID = "OPEN_SPACE_PRE_STOP";
  OpenSpacePreStopDeciderConfig open_space_pre_stop_decider_config_;
};

}  // namespace planning
}  // namespace jmc_auto
