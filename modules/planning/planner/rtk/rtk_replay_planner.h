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

#pragma once

#include <string>
#include <vector>

#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/proto/planning_config.pb.h"

/**
 * @namespace jmc_auto::planning
 * @brief jmc_auto::planning
 */
namespace jmc_auto {
namespace planning {

/**
 * @class RTKReplayPlanner
 * @brief RTKReplayPlanner is a derived class of Planner.
 *        It reads a recorded trajectory from a trajectory file and
 *        outputs proper segment of the trajectory according to vehicle
 * position.
 */
class RTKReplayPlanner : public PlannerWithReferenceLine {
 public:
  /**
   * @brief Constructor
   */
  RTKReplayPlanner();

  /**
   * @brief Destructor
   */
  virtual ~RTKReplayPlanner() = default;

  std::string Name() override { return "RTK"; }

  jmc_auto::common::Status Init(const PlanningConfig& config) override;

  void Stop() override {}

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  jmc_auto::common::Status Plan(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ADCTrajectory* ptr_computed_trajectory) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  jmc_auto::common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;
  /**
   * @brief Read the recorded trajectory file.
   * @param filename The name of the trajectory file.
   */
  void ReadTrajectoryFile(const std::string& filename);

 private:
  std::uint32_t QueryPositionMatchedPoint(
      const common::TrajectoryPoint& start_point,
      const std::vector<common::TrajectoryPoint>& trajectory) const;

  std::vector<common::TrajectoryPoint> complete_rtk_trajectory_;
};

}  // namespace planning
}  // namespace jmc_auto
