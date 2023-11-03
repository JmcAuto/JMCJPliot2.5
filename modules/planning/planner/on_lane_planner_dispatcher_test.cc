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

#include "gtest/gtest.h"

#include "modules/planning/planner/on_lane_planner_dispatcher.h"
#include "modules/planning/planner/planner_dispatcher.h"

namespace jmc_auto {
namespace planning {

class OnLanePlannerDispatcherTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

 protected:
  std::unique_ptr<PlannerDispatcher> pd_;
};

TEST_F(OnLanePlannerDispatcherTest, Simple) {
  pd_.reset(new OnLanePlannerDispatcher());
  pd_->Init();
  auto planner = pd_->DispatchPlanner();
  EXPECT_EQ(planner->Name(), "PUBLIC_ROAD");
}

}  // namespace planning
}  // namespace jmc_auto
