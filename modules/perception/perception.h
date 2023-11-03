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

#ifndef MODULES_PERCEPTION_PERCEPTION_H_
#define MODULES_PERCEPTION_PERCEPTION_H_

#include <string>

#include "modules/common/jmc_auto_app.h"
#include "modules/perception/onboard/dag_streaming.h"

/**
 * @namespace jmc_auto::perception
 * @brief jmc_auto::perception
 */
namespace jmc_auto {
namespace perception {

class Perception : public common::JmcAutoApp {
 public:
  std::string Name() const override;
  common::Status Init() override;
  common::Status Start() override;
  void Stop() override;

 private:
  DAGStreaming dag_streaming_;
  void RegistAllOnboardClass();
};

}  // namespace perception
}  // namespace jmc_auto

#endif  // MODULES_PERCEPTION_PERCEPTION_H_
