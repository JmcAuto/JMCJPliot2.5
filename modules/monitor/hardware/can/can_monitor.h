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
#ifndef MODULES_MONITOR_HARDWARE_CAN_CAN_MONITOR_H_
#define MODULES_MONITOR_HARDWARE_CAN_CAN_MONITOR_H_

#include <string>
#include <vector>

#include "modules/monitor/common/recurrent_runner.h"

namespace jmc_auto {
namespace monitor {

class CanMonitor : public RecurrentRunner {
 public:
  CanMonitor();
  void RunOnce(const double current_time) override;
};

}  // namespace monitor
}  // namespace jmc_auto

#endif  // MODULES_MONITOR_HARDWARE_CAN_CAN_MONITOR_H_
