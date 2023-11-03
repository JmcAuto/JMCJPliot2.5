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
#ifndef MODULES_MONITOR_SOFTWARE_PROCESS_MONITOR_H_
#define MODULES_MONITOR_SOFTWARE_PROCESS_MONITOR_H_

#include <map>
#include <string>

#include "modules/monitor/common/recurrent_runner.h"
#include "modules/monitor/proto/monitor_conf.pb.h"

namespace jmc_auto {
namespace monitor {

class ProcessMonitor : public RecurrentRunner {
 public:
  ProcessMonitor();
  void RunOnce(const double current_time) override;

 private:
  static void UpdateModule(
      const std::string &module_name, const ProcessConf &process_conf,
      const std::map<std::string, std::string> &running_processes);
};

}  // namespace monitor
}  // namespace jmc_auto

#endif  // MODULES_MONITOR_SOFTWARE_PROCESS_MONITOR_H_
