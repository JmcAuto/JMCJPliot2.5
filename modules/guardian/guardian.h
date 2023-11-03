/******************************************************************************
 * Copyright 2018 The JmcAuto Authors. All Rights Reserved.
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

#ifndef MODEULES_GUARDIAN_GUARDIAN_H_
#define MODEULES_GUARDIAN_GUARDIAN_H_

#include <map>
#include <mutex>
#include <queue>
#include <string>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/jmc_auto_app.h"
#include "modules/common/macro.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "ros/include/ros/ros.h"

/**
 * @namespace jmc_auto::guardian
 * @brief jmc_auto::guardian
 */
namespace jmc_auto {
namespace guardian {

class Guardian : public jmc_auto::common::JmcAutoApp {
 public:
  std::string Name() const override;
  jmc_auto::common::Status Init() override;
  jmc_auto::common::Status Start() override;
  void Stop() override;

 private:
  void OnTimer(const ros::TimerEvent&);
  void OnChassis(const jmc_auto::canbus::Chassis& message);
  void OnControl(const jmc_auto::control::ControlCommand& message);
  void OnSystemStatus(const jmc_auto::monitor::SystemStatus& message);
  void PassThroughControlCommand();
  void TriggerSafetyMode();

  jmc_auto::canbus::Chassis chassis_;
  jmc_auto::monitor::SystemStatus system_status_;
  jmc_auto::control::ControlCommand control_cmd_;
  jmc_auto::guardian::GuardianCommand guardian_cmd_;

  std::mutex mutex_;

  ros::Timer timer_;
};

}  // namespace guardian
}  // namespace jmc_auto

#endif  // MODULES_GUARDIAN_GUARDIAN_H_
