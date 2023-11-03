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

#ifndef MODULES_CANBUS_CANBUS_H_
#define MODULES_CANBUS_CANBUS_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/canbus/proto/chassis_detail.pb.h"

#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/jmc_auto_app.h"
#include "modules/common/macro.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"

#include "modules/remotecontrol/proto/remote_control.pb.h"
//#include "modules/guardian/proto/guardian.pb.h"

/**
 * @namespace jmc_auto::canbus
 * @brief jmc_auto::canbus
 */
namespace jmc_auto {
namespace canbus {

/**
 * @class Canbus
 *
 * @brief canbus module main class.
 * It processes the control data to send protocol messages to can card.
 */
class Canbus : public jmc_auto::common::JmcAutoApp {
 public:
  Canbus()
      : monitorger_(jmc_auto::common::monitor::MonitorMessageItem::CANBUS) {}

  /**
   * @brief obtain module name
   * @return module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  jmc_auto::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  jmc_auto::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

 private:
  void PublishChassis();
  void PublishChassisDetail();
  void OnTimer(const ros::TimerEvent &event);
  void OnControlCommand(const jmc_auto::control::ControlCommand &control_command);


void OnRemoteControlCommand(const jmc_auto::remote::RemoteControl &RemoteControlCommand);
  jmc_auto::control::ControlCommand RemoteCmdToControlCmd(const jmc_auto::remote::RemoteControl &RemoteControlCommand);
//   void OnGuardianCommand(
//       const jmc_auto::guardian::GuardianCommand &guardian_command);
  jmc_auto::common::Status OnError(const std::string &error_msg);
  void RegisterCanClients();
  void setControlcmd(const ros::TimerEvent &event);

  CanbusConf canbus_conf_;
  std::unique_ptr<jmc_auto::drivers::canbus::CanClient> can_client_;
  CanSender<ChassisDetail> can_sender_;
  jmc_auto::drivers::canbus::CanReceiver<ChassisDetail> can_receiver_;
  std::unique_ptr<MessageManager<::jmc_auto::canbus::ChassisDetail>>
      message_manager_;
  std::unique_ptr<VehicleController> vehicle_controller_;
  bool IS_STOP_MODE = false;
  bool IS_VEHCILE_STOP = false;
  jmc_auto::control::ControlCommand control_command_;

  bool IS_Remote_MODE=false;
  int64_t last_timestamp_ = 0;
  ros::Timer timer_;
  ros::Timer sent_cmd_timer_;
  jmc_auto::common::monitor::MonitorLogger monitorger_;
};

}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_CANBUS_H_
