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

#ifndef MODULES_DRIVERS_GNSS_M2_GNSS_M2_CANBUS_H_
#define MODULES_DRIVERS_GNSS_M2_GNSS_M2_CANBUS_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/jmc_auto_app.h"
#include "modules/common/macro.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/gnss_m2/proto/gnss_m2_conf.pb.h"
#include "modules/drivers/gnss_m2/common/m2_gflags.h"
#include "modules/drivers/gnss_m2/gnss_m2_message_manager.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/gnss_m2/protocol/odometer_input_99.h"

#include "modules/drivers/gnss_m2/proto/m2.pb.h"
/**
 * @namespace jmc_auto::drivers
 * @brief jmc_auto::drivers
 */
namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

/**
* @class TteContiRadarCanbus
*
* @brief template of canbus-based sensor module main class (e.g., conti_radar).
*/

using jmc_auto::common::adapter::AdapterConfig;
using jmc_auto::common::adapter::AdapterManager;
using jmc_auto::common::monitor::MonitorMessageItem;
using jmc_auto::common::Status;
using jmc_auto::common::ErrorCode;
using jmc_auto::common::time::Clock;
using jmc_auto::drivers::canbus::CanClientFactory;
using jmc_auto::drivers::canbus::CanClient;
using jmc_auto::drivers::canbus::CanReceiver;
using jmc_auto::drivers::canbus::CanSender;
using jmc_auto::drivers::canbus::SenderMessage;

class GnssM2Canbus : public jmc_auto::common::JmcAutoApp {
 public:
  // TODO(lizh): check whether we need a new msg item, say
  // MonitorMessageItem::SENSORCANBUS
  GnssM2Canbus()
          : monitor_logger_(jmc_auto::common::monitor::MonitorMessageItem::CANBUS) {}

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
  void OnTimer(const ros::TimerEvent &event);
  void PublishSensorData();
  Status OnError(const std::string &error_msg);
  void RegisterCanClients();
  void OnChassis(const jmc_auto::canbus::Chassis &Chassis);


  CanConf gnss_m2_conf_;
  std::shared_ptr<CanClient> can_client_;
  CanReceiver<jmc_auto::drivers::gnss_m2::M2> can_receiver_;
  CanSender<jmc_auto::drivers::gnss_m2::M2> can_sender_ ;
  std::unique_ptr<GnssM2MessageManager> sensor_message_manager_;
  ros::Timer timer_;

   Odometerinput99* odometer_input_99_ = nullptr;
   

  int64_t last_timestamp_ = 0;
  jmc_auto::common::monitor::MonitorLogger monitor_logger_;

};

}  // namespace conti_radar
}  // namespace drivers
}  // namespace jmc_auto

#endif  // MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR__CONTI_RADAR_CANBUS_H_
