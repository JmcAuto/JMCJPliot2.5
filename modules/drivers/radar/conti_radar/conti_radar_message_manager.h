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
 * @file conti_radar_message_manager.h
 * @brief The class of ContiRadarMessageManager
 */
#ifndef MODULES_DRIVERS_RADAR_CONTI_RADAR_CONTI_RADAR_MESSAGE_MANAGER_H_
#define MODULES_DRIVERS_RADAR_CONTI_RADAR_CONTI_RADAR_MESSAGE_MANAGER_H_

#include <memory>
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/radar/conti_radar/protocol/radar_config_200.h"
#include "modules/drivers/proto/conti_radar.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/drivers/canbus/sensor_gflags.h"

namespace jmc_auto {
namespace drivers {
namespace conti_radar {

using ::jmc_auto::drivers::canbus::ProtocolData;
using ::jmc_auto::common::adapter::AdapterManager;
using ::jmc_auto::drivers::canbus::MessageManager;
using Clock = ::jmc_auto::common::time::Clock;
using micros = std::chrono::microseconds;
using ::jmc_auto::common::ErrorCode;
using jmc_auto::drivers::canbus::CanClient;
using jmc_auto::drivers::canbus::SenderMessage;
using jmc_auto::drivers::conti_radar::RadarConfig200;

class ContiRadarMessageManager : public MessageManager<ContiRadar> {
 public:
  ContiRadarMessageManager();
  virtual ~ContiRadarMessageManager() {}
  void set_radar_conf(RadarConf radar_conf);
  ProtocolData<ContiRadar> *GetMutableProtocolDataById(
      const uint32_t message_id);
  void Parse(const uint32_t message_id, const uint8_t *data, int32_t length);
  void set_can_client(std::shared_ptr<CanClient> can_client);

 private:
  bool is_configured_ = false;
  RadarConfig200 radar_config_;
  std::shared_ptr<CanClient> can_client_;
};

}  // namespace conti_radar
}  // namespace drivers
}  // namespace jmc_auto

#endif  // MODULES_DRIVERS_RADAR_CONTI_RADAR_CONTI_RADAR_MESSAGE_MANAGER_H_
