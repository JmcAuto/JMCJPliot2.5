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

#ifndef MODULES_DRIVERS_RADAR_CONTI_RADAR_PROTOCOL_CLUSTER_LIST_STATUS_600_H_
#define MODULES_DRIVERS_RADAR_CONTI_RADAR_PROTOCOL_CLUSTER_LIST_STATUS_600_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/conti_radar.pb.h"

namespace jmc_auto {
namespace drivers {
namespace conti_radar {

using jmc_auto::drivers::ContiRadar;

class ClusterListStatus600
    : public jmc_auto::drivers::canbus::ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  ClusterListStatus600();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ContiRadar* conti_radar) const override;

 private:
  int near(const std::uint8_t* bytes, int32_t length) const;

  int far(const std::uint8_t* bytes, int32_t length) const;

  int meas_counter(const std::uint8_t* bytes, int32_t length) const;

  int interface_version(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace conti_radar
}  // namespace drivers
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_ESR_PROTOCOL_CLUSTER_LIST_STATUS_600_H_
