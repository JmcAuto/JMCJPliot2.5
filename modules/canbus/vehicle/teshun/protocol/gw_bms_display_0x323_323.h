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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_BMS_DISPLAY_0X323_323_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_BMS_DISPLAY_0X323_323_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwbmsdisplay0x323323 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwbmsdisplay0x323323();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x323', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x323(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_Counter_0x323', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x323(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_ConsumEQ_EST', 'offset': 0.0, 'precision': 0.1, 'len': 14, 'is_signed_var': False, 'physical_range': '[0|800]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kwh'}
  double bms_consumeq_est(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_AvailableEQ_EST', 'offset': 0.0, 'precision': 0.1, 'len': 14, 'is_signed_var': False, 'physical_range': '[0|800]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kwh'}
  double bms_availableeq_est(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_SOH_EST', 'offset': 0.0, 'precision': 0.25, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|400]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  double bms_soh_est(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_SocDisplay_EST', 'offset': 0.0, 'precision': 0.4, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  double bms_socdisplay_est(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_BMS_DISPLAY_0X323_323_H_
