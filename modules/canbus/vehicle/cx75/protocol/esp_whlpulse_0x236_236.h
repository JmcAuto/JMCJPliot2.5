/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_WHLPULSE_0X236_236_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_WHLPULSE_0X236_236_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Espwhlpulse0x236236 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Espwhlpulse0x236236();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ESP_WheelPulse_FR', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int esp_wheelpulse_fr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WheelPulse_RL', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int esp_wheelpulse_rl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WheelPulse_RR', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int esp_wheelpulse_rr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RollingCounter_ESP_0x236', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rollingcounter_esp_0x236(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WheelPulse_RR_Valid', 'enum': {0: 'ESP_WHEELPULSE_RR_VALID_VALID', 1: 'ESP_WHEELPULSE_RR_VALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_whlpulse_0x236_236::Esp_wheelpulse_rr_validType esp_wheelpulse_rr_valid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WheelPulse_RL_Valid', 'enum': {0: 'ESP_WHEELPULSE_RL_VALID_VALID', 1: 'ESP_WHEELPULSE_RL_VALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_whlpulse_0x236_236::Esp_wheelpulse_rl_validType esp_wheelpulse_rl_valid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WheelPulse_FR_Valid', 'enum': {0: 'ESP_WHEELPULSE_FR_VALID_VALID', 1: 'ESP_WHEELPULSE_FR_VALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_whlpulse_0x236_236::Esp_wheelpulse_fr_validType esp_wheelpulse_fr_valid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WheelPulse_FL_Valid', 'enum': {0: 'ESP_WHEELPULSE_FL_VALID_VALID', 1: 'ESP_WHEELPULSE_FL_VALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_whlpulse_0x236_236::Esp_wheelpulse_fl_validType esp_wheelpulse_fl_valid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_ESP_0x236', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_esp_0x236(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WheelPulse_FL', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int esp_wheelpulse_fl(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_ESP_WHLPULSE_0X236_236_H_
