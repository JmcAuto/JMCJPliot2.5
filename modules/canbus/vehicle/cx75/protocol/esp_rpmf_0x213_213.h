/******************************************************************************
 * Copyright 2017 The jmc_auto Authors. All Rights Reserved.
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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_ESP_RPMF_0X213_213_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_ESP_RPMF_0X213_213_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Esprpmf0x213213 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Esprpmf0x213213();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ESP_WheelRpmFR', 'offset': 0.0, 'precision': 0.0625, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|2047.875]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rpm'}
  double esp_wheelrpmfr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WheelRpmFRValid', 'enum': {0: 'ESP_WHEELRPMFRVALID_VALID', 1: 'ESP_WHEELRPMFRVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_rpmf_0x213_213::Esp_wheelrpmfrvalidType esp_wheelrpmfrvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x213', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x213(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x213', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x213(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WheelRpmFL', 'offset': 0.0, 'precision': 0.0625, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|2047.875]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rpm'}
  double esp_wheelrpmfl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WheelRpmFLValid', 'enum': {0: 'ESP_WHEELRPMFLVALID_VALID', 1: 'ESP_WHEELRPMFLVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_rpmf_0x213_213::Esp_wheelrpmflvalidType esp_wheelrpmflvalid(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_ESP_RPMF_0X213_213_H_
