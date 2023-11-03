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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_RAW_0X212_212_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_RAW_0X212_212_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Espraw0x212212 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Espraw0x212212();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ESP_vxwWheelRawFR', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 11, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
  double esp_vxwwheelrawfr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_vxwWheelRawRL', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
  double esp_vxwwheelrawrl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_vxwWheelRawRR', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 35, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
  double esp_vxwwheelrawrr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x212', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x212(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_vxwWheelRRValid', 'enum': {0: 'ESP_VXWWHEELRRVALID_VALID', 1: 'ESP_VXWWHEELRRVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_raw_0x212_212::Esp_vxwwheelrrvalidType esp_vxwwheelrrvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_vxwWheelRLValid', 'enum': {0: 'ESP_VXWWHEELRLVALID_VALID', 1: 'ESP_VXWWHEELRLVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_raw_0x212_212::Esp_vxwwheelrlvalidType esp_vxwwheelrlvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_vxwWheelFRValid', 'enum': {0: 'ESP_VXWWHEELFRVALID_VALID', 1: 'ESP_VXWWHEELFRVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_raw_0x212_212::Esp_vxwwheelfrvalidType esp_vxwwheelfrvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_vxwWheelFLValid', 'enum': {0: 'ESP_VXWWHEELFLVALID_VALID', 1: 'ESP_VXWWHEELFLVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_raw_0x212_212::Esp_vxwwheelflvalidType esp_vxwwheelflvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x212', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x212(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_vxwWheelRawFL', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
  double esp_vxwwheelrawfl(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_ESP_RAW_0X212_212_H_
