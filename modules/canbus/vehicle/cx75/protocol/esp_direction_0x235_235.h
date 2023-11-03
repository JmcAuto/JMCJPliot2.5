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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_DIRECTION_0X235_235_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_DIRECTION_0X235_235_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Espdirection0x235235 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Espdirection0x235235();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ESP_WhlMilgRearRe', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': 'edge'}
  int esp_whlmilgrearre(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_FRWheelDriveDirStatus', 'enum': {0: 'ESP_FRWHEELDRIVEDIRSTATUS_VALID', 1: 'ESP_FRWHEELDRIVEDIRSTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_frwheeldrivedirstatusType esp_frwheeldrivedirstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_FLWheelDriveDirStatus', 'enum': {0: 'ESP_FLWHEELDRIVEDIRSTATUS_VALID', 1: 'ESP_FLWHEELDRIVEDIRSTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_flwheeldrivedirstatusType esp_flwheeldrivedirstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_RRWheelDriveDirStatus', 'enum': {0: 'ESP_RRWHEELDRIVEDIRSTATUS_VALID', 1: 'ESP_RRWHEELDRIVEDIRSTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_rrwheeldrivedirstatusType esp_rrwheeldrivedirstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_RLWheelDriveDirStatus', 'enum': {0: 'ESP_RLWHEELDRIVEDIRSTATUS_VALID', 1: 'ESP_RLWHEELDRIVEDIRSTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_rlwheeldrivedirstatusType esp_rlwheeldrivedirstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_RRWheelDriveDirection', 'enum': {0: 'ESP_RRWHEELDRIVEDIRECTION_UNDEFINED', 1: 'ESP_RRWHEELDRIVEDIRECTION_STANDSTILL', 2: 'ESP_RRWHEELDRIVEDIRECTION_FORWARD', 3: 'ESP_RRWHEELDRIVEDIRECTION_BACKWARD'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_rrwheeldrivedirectionType esp_rrwheeldrivedirection(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_RLWheelDriveDirection', 'enum': {0: 'ESP_RLWHEELDRIVEDIRECTION_UNDEFINED', 1: 'ESP_RLWHEELDRIVEDIRECTION_STANDSTILL', 2: 'ESP_RLWHEELDRIVEDIRECTION_FORWARD', 3: 'ESP_RLWHEELDRIVEDIRECTION_BACKWARD'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 43, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_rlwheeldrivedirectionType esp_rlwheeldrivedirection(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_FRWheelDriveDirection', 'enum': {0: 'ESP_FRWHEELDRIVEDIRECTION_UNDEFINED', 1: 'ESP_FRWHEELDRIVEDIRECTION_STANDSTILL', 2: 'ESP_FRWHEELDRIVEDIRECTION_FORWARD', 3: 'ESP_FRWHEELDRIVEDIRECTION_BACKWARD'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_frwheeldrivedirectionType esp_frwheeldrivedirection(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_FLWheelDriveDirection', 'enum': {0: 'ESP_FLWHEELDRIVEDIRECTION_UNDEFINED', 1: 'ESP_FLWHEELDRIVEDIRECTION_STANDSTILL', 2: 'ESP_FLWHEELDRIVEDIRECTION_FORWARD', 3: 'ESP_FLWHEELDRIVEDIRECTION_BACKWARD'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_flwheeldrivedirectionType esp_flwheeldrivedirection(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x235', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x235(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WhlMilgReaRLeStatus', 'enum': {0: 'ESP_WHLMILGREARLESTATUS_VALID', 1: 'ESP_WHLMILGREARLESTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_whlmilgrearlestatusType esp_whlmilgrearlestatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WhlMilgReaRRiStatus', 'enum': {0: 'ESP_WHLMILGREARRISTATUS_VALID', 1: 'ESP_WHLMILGREARRISTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_whlmilgrearristatusType esp_whlmilgrearristatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_VehicleStandstill', 'enum': {0: 'ESP_VEHICLESTANDSTILL_NOT_STANDSTILL', 1: 'ESP_VEHICLESTANDSTILL_STANDSTILL', 2: 'ESP_VEHICLESTANDSTILL_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_direction_0x235_235::Esp_vehiclestandstillType esp_vehiclestandstill(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x235', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x235(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_WhlMilgRearLe', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'edge'}
  int esp_whlmilgrearle(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_ESP_DIRECTION_0X235_235_H_
