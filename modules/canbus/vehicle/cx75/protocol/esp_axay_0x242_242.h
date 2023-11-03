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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_AXAY_0X242_242_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_AXAY_0X242_242_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Espaxay0x242242 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Espaxay0x242242();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ESP_Alat', 'offset': -21.593092, 'precision': 0.027127, 'len': 12, 'is_signed_var': False, 'physical_range': '[-21.593092|21.593092]', 'bit': 11, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s^2'}
  double esp_alat(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_YawRate', 'offset': -2.0942132, 'precision': 0.0021326, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2.0942132|2.0942132]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad s'}
  double esp_yawrate(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x242', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x242(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_YawRateStatus', 'enum': {0: 'ESP_YAWRATESTATUS_OK', 1: 'ESP_YAWRATESTATUS_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_axay_0x242_242::Esp_yawratestatusType esp_yawratestatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_AlatStatus', 'enum': {0: 'ESP_ALATSTATUS_OK', 1: 'ESP_ALATSTATUS_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_axay_0x242_242::Esp_alatstatusType esp_alatstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_AlgtStatus', 'enum': {0: 'ESP_ALGTSTATUS_OK', 1: 'ESP_ALGTSTATUS_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_axay_0x242_242::Esp_algtstatusType esp_algtstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x242', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x242(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_Algt', 'offset': -21.593092, 'precision': 0.027127, 'len': 12, 'is_signed_var': False, 'physical_range': '[-21.593092|21.593092]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s^2'}
  double esp_algt(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_ESP_AXAY_0X242_242_H_
