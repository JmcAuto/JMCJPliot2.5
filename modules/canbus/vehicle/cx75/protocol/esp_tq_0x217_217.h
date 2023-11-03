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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_ESP_TQ_0X217_217_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_ESP_TQ_0X217_217_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Esptq0x217217 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Esptq0x217217();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ESP_TqDecReqdByEspSlow', 'offset': -1000.0, 'precision': 0.5, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|1000]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
  double esp_tqdecreqdbyespslow(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_TqInccReqdByEsp', 'offset': -1000.0, 'precision': 0.5, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|1000]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
  double esp_tqinccreqdbyesp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x217', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x217(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_TrqInc_Req', 'enum': {0: 'ESP_TRQINC_REQ_INACTIVE', 1: 'ESP_TRQINC_REQ_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_tq_0x217_217::Esp_trqinc_reqType esp_trqinc_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_TrqRed_Req', 'enum': {0: 'ESP_TRQRED_REQ_INACTIVE', 1: 'ESP_TRQRED_REQ_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_tq_0x217_217::Esp_trqred_reqType esp_trqred_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x217', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x217(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_TqDecReqdByEspFast', 'offset': -1000.0, 'precision': 0.5, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|1000]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
  double esp_tqdecreqdbyespfast(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_ESP_TQ_0X217_217_H_
