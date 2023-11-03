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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_SOD_NM_0X440_440_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_SOD_NM_0X440_440_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Sodnm0x440440 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Sodnm0x440440();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'SODNMLimphome', 'enum': {0: 'SODNMLIMPHOME_NOT_ACTIVE', 1: 'SODNMLIMPHOME_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 10, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_nm_0x440_440::SodnmlimphomeType sodnmlimphome(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SODNMSleepInd', 'enum': {0: 'SODNMSLEEPIND_NOT_ACTIVE', 1: 'SODNMSLEEPIND_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_nm_0x440_440::SodnmsleepindType sodnmsleepind(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SODNMSleepAck', 'enum': {0: 'SODNMSLEEPACK_NOT_ACTIVE', 1: 'SODNMSLEEPACK_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_nm_0x440_440::SodnmsleepackType sodnmsleepack(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SODNMDestAddr', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int sodnmdestaddr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SODNMAlive', 'enum': {0: 'SODNMALIVE_NOT_ACTIVE', 1: 'SODNMALIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_nm_0x440_440::SodnmaliveType sodnmalive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SODNMRing', 'enum': {0: 'SODNMRING_NOT_ACTIVE', 1: 'SODNMRING_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_nm_0x440_440::SodnmringType sodnmring(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_SOD_NM_0X440_440_H_
