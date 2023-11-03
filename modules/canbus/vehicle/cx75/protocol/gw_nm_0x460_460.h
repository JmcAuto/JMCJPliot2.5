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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_GW_NM_0X460_460_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_GW_NM_0X460_460_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwnm0x460460 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwnm0x460460();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'GWNMLimphome', 'enum': {0: 'GWNMLIMPHOME_NOT_ACTIVE', 1: 'GWNMLIMPHOME_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 10, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_nm_0x460_460::GwnmlimphomeType gwnmlimphome(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'GWNMSleepInd', 'enum': {0: 'GWNMSLEEPIND_NOT_ACTIVE', 1: 'GWNMSLEEPIND_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_nm_0x460_460::GwnmsleepindType gwnmsleepind(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'GWNMSleepAck', 'enum': {0: 'GWNMSLEEPACK_NOT_ACTIVE', 1: 'GWNMSLEEPACK_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_nm_0x460_460::GwnmsleepackType gwnmsleepack(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'GWNMDestAddr', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int gwnmdestaddr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'GWNMAlive', 'enum': {0: 'GWNMALIVE_NOT_ACTIVE', 1: 'GWNMALIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_nm_0x460_460::GwnmaliveType gwnmalive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'GWNMRing', 'enum': {0: 'GWNMRING_NOT_ACTIVE', 1: 'GWNMRING_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_nm_0x460_460::GwnmringType gwnmring(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_GW_NM_0X460_460_H_
