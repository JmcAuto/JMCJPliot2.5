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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_EMS_TQWHL_0X111_111_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_EMS_TQWHL_0X111_111_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwemstqwhl0x111111 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwemstqwhl0x111111();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'EMS_IndicatedRealEngTorqWhl', 'enum': {65535: 'EMS_INDICATEDREALENGTORQWHL_INVALID'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': -30000.0, 'physical_range': '[-30000|30000]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  int ems_indicatedrealengtorqwhl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_IndicatedACCMesgError', 'enum': {0: 'EMS_INDICATEDACCMESGERROR_NOEERROR', 1: 'EMS_INDICATEDACCMESGERROR_REVERSIBLE_ERROR', 2: 'EMS_INDICATEDACCMESGERROR_IRREVERSIBLE_ERROR', 3: 'EMS_INDICATEDACCMESGERROR_NOTDEFINED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_tqwhl_0x111_111::Ems_indicatedaccmesgerrorType ems_indicatedaccmesgerror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_IndicatedDriverOverride', 'enum': {0: 'EMS_INDICATEDDRIVEROVERRIDE_NOOVERRIDE', 1: 'EMS_INDICATEDDRIVEROVERRIDE_DRIVEROVERRIDE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_tqwhl_0x111_111::Ems_indicateddriveroverrideType ems_indicateddriveroverride(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_IndicatedDriverReqTorq', 'enum': {2047: 'EMS_INDICATEDDRIVERREQTORQ_INVALID'}, 'precision': 0.5, 'len': 12, 'is_signed_var': False, 'offset': -1000.0, 'physical_range': '[-1000|1000]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  double ems_indicateddriverreqtorq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DCM_EMS_RollingCounter_0x111', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int dcm_ems_rollingcounter_0x111(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DCM_EMS_Checksum_0x111', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int dcm_ems_checksum_0x111(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_IndicatedDriverReqTorqWhl', 'enum': {65535: 'EMS_INDICATEDDRIVERREQTORQWHL_INVALID'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': -30000.0, 'physical_range': '[-30000|30000]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  int ems_indicateddriverreqtorqwhl(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_EMS_TQWHL_0X111_111_H_
