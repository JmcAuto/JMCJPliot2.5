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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_IBC_STATUS2_0X124_124_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_IBC_STATUS2_0X124_124_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Ibcstatus20x124124 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Ibcstatus20x124124();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x124', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x124(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x124', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x124(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_DecelerationVaild', 'enum': {0: 'IBC_DECELERATIONVAILD_INVAILD', 1: 'IBC_DECELERATIONVAILD_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ibc_status2_0x124_124::Ibc_decelerationvaildType ibc_decelerationvaild(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_Deceleration', 'offset': 0.0, 'precision': 0.00159276, 'len': 12, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'g'}
  double ibc_deceleration(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_BrkTMCPositionVaild', 'enum': {0: 'IBC_BRKTMCPOSITIONVAILD_INVAILD', 1: 'IBC_BRKTMCPOSITIONVAILD_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ibc_status2_0x124_124::Ibc_brktmcpositionvaildType ibc_brktmcpositionvaild(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IBC_BrkTMCPosition', 'offset': 0.0, 'precision': 0.1, 'len': 10, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  double ibc_brktmcposition(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_IBC_STATUS2_0X124_124_H_
