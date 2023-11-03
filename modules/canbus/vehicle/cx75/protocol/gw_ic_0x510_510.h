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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_IC_0X510_510_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_IC_0X510_510_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwic0x510510 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwic0x510510();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'IC_AirbagTelltaleBehavior', 'enum': {0: 'IC_AIRBAGTELLTALEBEHAVIOR_NO_FAILURE_IN_LAMP_AND_LAMP_IS_OFF', 1: 'IC_AIRBAGTELLTALEBEHAVIOR_FAILURE_IN_LAMP', 2: 'IC_AIRBAGTELLTALEBEHAVIOR_NO_FAILURE_IN_THE_LAMP_LAMP_IS_ON', 3: 'IC_AIRBAGTELLTALEBEHAVIOR_NO_FAILURE_IN_THE_LAMP_LAMP_IS_BLINKING', 4: 'IC_AIRBAGTELLTALEBEHAVIOR_AIRBAGFAILSTS_SIGNAL_NOT_RECEIVED', 5: 'IC_AIRBAGTELLTALEBEHAVIOR_INVALID', 6: 'IC_AIRBAGTELLTALEBEHAVIOR_INVALID', 7: 'IC_AIRBAGTELLTALEBEHAVIOR_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 24, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ic_0x510_510::Ic_airbagtelltalebehaviorType ic_airbagtelltalebehavior(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_VehBrkPump_ERR_IC', 'enum': {0: 'IC_VEHBRKPUMP_ERR_IC_NORMAL', 1: 'IC_VEHBRKPUMP_ERR_IC_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ic_0x510_510::Ic_vehbrkpump_err_icType ic_vehbrkpump_err_ic(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_DISFail', 'enum': {0: 'IC_DISFAIL_NO_ERROR', 1: 'IC_DISFAIL_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ic_0x510_510::Ic_disfailType ic_disfail(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_QDashACCFail', 'enum': {0: 'IC_QDASHACCFAIL_NO_ERROR', 1: 'IC_QDASHACCFAIL_REVERSIBLE_ERROR', 2: 'IC_QDASHACCFAIL_IRREVERSIBLE_ERROR', 3: 'IC_QDASHACCFAIL_NOT_DEFINED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ic_0x510_510::Ic_qdashaccfailType ic_qdashaccfail(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_VehSpd_HMI', 'enum': {511: 'IC_VEHSPD_HMI_INVALID'}, 'precision': 1.0, 'len': 9, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|500]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'kph'}
  int ic_vehspd_hmi(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_Rolling_counter_0x510', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ic_rolling_counter_0x510(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_Checksum_0x510', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ic_checksum_0x510(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_OdometerMasterValue', 'enum': {16777215: 'IC_ODOMETERMASTERVALUE_INVALID'}, 'precision': 0.1, 'len': 24, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|999999]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'km'}
  double ic_odometermastervalue(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_IC_0X510_510_H_
