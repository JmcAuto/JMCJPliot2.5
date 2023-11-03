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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_DRIVESTS_0X10A_10A_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_DRIVESTS_0X10A_10A_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwvcudrivests0x10a10a : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwvcudrivests0x10a10a();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x10A', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x10a(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x10A', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x10a(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VehicleError_Intervene', 'enum': {0: 'VCU_VEHICLEERROR_INTERVENE_NOTINTERVENE', 1: 'VCU_VEHICLEERROR_INTERVENE_INTERVENE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_drivests_0x10a_10a::Vcu_vehicleerror_interveneType vcu_vehicleerror_intervene(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_Torque_ControlStatus', 'enum': {0: 'VCU_TORQUE_CONTROLSTATUS_TEMPORARILY_INHIBIT', 1: 'VCU_TORQUE_CONTROLSTATUS_AVAILABLEFORCONTROL', 2: 'VCU_TORQUE_CONTROLSTATUS_ACTIVE', 3: 'VCU_TORQUE_CONTROLSTATUS_PERMANENTLY_FAILED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_drivests_0x10a_10a::Vcu_torque_controlstatusType vcu_torque_controlstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_TorqueDriver_Intervene', 'enum': {0: 'VCU_TORQUEDRIVER_INTERVENE_NOTINTERVENE', 1: 'VCU_TORQUEDRIVER_INTERVENE_INTERVENE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_drivests_0x10a_10a::Vcu_torquedriver_interveneType vcu_torquedriver_intervene(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_GearPos_ControlStatus', 'enum': {0: 'VCU_GEARPOS_CONTROLSTATUS_TEMPORARILY_INHIBIT', 1: 'VCU_GEARPOS_CONTROLSTATUS_AVAILABLEFORCONTROL', 2: 'VCU_GEARPOS_CONTROLSTATUS_ACTIVE', 3: 'VCU_GEARPOS_CONTROLSTATUS_PERMANENTLY_FAILED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_drivests_0x10a_10a::Vcu_gearpos_controlstatusType vcu_gearpos_controlstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_GearPosDriver_Intervene', 'enum': {0: 'VCU_GEARPOSDRIVER_INTERVENE_NOTINTERVENE', 1: 'VCU_GEARPOSDRIVER_INTERVENE_INTERVENE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_drivests_0x10a_10a::Vcu_gearposdriver_interveneType vcu_gearposdriver_intervene(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_Veh_STS', 'enum': {0: 'VCU_VEH_STS_NOTREADY', 1: 'VCU_VEH_STS_READY'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_drivests_0x10a_10a::Vcu_veh_stsType vcu_veh_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_AxleSpdSt', 'enum': {0: 'VCU_AXLESPDST_NOERROR', 1: 'VCU_AXLESPDST_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_drivests_0x10a_10a::Vcu_axlespdstType vcu_axlespdst(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_AxleSpd', 'offset': 0.0, 'precision': 0.5, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|16383]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rpm'}
  double vcu_axlespd(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_VCU_DRIVESTS_0X10A_10A_H_
