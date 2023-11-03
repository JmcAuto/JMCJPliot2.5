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

#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_drivests_0x10a_10a.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwvcudrivests0x10a10a::Gwvcudrivests0x10a10a() {}
const int32_t Gwvcudrivests0x10a10a::ID = 0x10A;

void Gwvcudrivests0x10a10a::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_vcu_drivests_0x10a_10a()->set_checksum_0x10a(checksum_0x10a(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_drivests_0x10a_10a()->set_rolling_counter_0x10a(rolling_counter_0x10a(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_drivests_0x10a_10a()->set_vcu_vehicleerror_intervene(vcu_vehicleerror_intervene(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_drivests_0x10a_10a()->set_vcu_torque_controlstatus(vcu_torque_controlstatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_drivests_0x10a_10a()->set_vcu_torquedriver_intervene(vcu_torquedriver_intervene(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_drivests_0x10a_10a()->set_vcu_gearpos_controlstatus(vcu_gearpos_controlstatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_drivests_0x10a_10a()->set_vcu_gearposdriver_intervene(vcu_gearposdriver_intervene(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_drivests_0x10a_10a()->set_vcu_veh_sts(vcu_veh_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_drivests_0x10a_10a()->set_vcu_axlespdst(vcu_axlespdst(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_drivests_0x10a_10a()->set_vcu_axlespd(vcu_axlespd(bytes, length));

chassis->mutable_check_response()->set_is_vcu_online(
      vcu_torque_controlstatus(bytes, length) == 2);

  chassis->mutable_check_response()->set_is_switch_online(
      vcu_gearpos_controlstatus(bytes, length) == 2);
}

// config detail: {'name': 'checksum_0x10a', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcudrivests0x10a10a::checksum_0x10a(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x10a', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcudrivests0x10a10a::rolling_counter_0x10a(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_vehicleerror_intervene', 'enum': {0: 'VCU_VEHICLEERROR_INTERVENE_NOTINTERVENE', 1: 'VCU_VEHICLEERROR_INTERVENE_INTERVENE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_drivests_0x10a_10a::Vcu_vehicleerror_interveneType Gwvcudrivests0x10a10a::vcu_vehicleerror_intervene(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Gw_vcu_drivests_0x10a_10a::Vcu_vehicleerror_interveneType ret =  static_cast<Gw_vcu_drivests_0x10a_10a::Vcu_vehicleerror_interveneType>(x);
  return ret;
}

// config detail: {'name': 'vcu_torque_controlstatus', 'enum': {0: 'VCU_TORQUE_CONTROLSTATUS_TEMPORARILY_INHIBIT', 1: 'VCU_TORQUE_CONTROLSTATUS_AVAILABLEFORCONTROL', 2: 'VCU_TORQUE_CONTROLSTATUS_ACTIVE', 3: 'VCU_TORQUE_CONTROLSTATUS_PERMANENTLY_FAILED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_drivests_0x10a_10a::Vcu_torque_controlstatusType Gwvcudrivests0x10a10a::vcu_torque_controlstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 2);

  Gw_vcu_drivests_0x10a_10a::Vcu_torque_controlstatusType ret =  static_cast<Gw_vcu_drivests_0x10a_10a::Vcu_torque_controlstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_torquedriver_intervene', 'enum': {0: 'VCU_TORQUEDRIVER_INTERVENE_NOTINTERVENE', 1: 'VCU_TORQUEDRIVER_INTERVENE_INTERVENE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_drivests_0x10a_10a::Vcu_torquedriver_interveneType Gwvcudrivests0x10a10a::vcu_torquedriver_intervene(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Gw_vcu_drivests_0x10a_10a::Vcu_torquedriver_interveneType ret =  static_cast<Gw_vcu_drivests_0x10a_10a::Vcu_torquedriver_interveneType>(x);
  return ret;
}

// config detail: {'name': 'vcu_gearpos_controlstatus', 'enum': {0: 'VCU_GEARPOS_CONTROLSTATUS_TEMPORARILY_INHIBIT', 1: 'VCU_GEARPOS_CONTROLSTATUS_AVAILABLEFORCONTROL', 2: 'VCU_GEARPOS_CONTROLSTATUS_ACTIVE', 3: 'VCU_GEARPOS_CONTROLSTATUS_PERMANENTLY_FAILED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_drivests_0x10a_10a::Vcu_gearpos_controlstatusType Gwvcudrivests0x10a10a::vcu_gearpos_controlstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 2);

  Gw_vcu_drivests_0x10a_10a::Vcu_gearpos_controlstatusType ret =  static_cast<Gw_vcu_drivests_0x10a_10a::Vcu_gearpos_controlstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_gearposdriver_intervene', 'enum': {0: 'VCU_GEARPOSDRIVER_INTERVENE_NOTINTERVENE', 1: 'VCU_GEARPOSDRIVER_INTERVENE_INTERVENE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_drivests_0x10a_10a::Vcu_gearposdriver_interveneType Gwvcudrivests0x10a10a::vcu_gearposdriver_intervene(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Gw_vcu_drivests_0x10a_10a::Vcu_gearposdriver_interveneType ret =  static_cast<Gw_vcu_drivests_0x10a_10a::Vcu_gearposdriver_interveneType>(x);
  return ret;
}

// config detail: {'name': 'vcu_veh_sts', 'enum': {0: 'VCU_VEH_STS_NOTREADY', 1: 'VCU_VEH_STS_READY'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_drivests_0x10a_10a::Vcu_veh_stsType Gwvcudrivests0x10a10a::vcu_veh_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  Gw_vcu_drivests_0x10a_10a::Vcu_veh_stsType ret =  static_cast<Gw_vcu_drivests_0x10a_10a::Vcu_veh_stsType>(x);
  return ret;
}

// config detail: {'name': 'vcu_axlespdst', 'enum': {0: 'VCU_AXLESPDST_NOERROR', 1: 'VCU_AXLESPDST_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_drivests_0x10a_10a::Vcu_axlespdstType Gwvcudrivests0x10a10a::vcu_axlespdst(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Gw_vcu_drivests_0x10a_10a::Vcu_axlespdstType ret =  static_cast<Gw_vcu_drivests_0x10a_10a::Vcu_axlespdstType>(x);
  return ret;
}

// config detail: {'name': 'vcu_axlespd', 'offset': 0.0, 'precision': 0.5, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|16383]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rpm'}
double Gwvcudrivests0x10a10a::vcu_axlespd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  double ret = x * 0.500000;
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
