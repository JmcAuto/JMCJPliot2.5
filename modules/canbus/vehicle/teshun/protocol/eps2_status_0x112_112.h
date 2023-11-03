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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_EPS2_STATUS_0X112_112_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_EPS2_STATUS_0X112_112_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Eps2status0x112112 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Eps2status0x112112();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'EPS_ControlStatus', 'enum': {0: 'EPS_CONTROLSTATUS_TEMPORARILY_INHIBIT', 1: 'EPS_CONTROLSTATUS_AVAILABLE_FOR_CONTROL', 2: 'EPS_CONTROLSTATUS_ACTIVE', 3: 'EPS_CONTROLSTATUS_PERMANENTLY_FAILED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 62, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps2_status_0x112_112::Eps_controlstatusType eps_controlstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_Driver_Intervene', 'enum': {0: 'EPS_DRIVER_INTERVENE_NOT_INTERVENE', 1: 'EPS_DRIVER_INTERVENE_INTERVENE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 56, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps2_status_0x112_112::Eps_driver_interveneType eps_driver_intervene(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_SteerWheelRotSpd', 'offset': 0.0, 'precision': 1.0, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|511]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg/s'}
  int eps_steerwheelrotspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_SteeringWheelAng', 'offset': -1000.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|1000]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'degree'}
  double eps_steeringwheelang(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_TorsionBarTorque', 'offset': 0.0, 'precision': 0.01, 'len': 10, 'is_signed_var': False, 'physical_range': '[0|8]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
  double eps_torsionbartorque(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_SASFailureSts', 'enum': {0: 'EPS_SASFAILURESTS_SENSOR_INFORMATION_INVALID__AN_INTERNAL_SENSOR_FAULT_OCCURRED', 1: 'EPS_SASFAILURESTS_SENSOR_INFORMATION_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps2_status_0x112_112::Eps_sasfailurestsType eps_sasfailurests(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_TorsionBarTorqueDir', 'enum': {0: 'EPS_TORSIONBARTORQUEDIR_POSITIVE', 1: 'EPS_TORSIONBARTORQUEDIR_NEGATIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps2_status_0x112_112::Eps_torsionbartorquedirType eps_torsionbartorquedir(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_TorsionBarTorqueValid', 'enum': {0: 'EPS_TORSIONBARTORQUEVALID_INVALID', 1: 'EPS_TORSIONBARTORQUEVALID_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps2_status_0x112_112::Eps_torsionbartorquevalidType eps_torsionbartorquevalid(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_EPS2_STATUS_0X112_112_H_
