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

#include "modules/canbus/vehicle/teshun/protocol/eps2_status_0x112_112.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Eps2status0x112112::Eps2status0x112112() {}
const int32_t Eps2status0x112112::ID = 0x112;

void Eps2status0x112112::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_eps2_status_0x112_112()->set_eps_controlstatus(eps_controlstatus(bytes, length));
  chassis->mutable_teshun()->mutable_eps2_status_0x112_112()->set_eps_driver_intervene(eps_driver_intervene(bytes, length));
  chassis->mutable_teshun()->mutable_eps2_status_0x112_112()->set_eps_steerwheelrotspd(eps_steerwheelrotspd(bytes, length));
  chassis->mutable_teshun()->mutable_eps2_status_0x112_112()->set_eps_steeringwheelang(eps_steeringwheelang(bytes, length));
  chassis->mutable_teshun()->mutable_eps2_status_0x112_112()->set_eps_torsionbartorque(eps_torsionbartorque(bytes, length));
  chassis->mutable_teshun()->mutable_eps2_status_0x112_112()->set_eps_sasfailurests(eps_sasfailurests(bytes, length));
  chassis->mutable_teshun()->mutable_eps2_status_0x112_112()->set_eps_torsionbartorquedir(eps_torsionbartorquedir(bytes, length));
  chassis->mutable_teshun()->mutable_eps2_status_0x112_112()->set_eps_torsionbartorquevalid(eps_torsionbartorquevalid(bytes, length));

chassis->mutable_check_response()->set_is_eps_online(
      eps_controlstatus(bytes, length) == 2|| eps_controlstatus(bytes, length) == 0);
}

// config detail: {'name': 'eps_controlstatus', 'enum': {0: 'EPS_CONTROLSTATUS_TEMPORARILY_INHIBIT', 1: 'EPS_CONTROLSTATUS_AVAILABLE_FOR_CONTROL', 2: 'EPS_CONTROLSTATUS_ACTIVE', 3: 'EPS_CONTROLSTATUS_PERMANENTLY_FAILED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 62, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps2_status_0x112_112::Eps_controlstatusType Eps2status0x112112::eps_controlstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(5, 2);

  Eps2_status_0x112_112::Eps_controlstatusType ret =  static_cast<Eps2_status_0x112_112::Eps_controlstatusType>(x);
  return ret;
}

// config detail: {'name': 'eps_driver_intervene', 'enum': {0: 'EPS_DRIVER_INTERVENE_NOT_INTERVENE', 1: 'EPS_DRIVER_INTERVENE_INTERVENE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 56, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps2_status_0x112_112::Eps_driver_interveneType Eps2status0x112112::eps_driver_intervene(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 1);

  Eps2_status_0x112_112::Eps_driver_interveneType ret =  static_cast<Eps2_status_0x112_112::Eps_driver_interveneType>(x);
  return ret;
}

// config detail: {'name': 'eps_steerwheelrotspd', 'offset': 0.0, 'precision': 1.0, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|511]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg/s'}
int Eps2status0x112112::eps_steerwheelrotspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'eps_steeringwheelang', 'offset': -1000.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|1000]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'degree'}
double Eps2status0x112112::eps_steeringwheelang(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -1000.000000;
  return ret;
}

// config detail: {'name': 'eps_torsionbartorque', 'offset': 0.0, 'precision': 0.01, 'len': 10, 'is_signed_var': False, 'physical_range': '[0|8]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
double Eps2status0x112112::eps_torsionbartorque(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'eps_sasfailurests', 'enum': {0: 'EPS_SASFAILURESTS_SENSOR_INFORMATION_INVALID__AN_INTERNAL_SENSOR_FAULT_OCCURRED', 1: 'EPS_SASFAILURESTS_SENSOR_INFORMATION_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps2_status_0x112_112::Eps_sasfailurestsType Eps2status0x112112::eps_sasfailurests(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  Eps2_status_0x112_112::Eps_sasfailurestsType ret =  static_cast<Eps2_status_0x112_112::Eps_sasfailurestsType>(x);
  return ret;
}

// config detail: {'name': 'eps_torsionbartorquedir', 'enum': {0: 'EPS_TORSIONBARTORQUEDIR_POSITIVE', 1: 'EPS_TORSIONBARTORQUEDIR_NEGATIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps2_status_0x112_112::Eps_torsionbartorquedirType Eps2status0x112112::eps_torsionbartorquedir(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Eps2_status_0x112_112::Eps_torsionbartorquedirType ret =  static_cast<Eps2_status_0x112_112::Eps_torsionbartorquedirType>(x);
  return ret;
}

// config detail: {'name': 'eps_torsionbartorquevalid', 'enum': {0: 'EPS_TORSIONBARTORQUEVALID_INVALID', 1: 'EPS_TORSIONBARTORQUEVALID_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps2_status_0x112_112::Eps_torsionbartorquevalidType Eps2status0x112112::eps_torsionbartorquevalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  Eps2_status_0x112_112::Eps_torsionbartorquevalidType ret =  static_cast<Eps2_status_0x112_112::Eps_torsionbartorquevalidType>(x);
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
