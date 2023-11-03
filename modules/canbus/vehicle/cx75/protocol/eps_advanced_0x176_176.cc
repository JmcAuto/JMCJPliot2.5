/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/cx75/protocol/eps_advanced_0x176_176.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Epsadvanced0x176176::Epsadvanced0x176176() {}
const int32_t Epsadvanced0x176176::ID = 0x176;

void Epsadvanced0x176176::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_eps_lkaresponsetorque(eps_lkaresponsetorque(bytes, length));
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_eps_tosionbartorquevalid(eps_tosionbartorquevalid(bytes, length));
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_eps_lkaresponsetorquevalid(eps_lkaresponsetorquevalid(bytes, length));
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_eps_ldwcontrolstatus(eps_ldwcontrolstatus(bytes, length));
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_eps_pam_steeringsts_reserved(eps_pam_steeringsts_reserved(bytes, length));
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_eps_lkacontrolstatus(eps_lkacontrolstatus(bytes, length));
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_eps_epspaminh(eps_epspaminh(bytes, length));
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_rolling_counter_0x176(rolling_counter_0x176(bytes, length));
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_eps_epspamsts(eps_epspamsts(bytes, length));
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_checksum_0x176(checksum_0x176(bytes, length));
  chassis->mutable_cx75()->mutable_eps_advanced_0x176_176()->set_eps_torsionbartorque(eps_torsionbartorque(bytes, length));
  chassis->mutable_check_response()->set_is_eps_online(
      eps_lkacontrolstatus(bytes, length) == 1);
  chassis->mutable_check_response()->set_is_epspam_online(
      eps_epspamsts(bytes, length) == 2||eps_epspamsts(bytes, length) == 4);
}

// config detail: {'name': 'eps_lkaresponsetorque', 'enum': {2047: 'EPS_LKARESPONSETORQUE_INVALID'}, 'precision': 0.01, 'len': 11, 'is_signed_var': False, 'offset': -10.24, 'physical_range': '[-10.24|10.22]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
double Epsadvanced0x176176::eps_lkaresponsetorque(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  double ret =  static_cast<double>(x);
  ret=ret*0.01-10.24;
  return ret;
}

// config detail: {'name': 'eps_tosionbartorquevalid', 'enum': {0: 'EPS_TOSIONBARTORQUEVALID_VAILD', 1: 'EPS_TOSIONBARTORQUEVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_advanced_0x176_176::Eps_tosionbartorquevalidType Epsadvanced0x176176::eps_tosionbartorquevalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  Eps_advanced_0x176_176::Eps_tosionbartorquevalidType ret =  static_cast<Eps_advanced_0x176_176::Eps_tosionbartorquevalidType>(x);
  return ret;
}

// config detail: {'name': 'eps_lkaresponsetorquevalid', 'enum': {0: 'EPS_LKARESPONSETORQUEVALID_VAILD', 1: 'EPS_LKARESPONSETORQUEVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_advanced_0x176_176::Eps_lkaresponsetorquevalidType Epsadvanced0x176176::eps_lkaresponsetorquevalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Eps_advanced_0x176_176::Eps_lkaresponsetorquevalidType ret =  static_cast<Eps_advanced_0x176_176::Eps_lkaresponsetorquevalidType>(x);
  return ret;
}

// config detail: {'name': 'eps_ldwcontrolstatus', 'enum': {0: 'EPS_LDWCONTROLSTATUS_DEACTIVATED', 1: 'EPS_LDWCONTROLSTATUS_INACTIVE', 2: 'EPS_LDWCONTROLSTATUS_ACTIVE', 3: 'EPS_LDWCONTROLSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_advanced_0x176_176::Eps_ldwcontrolstatusType Epsadvanced0x176176::eps_ldwcontrolstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 2);

  Eps_advanced_0x176_176::Eps_ldwcontrolstatusType ret =  static_cast<Eps_advanced_0x176_176::Eps_ldwcontrolstatusType>(x);
  return ret;
}

// config detail: {'name': 'eps_pam_steeringsts_reserved', 'enum': {0: 'EPS_PAM_STEERINGSTS_RESERVED_STEERING_POSITION_IS_OK', 1: 'EPS_PAM_STEERINGSTS_RESERVED_STEERING_POSITION_IS_ADJUSTING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_advanced_0x176_176::Eps_pam_steeringsts_reservedType Epsadvanced0x176176::eps_pam_steeringsts_reserved(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  Eps_advanced_0x176_176::Eps_pam_steeringsts_reservedType ret =  static_cast<Eps_advanced_0x176_176::Eps_pam_steeringsts_reservedType>(x);
  return ret;
}

// config detail: {'name': 'eps_lkacontrolstatus', 'enum': {0: 'EPS_LKACONTROLSTATUS_NO_REQUEST', 1: 'EPS_LKACONTROLSTATUS_REQUEST_HONORED', 2: 'EPS_LKACONTROLSTATUS_CONTROL_REQUEST_NOT_ALLOWED_TEMPORARILY', 3: 'EPS_LKACONTROLSTATUS_CONTROL_REQUEST_NOT_ALLOWED_PERMANENT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_advanced_0x176_176::Eps_lkacontrolstatusType Epsadvanced0x176176::eps_lkacontrolstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(6, 2);

  Eps_advanced_0x176_176::Eps_lkacontrolstatusType ret =  static_cast<Eps_advanced_0x176_176::Eps_lkacontrolstatusType>(x);
  return ret;
}

// config detail: {'name': 'eps_epspaminh', 'enum': {0: 'EPS_EPSPAMINH_NORMAL_OPERATION', 1: 'EPS_EPSPAMINH_OVER_SPEED', 2: 'EPS_EPSPAMINH_DRIVER_INTERFERENCE', 4: 'EPS_EPSPAMINH_ABNORMAL_CAN_INPUT', 16: 'EPS_EPSPAMINH_EPS_FAILURE', 8: 'EPS_EPSPAMINH_EXCESS_ANGLE_DEVIATION'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
//Eps_advanced_0x176_176::Eps_epspaminhType Epsadvanced0x176176::eps_epspaminh(const std::uint8_t* bytes, int32_t length) const {
//  Byte t0(bytes + 4);
//  int32_t x = t0.get_byte(0, 8);

//  Eps_advanced_0x176_176::Eps_epspaminhType ret =  static_cast<Eps_advanced_0x176_176::Eps_epspaminhType>(x);
//  return ret;
//}

int32_t Epsadvanced0x176176::eps_epspaminh(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int32_t ret = (x);
  return ret;
}


// config detail: {'name': 'rolling_counter_0x176', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Epsadvanced0x176176::rolling_counter_0x176(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'eps_epspamsts', 'enum': {0: 'EPS_EPSPAMSTS_INHIBIT', 1: 'EPS_EPSPAMSTS_AVAILABLE', 2: 'EPS_EPSPAMSTS_ACTIVE', 3: 'EPS_EPSPAMSTS_ABORT', 4: 'EPS_EPSPAMSTS_REPLY_FOR_CONTROL', 5: 'EPS_EPSPAMSTS_NRCD_ACTIVE', 6: 'EPS_EPSPAMSTS_ADAS_ACTIVE', 7: 'EPS_EPSPAMSTS_FAILURE'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_advanced_0x176_176::Eps_epspamstsType Epsadvanced0x176176::eps_epspamsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 4);

  Eps_advanced_0x176_176::Eps_epspamstsType ret =  static_cast<Eps_advanced_0x176_176::Eps_epspamstsType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x176', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Epsadvanced0x176176::checksum_0x176(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'eps_torsionbartorque', 'offset': -10.24, 'precision': 0.01, 'len': 11, 'is_signed_var': False, 'physical_range': '[-10.24|10.22]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
double Epsadvanced0x176176::eps_torsionbartorque(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;

  double ret = x * 0.010000 + -10.240000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
