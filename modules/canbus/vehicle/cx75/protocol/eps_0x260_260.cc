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

#include "modules/canbus/vehicle/cx75/protocol/eps_0x260_260.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Eps0x260260::Eps0x260260() {}
const int32_t Eps0x260260::ID = 0x260;

void Eps0x260260::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_sassoftlimitrightflg(eps_sassoftlimitrightflg(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_n_loadinfo(eps_n_loadinfo(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_st_emergencypowerlimit(eps_st_emergencypowerlimit(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_st_emergencymotor(eps_st_emergencymotor(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_st_emergencyecu(eps_st_emergencyecu(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_st_emergencycaninterface(eps_st_emergencycaninterface(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_st_emergencysensor(eps_st_emergencysensor(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_st_emergencyovertemp(eps_st_emergencyovertemp(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_st_emergencyovervoltage(eps_st_emergencyovervoltage(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_st_emergencyundervoltage(eps_st_emergencyundervoltage(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_f_ecutempvalid(eps_f_ecutempvalid(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_sassoftlimitleftflg(eps_sassoftlimitleftflg(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_n_ecutemp(eps_n_ecutemp(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_n_performanceredu(eps_n_performanceredu(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_st_dtcflag(eps_st_dtcflag(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_rolling_counter_0x260(rolling_counter_0x260(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_sasindexsts(eps_sasindexsts(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_s_warninglampyellow(eps_s_warninglampyellow(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_checksum_0x260(checksum_0x260(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_s_safelampred(eps_s_safelampred(bytes, length));
  chassis->mutable_cx75()->mutable_eps_0x260_260()->set_eps_f_loadinfo(eps_f_loadinfo(bytes, length));
  
}

// config detail: {'name': 'eps_sassoftlimitrightflg', 'enum': {0: 'EPS_SASSOFTLIMITRIGHTFLG_NO_LEARNED', 1: 'EPS_SASSOFTLIMITRIGHTFLG_PRIMARY_LEARNED_ONLY_FOR_CEPS', 2: 'EPS_SASSOFTLIMITRIGHTFLG_LEARNED', 3: 'EPS_SASSOFTLIMITRIGHTFLG_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_sassoftlimitrightflgType Eps0x260260::eps_sassoftlimitrightflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Eps_0x260_260::Eps_sassoftlimitrightflgType ret =  static_cast<Eps_0x260_260::Eps_sassoftlimitrightflgType>(x);
  return ret;
}

// config detail: {'name': 'eps_n_loadinfo', 'offset': 0.0, 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|127]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': 'A'}
int Eps0x260260::eps_n_loadinfo(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 7);

  int ret = x;
  return ret;
}

// config detail: {'name': 'eps_st_emergencypowerlimit', 'enum': {0: 'EPS_ST_EMERGENCYPOWERLIMIT_NORMAL', 1: 'EPS_ST_EMERGENCYPOWERLIMIT_POWER_DENSITY_LIMIT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_st_emergencypowerlimitType Eps0x260260::eps_st_emergencypowerlimit(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Eps_0x260_260::Eps_st_emergencypowerlimitType ret =  static_cast<Eps_0x260_260::Eps_st_emergencypowerlimitType>(x);
  return ret;
}

// config detail: {'name': 'eps_st_emergencymotor', 'enum': {0: 'EPS_ST_EMERGENCYMOTOR_NORMAL', 1: 'EPS_ST_EMERGENCYMOTOR_SERVO_MOTOR_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_st_emergencymotorType Eps0x260260::eps_st_emergencymotor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 1);

  Eps_0x260_260::Eps_st_emergencymotorType ret =  static_cast<Eps_0x260_260::Eps_st_emergencymotorType>(x);
  return ret;
}

// config detail: {'name': 'eps_st_emergencyecu', 'enum': {0: 'EPS_ST_EMERGENCYECU_NORMAL', 1: 'EPS_ST_EMERGENCYECU_ECU_INNER_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_st_emergencyecuType Eps0x260260::eps_st_emergencyecu(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 1);

  Eps_0x260_260::Eps_st_emergencyecuType ret =  static_cast<Eps_0x260_260::Eps_st_emergencyecuType>(x);
  return ret;
}

// config detail: {'name': 'eps_st_emergencycaninterface', 'enum': {0: 'EPS_ST_EMERGENCYCANINTERFACE_NORMAL', 1: 'EPS_ST_EMERGENCYCANINTERFACE_CAN_INTERFACE_IS_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_st_emergencycaninterfaceType Eps0x260260::eps_st_emergencycaninterface(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 1);

  Eps_0x260_260::Eps_st_emergencycaninterfaceType ret =  static_cast<Eps_0x260_260::Eps_st_emergencycaninterfaceType>(x);
  return ret;
}

// config detail: {'name': 'eps_st_emergencysensor', 'enum': {0: 'EPS_ST_EMERGENCYSENSOR_NORMAL', 1: 'EPS_ST_EMERGENCYSENSOR_SENSOR_IS_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_st_emergencysensorType Eps0x260260::eps_st_emergencysensor(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 1);

  Eps_0x260_260::Eps_st_emergencysensorType ret =  static_cast<Eps_0x260_260::Eps_st_emergencysensorType>(x);
  return ret;
}

// config detail: {'name': 'eps_st_emergencyovertemp', 'enum': {0: 'EPS_ST_EMERGENCYOVERTEMP_NORMAL', 1: 'EPS_ST_EMERGENCYOVERTEMP_OVER_TEMPERATURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_st_emergencyovertempType Eps0x260260::eps_st_emergencyovertemp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 1);

  Eps_0x260_260::Eps_st_emergencyovertempType ret =  static_cast<Eps_0x260_260::Eps_st_emergencyovertempType>(x);
  return ret;
}

// config detail: {'name': 'eps_st_emergencyovervoltage', 'enum': {0: 'EPS_ST_EMERGENCYOVERVOLTAGE_NORMAL', 1: 'EPS_ST_EMERGENCYOVERVOLTAGE_OVER_VOLTAGE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_st_emergencyovervoltageType Eps0x260260::eps_st_emergencyovervoltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 1);

  Eps_0x260_260::Eps_st_emergencyovervoltageType ret =  static_cast<Eps_0x260_260::Eps_st_emergencyovervoltageType>(x);
  return ret;
}

// config detail: {'name': 'eps_st_emergencyundervoltage', 'enum': {0: 'EPS_ST_EMERGENCYUNDERVOLTAGE_NORMAL', 1: 'EPS_ST_EMERGENCYUNDERVOLTAGE_UNDER_VOLTAGE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_st_emergencyundervoltageType Eps0x260260::eps_st_emergencyundervoltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  Eps_0x260_260::Eps_st_emergencyundervoltageType ret =  static_cast<Eps_0x260_260::Eps_st_emergencyundervoltageType>(x);
  return ret;
}

// config detail: {'name': 'eps_f_ecutempvalid', 'enum': {0: 'EPS_F_ECUTEMPVALID_INVALID', 1: 'EPS_F_ECUTEMPVALID_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_f_ecutempvalidType Eps0x260260::eps_f_ecutempvalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  Eps_0x260_260::Eps_f_ecutempvalidType ret =  static_cast<Eps_0x260_260::Eps_f_ecutempvalidType>(x);
  return ret;
}

// config detail: {'name': 'eps_sassoftlimitleftflg', 'enum': {0: 'EPS_SASSOFTLIMITLEFTFLG_NO_LEARNED', 1: 'EPS_SASSOFTLIMITLEFTFLG_PRIMARY_LEARNED_ONLY_FOR_CEPS', 2: 'EPS_SASSOFTLIMITLEFTFLG_LEARNED', 3: 'EPS_SASSOFTLIMITLEFTFLG_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_sassoftlimitleftflgType Eps0x260260::eps_sassoftlimitleftflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 2);

  Eps_0x260_260::Eps_sassoftlimitleftflgType ret =  static_cast<Eps_0x260_260::Eps_sassoftlimitleftflgType>(x);
  return ret;
}

// config detail: {'name': 'eps_n_ecutemp', 'offset': -70.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-70|185]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Degree'}
int Eps0x260260::eps_n_ecutemp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -70.000000;
  return ret;
}

// config detail: {'name': 'eps_n_performanceredu', 'offset': 0.0, 'precision': 0.5, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Eps0x260260::eps_n_performanceredu(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.500000;
  return ret;
}

// config detail: {'name': 'eps_st_dtcflag', 'enum': {0: 'EPS_ST_DTCFLAG_NO_DTC_EXIST', 1: 'EPS_ST_DTCFLAG_DTC_EXIST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_st_dtcflagType Eps0x260260::eps_st_dtcflag(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  Eps_0x260_260::Eps_st_dtcflagType ret =  static_cast<Eps_0x260_260::Eps_st_dtcflagType>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x260', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Eps0x260260::rolling_counter_0x260(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'eps_sasindexsts', 'enum': {0: 'EPS_SASINDEXSTS_STEERWHEEL_NOT_AT_MIDDLE_POSITION', 1: 'EPS_SASINDEXSTS_STEERWHEEL_AT_MIDDLE_POSITION'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_sasindexstsType Eps0x260260::eps_sasindexsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Eps_0x260_260::Eps_sasindexstsType ret =  static_cast<Eps_0x260_260::Eps_sasindexstsType>(x);
  return ret;
}

// config detail: {'name': 'eps_s_warninglampyellow', 'enum': {0: 'EPS_S_WARNINGLAMPYELLOW_CLOSE', 1: 'EPS_S_WARNINGLAMPYELLOW_OPEN'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_s_warninglampyellowType Eps0x260260::eps_s_warninglampyellow(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  Eps_0x260_260::Eps_s_warninglampyellowType ret =  static_cast<Eps_0x260_260::Eps_s_warninglampyellowType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x260', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Eps0x260260::checksum_0x260(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'eps_s_safelampred', 'enum': {0: 'EPS_S_SAFELAMPRED_CLOSE', 1: 'EPS_S_SAFELAMPRED_OPEN'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_s_safelampredType Eps0x260260::eps_s_safelampred(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Eps_0x260_260::Eps_s_safelampredType ret =  static_cast<Eps_0x260_260::Eps_s_safelampredType>(x);
  return ret;
}

// config detail: {'name': 'eps_f_loadinfo', 'enum': {0: 'EPS_F_LOADINFO_NO_FAULT', 1: 'EPS_F_LOADINFO_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Eps_0x260_260::Eps_f_loadinfoType Eps0x260260::eps_f_loadinfo(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Eps_0x260_260::Eps_f_loadinfoType ret =  static_cast<Eps_0x260_260::Eps_f_loadinfoType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
