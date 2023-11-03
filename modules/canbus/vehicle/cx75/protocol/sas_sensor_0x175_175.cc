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

#include "modules/canbus/vehicle/cx75/protocol/sas_sensor_0x175_175.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Sassensor0x175175::Sassensor0x175175() {}
const int32_t Sassensor0x175175::ID = 0x175;

void Sassensor0x175175::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_sas_sensor_0x175_175()->set_sas_raw_steerwheelangle(sas_raw_steerwheelangle(bytes, length));
  chassis->mutable_cx75()->mutable_sas_sensor_0x175_175()->set_sas_steerwheelrotspd(sas_steerwheelrotspd(bytes, length));
  chassis->mutable_cx75()->mutable_sas_sensor_0x175_175()->set_sas_trimmingsts(sas_trimmingsts(bytes, length));
  chassis->mutable_cx75()->mutable_sas_sensor_0x175_175()->set_sas_steerwheelrotspdstatus(sas_steerwheelrotspdstatus(bytes, length));
  chassis->mutable_cx75()->mutable_sas_sensor_0x175_175()->set_rolling_counter_0x175(rolling_counter_0x175(bytes, length));
  chassis->mutable_cx75()->mutable_sas_sensor_0x175_175()->set_sas_sasstscal(sas_sasstscal(bytes, length));
  chassis->mutable_cx75()->mutable_sas_sensor_0x175_175()->set_sas_raw_sasstssnsr(sas_raw_sasstssnsr(bytes, length));
  chassis->mutable_cx75()->mutable_sas_sensor_0x175_175()->set_sas_sasstssnsr(sas_sasstssnsr(bytes, length));
  chassis->mutable_cx75()->mutable_sas_sensor_0x175_175()->set_checksum_0x175(checksum_0x175(bytes, length));
  chassis->mutable_cx75()->mutable_sas_sensor_0x175_175()->set_sas_steerwheelangle(sas_steerwheelangle(bytes, length));
}

// config detail: {'name': 'sas_raw_steerwheelangle', 'enum': {65535: 'SAS_RAW_STEERWHEELANGLE_INVALID'}, 'precision': 0.0238, 'len': 16, 'is_signed_var': False, 'offset': -780.0, 'physical_range': '[-780|779.7092]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'deg'}
double Sassensor0x175175::sas_raw_steerwheelangle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret =  static_cast<double>(x);
  ret=ret*0.0238-780.0;
  return ret;
}

// config detail: {'name': 'sas_steerwheelrotspd', 'enum': {16383: 'SAS_STEERWHEELROTSPD_INVALID'}, 'precision': 0.125, 'len': 14, 'is_signed_var': False, 'offset': -1024.0, 'physical_range': '[-1024|1023.75]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'deg s'}
double Sassensor0x175175::sas_steerwheelrotspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(2, 6);
  x <<= 6;
  x |= t;

  double ret =  static_cast<double>(x);
  ret=ret*0.125-1024;
  return ret;
}

// config detail: {'name': 'sas_trimmingsts', 'enum': {0: 'SAS_TRIMMINGSTS_NOT_TRIMMED', 1: 'SAS_TRIMMINGSTS_TRIMMED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sas_sensor_0x175_175::Sas_trimmingstsType Sassensor0x175175::sas_trimmingsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  Sas_sensor_0x175_175::Sas_trimmingstsType ret =  static_cast<Sas_sensor_0x175_175::Sas_trimmingstsType>(x);
  return ret;
}

// config detail: {'name': 'sas_steerwheelrotspdstatus', 'enum': {0: 'SAS_STEERWHEELROTSPDSTATUS_VALID', 1: 'SAS_STEERWHEELROTSPDSTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sas_sensor_0x175_175::Sas_steerwheelrotspdstatusType Sassensor0x175175::sas_steerwheelrotspdstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 1);

  Sas_sensor_0x175_175::Sas_steerwheelrotspdstatusType ret =  static_cast<Sas_sensor_0x175_175::Sas_steerwheelrotspdstatusType>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x175', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Sassensor0x175175::rolling_counter_0x175(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'sas_sasstscal', 'enum': {0: 'SAS_SASSTSCAL_SENSOR_NOT_CALIBRATED', 1: 'SAS_SASSTSCAL_SENSOR_CALIBRATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sas_sensor_0x175_175::Sas_sasstscalType Sassensor0x175175::sas_sasstscal(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Sas_sensor_0x175_175::Sas_sasstscalType ret =  static_cast<Sas_sensor_0x175_175::Sas_sasstscalType>(x);
  return ret;
}

// config detail: {'name': 'sas_raw_sasstssnsr', 'enum': {0: 'SAS_RAW_SASSTSSNSR_SENSOR_VALUE_INVALID', 1: 'SAS_RAW_SASSTSSNSR_SENSOR_VALUE_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sas_sensor_0x175_175::Sas_raw_sasstssnsrType Sassensor0x175175::sas_raw_sasstssnsr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Sas_sensor_0x175_175::Sas_raw_sasstssnsrType ret =  static_cast<Sas_sensor_0x175_175::Sas_raw_sasstssnsrType>(x);
  return ret;
}

// config detail: {'name': 'sas_sasstssnsr', 'enum': {0: 'SAS_SASSTSSNSR_SENSOR_VALUE_INVALID', 1: 'SAS_SASSTSSNSR_SENSOR_VALUE_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sas_sensor_0x175_175::Sas_sasstssnsrType Sassensor0x175175::sas_sasstssnsr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  Sas_sensor_0x175_175::Sas_sasstssnsrType ret =  static_cast<Sas_sensor_0x175_175::Sas_sasstssnsrType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x175', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Sassensor0x175175::checksum_0x175(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'sas_steerwheelangle', 'enum': {65535: 'SAS_STEERWHEELANGLE_INVALID'}, 'precision': 0.0238, 'len': 16, 'is_signed_var': False, 'offset': -780.0, 'physical_range': '[-780|779.7092]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'deg'}
double Sassensor0x175175::sas_steerwheelangle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret =  static_cast<double>(x);
  ret=ret*0.0238-780.0;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
