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

#include "modules/canbus/vehicle/cx75/protocol/gw_ems_tq_0x101_101.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwemstq0x101101::Gwemstq0x101101() {}
const int32_t Gwemstq0x101101::ID = 0x101;

void Gwemstq0x101101::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_ems_tq_0x101_101()->set_ems_indicatedrealengtorq(ems_indicatedrealengtorq(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tq_0x101_101()->set_ems_engspeed(ems_engspeed(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tq_0x101_101()->set_ems_engspeederror(ems_engspeederror(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tq_0x101_101()->set_ems_rngmodtorqcrsleadmin(ems_rngmodtorqcrsleadmin(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tq_0x101_101()->set_dcm_ems_rollingcounter_0x101(dcm_ems_rollingcounter_0x101(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tq_0x101_101()->set_dcm_ems_checksum_0x101(dcm_ems_checksum_0x101(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tq_0x101_101()->set_ems_rngmodtorqcrsleadmax(ems_rngmodtorqcrsleadmax(bytes, length));
}

// config detail: {'name': 'ems_indicatedrealengtorq', 'enum': {2047: 'EMS_INDICATEDREALENGTORQ_INVALID'}, 'precision': 0.5, 'len': 12, 'is_signed_var': False, 'offset': -1000.0, 'physical_range': '[-1000|1000]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
double Gwemstq0x101101::ems_indicatedrealengtorq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'ems_engspeed', 'enum': {32767: 'EMS_ENGSPEED_INVALID'}, 'precision': 0.5, 'len': 15, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|16383]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'rpm'}
double Gwemstq0x101101::ems_engspeed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'ems_engspeederror', 'enum': {0: 'EMS_ENGSPEEDERROR_NOERROR', 1: 'EMS_ENGSPEEDERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_tq_0x101_101::Ems_engspeederrorType Gwemstq0x101101::ems_engspeederror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Gw_ems_tq_0x101_101::Ems_engspeederrorType ret =  static_cast<Gw_ems_tq_0x101_101::Ems_engspeederrorType>(x);
  return ret;
}

// config detail: {'name': 'ems_rngmodtorqcrsleadmin', 'enum': {2047: 'EMS_RNGMODTORQCRSLEADMIN_INVALID'}, 'precision': 0.5, 'len': 12, 'is_signed_var': False, 'offset': -1000.0, 'physical_range': '[-1000|1000]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
double Gwemstq0x101101::ems_rngmodtorqcrsleadmin(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'dcm_ems_rollingcounter_0x101', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwemstq0x101101::dcm_ems_rollingcounter_0x101(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'dcm_ems_checksum_0x101', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwemstq0x101101::dcm_ems_checksum_0x101(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ems_rngmodtorqcrsleadmax', 'enum': {2047: 'EMS_RNGMODTORQCRSLEADMAX_INVALID'}, 'precision': 0.5, 'len': 12, 'is_signed_var': False, 'offset': -1000.0, 'physical_range': '[-1000|1000]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
double Gwemstq0x101101::ems_rngmodtorqcrsleadmax(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
