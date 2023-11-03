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

#include "modules/canbus/vehicle/cx75/protocol/gw_ems_tqwhl_0x111_111.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwemstqwhl0x111111::Gwemstqwhl0x111111() {}
const int32_t Gwemstqwhl0x111111::ID = 0x111;

void Gwemstqwhl0x111111::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_ems_tqwhl_0x111_111()->set_ems_indicatedrealengtorqwhl(ems_indicatedrealengtorqwhl(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tqwhl_0x111_111()->set_ems_indicatedaccmesgerror(ems_indicatedaccmesgerror(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tqwhl_0x111_111()->set_ems_indicateddriveroverride(ems_indicateddriveroverride(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tqwhl_0x111_111()->set_ems_indicateddriverreqtorq(ems_indicateddriverreqtorq(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tqwhl_0x111_111()->set_dcm_ems_rollingcounter_0x111(dcm_ems_rollingcounter_0x111(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tqwhl_0x111_111()->set_dcm_ems_checksum_0x111(dcm_ems_checksum_0x111(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_tqwhl_0x111_111()->set_ems_indicateddriverreqtorqwhl(ems_indicateddriverreqtorqwhl(bytes, length));
}

// config detail: {'name': 'ems_indicatedrealengtorqwhl', 'enum': {65535: 'EMS_INDICATEDREALENGTORQWHL_INVALID'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': -30000.0, 'physical_range': '[-30000|30000]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwemstqwhl0x111111::ems_indicatedrealengtorqwhl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'name': 'ems_indicatedaccmesgerror', 'enum': {0: 'EMS_INDICATEDACCMESGERROR_NOEERROR', 1: 'EMS_INDICATEDACCMESGERROR_REVERSIBLE_ERROR', 2: 'EMS_INDICATEDACCMESGERROR_IRREVERSIBLE_ERROR', 3: 'EMS_INDICATEDACCMESGERROR_NOTDEFINED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_tqwhl_0x111_111::Ems_indicatedaccmesgerrorType Gwemstqwhl0x111111::ems_indicatedaccmesgerror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 2);

  Gw_ems_tqwhl_0x111_111::Ems_indicatedaccmesgerrorType ret =  static_cast<Gw_ems_tqwhl_0x111_111::Ems_indicatedaccmesgerrorType>(x);
  return ret;
}

// config detail: {'name': 'ems_indicateddriveroverride', 'enum': {0: 'EMS_INDICATEDDRIVEROVERRIDE_NOOVERRIDE', 1: 'EMS_INDICATEDDRIVEROVERRIDE_DRIVEROVERRIDE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_tqwhl_0x111_111::Ems_indicateddriveroverrideType Gwemstqwhl0x111111::ems_indicateddriveroverride(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  Gw_ems_tqwhl_0x111_111::Ems_indicateddriveroverrideType ret =  static_cast<Gw_ems_tqwhl_0x111_111::Ems_indicateddriveroverrideType>(x);
  return ret;
}

// config detail: {'name': 'ems_indicateddriverreqtorq', 'enum': {2047: 'EMS_INDICATEDDRIVERREQTORQ_INVALID'}, 'precision': 0.5, 'len': 12, 'is_signed_var': False, 'offset': -1000.0, 'physical_range': '[-1000|1000]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
double Gwemstqwhl0x111111::ems_indicateddriverreqtorq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'dcm_ems_rollingcounter_0x111', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwemstqwhl0x111111::dcm_ems_rollingcounter_0x111(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'dcm_ems_checksum_0x111', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwemstqwhl0x111111::dcm_ems_checksum_0x111(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ems_indicateddriverreqtorqwhl', 'enum': {65535: 'EMS_INDICATEDDRIVERREQTORQWHL_INVALID'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': -30000.0, 'physical_range': '[-30000|30000]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwemstqwhl0x111111::ems_indicateddriverreqtorqwhl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
