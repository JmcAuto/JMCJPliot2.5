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

#include "modules/canbus/vehicle/cx75/protocol/sod_nm_0x440_440.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Sodnm0x440440::Sodnm0x440440() {}
const int32_t Sodnm0x440440::ID = 0x440;

void Sodnm0x440440::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_sod_nm_0x440_440()->set_sodnmlimphome(sodnmlimphome(bytes, length));
  chassis->mutable_cx75()->mutable_sod_nm_0x440_440()->set_sodnmsleepind(sodnmsleepind(bytes, length));
  chassis->mutable_cx75()->mutable_sod_nm_0x440_440()->set_sodnmsleepack(sodnmsleepack(bytes, length));
  chassis->mutable_cx75()->mutable_sod_nm_0x440_440()->set_sodnmdestaddr(sodnmdestaddr(bytes, length));
  chassis->mutable_cx75()->mutable_sod_nm_0x440_440()->set_sodnmalive(sodnmalive(bytes, length));
  chassis->mutable_cx75()->mutable_sod_nm_0x440_440()->set_sodnmring(sodnmring(bytes, length));
}

// config detail: {'name': 'sodnmlimphome', 'enum': {0: 'SODNMLIMPHOME_NOT_ACTIVE', 1: 'SODNMLIMPHOME_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 10, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_nm_0x440_440::SodnmlimphomeType Sodnm0x440440::sodnmlimphome(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  Sod_nm_0x440_440::SodnmlimphomeType ret =  static_cast<Sod_nm_0x440_440::SodnmlimphomeType>(x);
  return ret;
}

// config detail: {'name': 'sodnmsleepind', 'enum': {0: 'SODNMSLEEPIND_NOT_ACTIVE', 1: 'SODNMSLEEPIND_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_nm_0x440_440::SodnmsleepindType Sodnm0x440440::sodnmsleepind(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  Sod_nm_0x440_440::SodnmsleepindType ret =  static_cast<Sod_nm_0x440_440::SodnmsleepindType>(x);
  return ret;
}

// config detail: {'name': 'sodnmsleepack', 'enum': {0: 'SODNMSLEEPACK_NOT_ACTIVE', 1: 'SODNMSLEEPACK_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_nm_0x440_440::SodnmsleepackType Sodnm0x440440::sodnmsleepack(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  Sod_nm_0x440_440::SodnmsleepackType ret =  static_cast<Sod_nm_0x440_440::SodnmsleepackType>(x);
  return ret;
}

// config detail: {'name': 'sodnmdestaddr', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Sodnm0x440440::sodnmdestaddr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'sodnmalive', 'enum': {0: 'SODNMALIVE_NOT_ACTIVE', 1: 'SODNMALIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_nm_0x440_440::SodnmaliveType Sodnm0x440440::sodnmalive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Sod_nm_0x440_440::SodnmaliveType ret =  static_cast<Sod_nm_0x440_440::SodnmaliveType>(x);
  return ret;
}

// config detail: {'name': 'sodnmring', 'enum': {0: 'SODNMRING_NOT_ACTIVE', 1: 'SODNMRING_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_nm_0x440_440::SodnmringType Sodnm0x440440::sodnmring(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  Sod_nm_0x440_440::SodnmringType ret =  static_cast<Sod_nm_0x440_440::SodnmringType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
