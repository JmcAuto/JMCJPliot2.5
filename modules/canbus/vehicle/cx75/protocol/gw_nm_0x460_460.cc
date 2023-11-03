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

#include "modules/canbus/vehicle/cx75/protocol/gw_nm_0x460_460.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwnm0x460460::Gwnm0x460460() {}
const int32_t Gwnm0x460460::ID = 0x460;

void Gwnm0x460460::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_nm_0x460_460()->set_gwnmlimphome(gwnmlimphome(bytes, length));
  chassis->mutable_cx75()->mutable_gw_nm_0x460_460()->set_gwnmsleepind(gwnmsleepind(bytes, length));
  chassis->mutable_cx75()->mutable_gw_nm_0x460_460()->set_gwnmsleepack(gwnmsleepack(bytes, length));
  chassis->mutable_cx75()->mutable_gw_nm_0x460_460()->set_gwnmdestaddr(gwnmdestaddr(bytes, length));
  chassis->mutable_cx75()->mutable_gw_nm_0x460_460()->set_gwnmalive(gwnmalive(bytes, length));
  chassis->mutable_cx75()->mutable_gw_nm_0x460_460()->set_gwnmring(gwnmring(bytes, length));
}

// config detail: {'name': 'gwnmlimphome', 'enum': {0: 'GWNMLIMPHOME_NOT_ACTIVE', 1: 'GWNMLIMPHOME_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 10, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_nm_0x460_460::GwnmlimphomeType Gwnm0x460460::gwnmlimphome(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  Gw_nm_0x460_460::GwnmlimphomeType ret =  static_cast<Gw_nm_0x460_460::GwnmlimphomeType>(x);
  return ret;
}

// config detail: {'name': 'gwnmsleepind', 'enum': {0: 'GWNMSLEEPIND_NOT_ACTIVE', 1: 'GWNMSLEEPIND_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_nm_0x460_460::GwnmsleepindType Gwnm0x460460::gwnmsleepind(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  Gw_nm_0x460_460::GwnmsleepindType ret =  static_cast<Gw_nm_0x460_460::GwnmsleepindType>(x);
  return ret;
}

// config detail: {'name': 'gwnmsleepack', 'enum': {0: 'GWNMSLEEPACK_NOT_ACTIVE', 1: 'GWNMSLEEPACK_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_nm_0x460_460::GwnmsleepackType Gwnm0x460460::gwnmsleepack(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  Gw_nm_0x460_460::GwnmsleepackType ret =  static_cast<Gw_nm_0x460_460::GwnmsleepackType>(x);
  return ret;
}

// config detail: {'name': 'gwnmdestaddr', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwnm0x460460::gwnmdestaddr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'gwnmalive', 'enum': {0: 'GWNMALIVE_NOT_ACTIVE', 1: 'GWNMALIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_nm_0x460_460::GwnmaliveType Gwnm0x460460::gwnmalive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Gw_nm_0x460_460::GwnmaliveType ret =  static_cast<Gw_nm_0x460_460::GwnmaliveType>(x);
  return ret;
}

// config detail: {'name': 'gwnmring', 'enum': {0: 'GWNMRING_NOT_ACTIVE', 1: 'GWNMRING_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_nm_0x460_460::GwnmringType Gwnm0x460460::gwnmring(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  Gw_nm_0x460_460::GwnmringType ret =  static_cast<Gw_nm_0x460_460::GwnmringType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
