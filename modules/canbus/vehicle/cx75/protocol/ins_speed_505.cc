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

#include "modules/canbus/vehicle/cx75/protocol/ins_speed_505.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Insspeed505::Insspeed505() {}
const int32_t Insspeed505::ID = 0x505;

void Insspeed505::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_ins_speed_505()->set_ins_northspd(ins_northspd(bytes, length));
  chassis->mutable_cx75()->mutable_ins_speed_505()->set_ins_eastspd(ins_eastspd(bytes, length));
  chassis->mutable_cx75()->mutable_ins_speed_505()->set_ins_togroundspd(ins_togroundspd(bytes, length));
}

// config detail: {'name': 'ins_northspd', 'offset': -100.0, 'precision': 0.0030517, 'len': 16, 'is_signed_var': True, 'physical_range': '[-100|100]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Insspeed505::ins_northspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  //x <<= 16;
  //x >>= 16;

  double ret = x * 0.0030517 + -100.000000;
  return ret;
}

// config detail: {'name': 'ins_eastspd', 'offset': -100.0, 'precision': 0.0030517, 'len': 16, 'is_signed_var': True, 'physical_range': '[-100|100]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Insspeed505::ins_eastspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  //x <<= 16;
  //x >>= 16;

  double ret = x * 0.0030517 + -100.000000;
  return ret;
}

// config detail: {'name': 'ins_togroundspd', 'offset': -100.0, 'precision': 0.0030517, 'len': 16, 'is_signed_var': True, 'physical_range': '[-100|100]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Insspeed505::ins_togroundspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  //x <<= 16;
  //x >>= 16;

  double ret = x * 0.0030517 + -100.000000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
