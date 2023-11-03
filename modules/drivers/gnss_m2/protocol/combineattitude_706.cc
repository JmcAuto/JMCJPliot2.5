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

#include "modules/drivers/gnss_m2/protocol/combineattitude_706.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

using ::jmc_auto::drivers::canbus::Byte;

Combineattitude706::Combineattitude706() {}
const int32_t Combineattitude706::ID = 0x706;

void Combineattitude706::Parse(const std::uint8_t* bytes, int32_t length,
                         jmc_auto::drivers::gnss_m2::M2* m2) const {
  m2->mutable_combineattitude_706()->set_roll(roll(bytes, length));
  m2->mutable_combineattitude_706()->set_pitch(pitch(bytes, length));
  m2->mutable_combineattitude_706()->set_yaw(yaw(bytes, length));
}

// config detail: {'name': 'roll', 'offset': 0.0, 'precision': 0.001, 'len': 20, 'is_signed_var': True, 'physical_range': '[-180|180]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Combineattitude706::roll(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 7);
  t = t2.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 12;
  x >>= 12;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'pitch', 'offset': 0.0, 'precision': 0.001, 'len': 20, 'is_signed_var': True, 'physical_range': '[-90|90]', 'bit': 19, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Combineattitude706::pitch(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 4);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 12;
  x >>= 12;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'yaw', 'offset': 0.0, 'precision': 0.001, 'len': 20, 'is_signed_var': False, 'physical_range': '[0|360]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Combineattitude706::yaw(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace m2
}  // namespace canbus
}  // namespace jmc_auto
