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

#include "modules/drivers/gnss_m2/protocol/combineutctime_702.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

using ::jmc_auto::drivers::canbus::Byte;

Combineutctime702::Combineutctime702() {}
const int32_t Combineutctime702::ID = 0x702;

void Combineutctime702::Parse(const std::uint8_t* bytes, int32_t length,
                         jmc_auto::drivers::gnss_m2::M2* m2) const {
  m2->mutable_combineutctime_702()->set_utcmilsecond(utcmilsecond(bytes, length));
  m2->mutable_combineutctime_702()->set_utcsecond(utcsecond(bytes, length));
  m2->mutable_combineutctime_702()->set_utcminute(utcminute(bytes, length));
  m2->mutable_combineutctime_702()->set_utchour(utchour(bytes, length));
  m2->mutable_combineutctime_702()->set_utcdate(utcdate(bytes, length));
  m2->mutable_combineutctime_702()->set_utcmonth(utcmonth(bytes, length));
  m2->mutable_combineutctime_702()->set_utcyear(utcyear(bytes, length));
}

// config detail: {'name': 'utcmilsecond', 'offset': 0.0, 'precision': 0.01, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 63, 'type': 'double', 'order': 'motorola', 'physical_unit': 's'}
double Combineutctime702::utcmilsecond(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'utcsecond', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|60]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': 's'}
int Combineutctime702::utcsecond(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'utcminute', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|60]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Combineutctime702::utcminute(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'utchour', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|24]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Combineutctime702::utchour(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'utcdate', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range': '[0|31]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Combineutctime702::utcdate(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'name': 'utcmonth', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|12]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Combineutctime702::utcmonth(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'utcyear', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Combineutctime702::utcyear(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace m2
}  // namespace canbus
}  // namespace jmc_auto
