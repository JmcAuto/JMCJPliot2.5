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

#include "modules/drivers/gnss_m2/protocol/revrutctime_732.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

using ::jmc_auto::drivers::canbus::Byte;

Revrutctime732::Revrutctime732() {}
const int32_t Revrutctime732::ID = 0x732;

void Revrutctime732::Parse(const std::uint8_t* bytes, int32_t length,
                         jmc_auto::drivers::gnss_m2::M2* m2) const {
  m2->mutable_revrutctime_732()->set_utctime_receiver(utctime_receiver(bytes, length));
  m2->mutable_revrutctime_732()->set_utcdate_receiver(utcdate_receiver(bytes, length));
}

// config detail: {'name': 'utctime_receiver', 'offset': 0.0, 'precision': 0.001, 'len': 32, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Revrutctime732::utctime_receiver(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 6);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 7);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'utcdate_receiver', 'offset': 0.0, 'precision': 1.0, 'len': 32, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Revrutctime732::utcdate_receiver(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 3);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace m2
}  // namespace canbus
}  // namespace jmc_auto
