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

#include "modules/drivers/gnss_m2/protocol/combinegpstime_701.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

using ::jmc_auto::drivers::canbus::Byte;

Combinegpstime701::Combinegpstime701() {}
const int32_t Combinegpstime701::ID = 0x701;

void Combinegpstime701::Parse(const std::uint8_t* bytes, int32_t length,
                         jmc_auto::drivers::gnss_m2::M2* m2) const {
  m2->mutable_combinegpstime_701()->set_gpssecond(gpssecond(bytes, length));
  m2->mutable_combinegpstime_701()->set_gpsweek(gpsweek(bytes, length));
}

// config detail: {'description': '\xd7\xe9\xba\xcf\xcf\xb5\xcd\xb3GPS\xc3\xeb', 'offset': 0.0, 'precision': 0.001, 'len': 32, 'name': 'gpssecond', 'is_signed_var': False, 'physical_range': '[0|604800]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 's'}
double Combinegpstime701::gpssecond(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 4);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 5);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'gpsweek', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': '\xd6\xdc'}
int Combinegpstime701::gpsweek(const std::uint8_t* bytes, int32_t length) const {
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
