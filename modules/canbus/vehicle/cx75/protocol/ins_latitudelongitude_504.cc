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

#include "modules/canbus/vehicle/cx75/protocol/ins_latitudelongitude_504.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Inslatitudelongitude504::Inslatitudelongitude504() {}
const int32_t Inslatitudelongitude504::ID = 0x504;

void Inslatitudelongitude504::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_ins_latitudelongitude_504()->set_ins__latitude(ins__latitude(bytes, length));
  chassis->mutable_cx75()->mutable_ins_latitudelongitude_504()->set_ins_longitude(ins_longitude(bytes, length));
}

// config detail: {'name': 'ins__latitude', 'offset': -180.0, 'precision': 1e-07, 'len': 32, 'is_signed_var': True, 'physical_range': '[-180|180]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Inslatitudelongitude504::ins__latitude(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  uint32_t x = t0.get_byte(0, 8);

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

  //x <<= 0;
  //x >>= 0;

  double ret = x * 0.0000001 + -180.000000;
  return ret;
}

// config detail: {'name': 'ins_longitude', 'offset': -180.0, 'precision': 1e-07, 'len': 32, 'is_signed_var': True, 'physical_range': '[-180|180]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Inslatitudelongitude504::ins_longitude(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  uint32_t x = t0.get_byte(0, 8);

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

  //x <<= 0;
  //x >>= 0;
AINFO<<"ins_longitude X:"<<x;
  double ret = x * 0.0000001 + -180.000000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
