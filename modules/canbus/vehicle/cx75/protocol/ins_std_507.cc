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

#include "modules/canbus/vehicle/cx75/protocol/ins_std_507.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Insstd507::Insstd507() {}
const int32_t Insstd507::ID = 0x507;

void Insstd507::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_ins_std_507()->set_ins_std_lat(ins_std_lat(bytes, length));
  chassis->mutable_cx75()->mutable_ins_std_507()->set_ins_std_lon(ins_std_lon(bytes, length));
  chassis->mutable_cx75()->mutable_ins_std_507()->set_ins_std_locatheight(ins_std_locatheight(bytes, length));
  chassis->mutable_cx75()->mutable_ins_std_507()->set_inins_std_heading(inins_std_heading(bytes, length));
}

// config detail: {'name': 'ins_std_lat', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Insstd507::ins_std_lat(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'ins_std_lon', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Insstd507::ins_std_lon(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'ins_std_locatheight', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Insstd507::ins_std_locatheight(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'inins_std_heading', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65.535]', 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Insstd507::inins_std_heading(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
