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

#include "modules/drivers/gnss_m2/protocol/combinevelocity_705.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

using ::jmc_auto::drivers::canbus::Byte;

Combinevelocity705::Combinevelocity705() {}
const int32_t Combinevelocity705::ID = 0x705;

void Combinevelocity705::Parse(const std::uint8_t* bytes, int32_t length,
                         jmc_auto::drivers::gnss_m2::M2* m2) const {
  m2->mutable_combinevelocity_705()->set_velocityhorizon(velocityhorizon(bytes, length));
  m2->mutable_combinevelocity_705()->set_velocityup(velocityup(bytes, length));
  m2->mutable_combinevelocity_705()->set_velocitynorth(velocitynorth(bytes, length));
  m2->mutable_combinevelocity_705()->set_velocityeast(velocityeast(bytes, length));
}

// config detail: {'name': 'velocityhorizon', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Combinevelocity705::velocityhorizon(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'velocityup', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Combinevelocity705::velocityup(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'velocitynorth', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Combinevelocity705::velocitynorth(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'velocityeast', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm/s'}
double Combinevelocity705::velocityeast(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace m2
}  // namespace canbus
}  // namespace jmc_auto
