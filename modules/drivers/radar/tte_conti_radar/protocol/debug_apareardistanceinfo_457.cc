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

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_apareardistanceinfo_457.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

using ::jmc_auto::drivers::canbus::Byte;

Debugapareardistanceinfo457::Debugapareardistanceinfo457() {}
const int32_t Debugapareardistanceinfo457::ID = 0x457;

void Debugapareardistanceinfo457::Parse(const std::uint8_t* bytes, int32_t length,
                       TteContiRadar* ttecontiradar) const {
  ttecontiradar->mutable_debug_apareardistanceinfo_457()->set_aparrs_distance(aparrs_distance(bytes, length));
  ttecontiradar->mutable_debug_apareardistanceinfo_457()->set_aparrm_distance(aparrm_distance(bytes, length));
  ttecontiradar->mutable_debug_apareardistanceinfo_457()->set_aparr_distance(aparr_distance(bytes, length));
  ttecontiradar->mutable_debug_apareardistanceinfo_457()->set_aparls_distance(aparls_distance(bytes, length));
  ttecontiradar->mutable_debug_apareardistanceinfo_457()->set_aparl_distance(aparl_distance(bytes, length));
  ttecontiradar->mutable_debug_apareardistanceinfo_457()->set_aparlm_distance(aparlm_distance(bytes, length));
}

// config detail: {'name': 'aparrs_distance', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Debugapareardistanceinfo457::aparrs_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'aparrm_distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Debugapareardistanceinfo457::aparrm_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'name': 'aparr_distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Debugapareardistanceinfo457::aparr_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'name': 'aparls_distance', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Debugapareardistanceinfo457::aparls_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'aparl_distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Debugapareardistanceinfo457::aparl_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'name': 'aparlm_distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Debugapareardistanceinfo457::aparlm_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
