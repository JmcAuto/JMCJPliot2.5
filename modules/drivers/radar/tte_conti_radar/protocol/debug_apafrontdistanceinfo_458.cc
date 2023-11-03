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

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_apafrontdistanceinfo_458.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

using ::jmc_auto::drivers::canbus::Byte;

Debugapafrontdistanceinfo458::Debugapafrontdistanceinfo458() {}
const int32_t Debugapafrontdistanceinfo458::ID = 0x458;

void Debugapafrontdistanceinfo458::Parse(const std::uint8_t* bytes, int32_t length,
                       TteContiRadar* ttecontiradar) const {
  ttecontiradar->mutable_debug_apafrontdistanceinfo_458()->set_apafrs_distance(apafrs_distance(bytes, length));
  ttecontiradar->mutable_debug_apafrontdistanceinfo_458()->set_apafrm_distance(apafrm_distance(bytes, length));
  ttecontiradar->mutable_debug_apafrontdistanceinfo_458()->set_apafr_distance(apafr_distance(bytes, length));
  ttecontiradar->mutable_debug_apafrontdistanceinfo_458()->set_apafls_distance(apafls_distance(bytes, length));
  ttecontiradar->mutable_debug_apafrontdistanceinfo_458()->set_apaflm_distance(apaflm_distance(bytes, length));
  ttecontiradar->mutable_debug_apafrontdistanceinfo_458()->set_apafl_distance(apafl_distance(bytes, length));
}

// config detail: {'name': 'apafrs_distance', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Debugapafrontdistanceinfo458::apafrs_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'apafrm_distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Debugapafrontdistanceinfo458::apafrm_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'name': 'apafr_distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Debugapafrontdistanceinfo458::apafr_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'name': 'apafls_distance', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Debugapafrontdistanceinfo458::apafls_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'apaflm_distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Debugapafrontdistanceinfo458::apaflm_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'name': 'apafl_distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
double Debugapafrontdistanceinfo458::apafl_distance(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
