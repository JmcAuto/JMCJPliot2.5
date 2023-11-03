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

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_rightvplslot_ptcd_464.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

using ::jmc_auto::drivers::canbus::Byte;

Debugrightvplslotptcd464::Debugrightvplslotptcd464() {}
const int32_t Debugrightvplslotptcd464::ID = 0x464;

void Debugrightvplslotptcd464::Parse(const std::uint8_t* bytes, int32_t length,
                       TteContiRadar* ttecontiradar) const {
  ttecontiradar->mutable_debug_rightvplslot_ptcd_464()->set_rightvplslot_ptd_y(rightvplslot_ptd_y(bytes, length));
  ttecontiradar->mutable_debug_rightvplslot_ptcd_464()->set_rightvplslot_ptd_x(rightvplslot_ptd_x(bytes, length));
  ttecontiradar->mutable_debug_rightvplslot_ptcd_464()->set_rightvplslot_ptc_y(rightvplslot_ptc_y(bytes, length));
  ttecontiradar->mutable_debug_rightvplslot_ptcd_464()->set_rightvplslot_ptc_x(rightvplslot_ptc_x(bytes, length));
}

// config detail: {'name': 'rightvplslot_ptd_y', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Debugrightvplslotptcd464::rightvplslot_ptd_y(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'rightvplslot_ptd_x', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Debugrightvplslotptcd464::rightvplslot_ptd_x(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'rightvplslot_ptc_y', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Debugrightvplslotptcd464::rightvplslot_ptc_y(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'rightvplslot_ptc_x', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Debugrightvplslotptcd464::rightvplslot_ptc_x(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
