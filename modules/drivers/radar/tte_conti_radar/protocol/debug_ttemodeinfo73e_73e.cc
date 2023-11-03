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

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_ttemodeinfo73e_73e.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

using ::jmc_auto::drivers::canbus::Byte;

Debugttemodeinfo73e73e::Debugttemodeinfo73e73e() {}
const int32_t Debugttemodeinfo73e73e::ID = 0x73E;

void Debugttemodeinfo73e73e::Parse(const std::uint8_t* bytes, int32_t length,
                       TteContiRadar* ttecontiradar) const {
  ttecontiradar->mutable_debug_ttemodeinfo73e_73e()->set_ttemodeinfo73e_copy_7(ttemodeinfo73e_copy_7(bytes, length));
  ttecontiradar->mutable_debug_ttemodeinfo73e_73e()->set_ttemodeinfo73e_copy_6(ttemodeinfo73e_copy_6(bytes, length));
  ttecontiradar->mutable_debug_ttemodeinfo73e_73e()->set_ttemodeinfo73e_copy_5(ttemodeinfo73e_copy_5(bytes, length));
  ttecontiradar->mutable_debug_ttemodeinfo73e_73e()->set_ttemodeinfo73e_copy_4(ttemodeinfo73e_copy_4(bytes, length));
  ttecontiradar->mutable_debug_ttemodeinfo73e_73e()->set_ttemodeinfo73e_copy_3(ttemodeinfo73e_copy_3(bytes, length));
  ttecontiradar->mutable_debug_ttemodeinfo73e_73e()->set_ttemodeinfo73e_copy_2(ttemodeinfo73e_copy_2(bytes, length));
  ttecontiradar->mutable_debug_ttemodeinfo73e_73e()->set_ttemodeinfo73e_copy_1(ttemodeinfo73e_copy_1(bytes, length));
  ttecontiradar->mutable_debug_ttemodeinfo73e_73e()->set_ttemodeinfo73e(ttemodeinfo73e(bytes, length));
}

// config detail: {'name': 'ttemodeinfo73e_copy_7', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Debugttemodeinfo73e73e::ttemodeinfo73e_copy_7(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ttemodeinfo73e_copy_6', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Debugttemodeinfo73e73e::ttemodeinfo73e_copy_6(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ttemodeinfo73e_copy_5', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Debugttemodeinfo73e73e::ttemodeinfo73e_copy_5(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ttemodeinfo73e_copy_4', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Debugttemodeinfo73e73e::ttemodeinfo73e_copy_4(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ttemodeinfo73e_copy_3', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Debugttemodeinfo73e73e::ttemodeinfo73e_copy_3(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ttemodeinfo73e_copy_2', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Debugttemodeinfo73e73e::ttemodeinfo73e_copy_2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ttemodeinfo73e_copy_1', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Debugttemodeinfo73e73e::ttemodeinfo73e_copy_1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ttemodeinfo73e', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Debugttemodeinfo73e73e::ttemodeinfo73e(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
