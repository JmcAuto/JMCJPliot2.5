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

#include "modules/drivers/gnss_m2/protocol/revrattitude_736.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

using ::jmc_auto::drivers::canbus::Byte;

Revrattitude736::Revrattitude736() {}
const int32_t Revrattitude736::ID = 0x736;

void Revrattitude736::Parse(const std::uint8_t* bytes, int32_t length,
                         jmc_auto::drivers::gnss_m2::M2* m2) const {
  m2->mutable_revrattitude_736()->set_track_receiver(track_receiver(bytes, length));
  m2->mutable_revrattitude_736()->set_baseline(baseline(bytes, length));
  m2->mutable_revrattitude_736()->set_pitch_receiver(pitch_receiver(bytes, length));
  m2->mutable_revrattitude_736()->set_yaw_receiver(yaw_receiver(bytes, length));
}

// config detail: {'name': 'track_receiver', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|360]', 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Revrattitude736::track_receiver(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'baseline', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
double Revrattitude736::baseline(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.001000;
  return ret;
}

// config detail: {'name': 'pitch_receiver', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[-90|90]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Revrattitude736::pitch_receiver(const std::uint8_t* bytes, int32_t length) const {
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

// config detail: {'name': 'yaw_receiver', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|360]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'deg'}
double Revrattitude736::yaw_receiver(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace m2
}  // namespace canbus
}  // namespace jmc_auto
