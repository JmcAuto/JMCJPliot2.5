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

#include "modules/canbus/vehicle/teshun/protocol/ibc_status2_0x124_124.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Ibcstatus20x124124::Ibcstatus20x124124() {}
const int32_t Ibcstatus20x124124::ID = 0x124;

void Ibcstatus20x124124::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_ibc_status2_0x124_124()->set_checksum_0x124(checksum_0x124(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status2_0x124_124()->set_rolling_counter_0x124(rolling_counter_0x124(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status2_0x124_124()->set_ibc_decelerationvaild(ibc_decelerationvaild(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status2_0x124_124()->set_ibc_deceleration(ibc_deceleration(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status2_0x124_124()->set_ibc_brktmcpositionvaild(ibc_brktmcpositionvaild(bytes, length));
  chassis->mutable_teshun()->mutable_ibc_status2_0x124_124()->set_ibc_brktmcposition(ibc_brktmcposition(bytes, length));
}

// config detail: {'name': 'checksum_0x124', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Ibcstatus20x124124::checksum_0x124(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x124', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Ibcstatus20x124124::rolling_counter_0x124(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ibc_decelerationvaild', 'enum': {0: 'IBC_DECELERATIONVAILD_INVAILD', 1: 'IBC_DECELERATIONVAILD_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Ibc_status2_0x124_124::Ibc_decelerationvaildType Ibcstatus20x124124::ibc_decelerationvaild(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  Ibc_status2_0x124_124::Ibc_decelerationvaildType ret =  static_cast<Ibc_status2_0x124_124::Ibc_decelerationvaildType>(x);
  return ret;
}

// config detail: {'name': 'ibc_deceleration', 'offset': 0.0, 'precision': 0.00159276, 'len': 12, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'g'}
double Ibcstatus20x124124::ibc_deceleration(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.001593;
  return ret;
}

// config detail: {'name': 'ibc_brktmcpositionvaild', 'enum': {0: 'IBC_BRKTMCPOSITIONVAILD_INVAILD', 1: 'IBC_BRKTMCPOSITIONVAILD_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Ibc_status2_0x124_124::Ibc_brktmcpositionvaildType Ibcstatus20x124124::ibc_brktmcpositionvaild(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Ibc_status2_0x124_124::Ibc_brktmcpositionvaildType ret =  static_cast<Ibc_status2_0x124_124::Ibc_brktmcpositionvaildType>(x);
  return ret;
}

// config detail: {'name': 'ibc_brktmcposition', 'offset': 0.0, 'precision': 0.1, 'len': 10, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Ibcstatus20x124124::ibc_brktmcposition(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
