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

#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_whltq_0x107_107.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwvcuwhltq0x107107::Gwvcuwhltq0x107107() {}
const int32_t Gwvcuwhltq0x107107::ID = 0x107;

void Gwvcuwhltq0x107107::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_vcu_whltq_0x107_107()->set_checksum_0x107(checksum_0x107(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_whltq_0x107_107()->set_rolling_counter_0x107(rolling_counter_0x107(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_whltq_0x107_107()->set_vcu_kickdown(vcu_kickdown(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_whltq_0x107_107()->set_vcu_accpedalratioerror(vcu_accpedalratioerror(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_whltq_0x107_107()->set_vcu_accpedalratio(vcu_accpedalratio(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_whltq_0x107_107()->set_vcu_accrawpedalratio(vcu_accrawpedalratio(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_whltq_0x107_107()->set_vcu_brkpedalstasus(vcu_brkpedalstasus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_whltq_0x107_107()->set_vcu_minindicatedtorqwhl(vcu_minindicatedtorqwhl(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_whltq_0x107_107()->set_vcu_maxindicatedtorqwhl(vcu_maxindicatedtorqwhl(bytes, length));
}

// config detail: {'name': 'checksum_0x107', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcuwhltq0x107107::checksum_0x107(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x107', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcuwhltq0x107107::rolling_counter_0x107(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_kickdown', 'enum': {0: 'VCU_KICKDOWN_NOTACTIVE', 1: 'VCU_KICKDOWN_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_whltq_0x107_107::Vcu_kickdownType Gwvcuwhltq0x107107::vcu_kickdown(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Gw_vcu_whltq_0x107_107::Vcu_kickdownType ret =  static_cast<Gw_vcu_whltq_0x107_107::Vcu_kickdownType>(x);
  return ret;
}

// config detail: {'name': 'vcu_accpedalratioerror', 'enum': {0: 'VCU_ACCPEDALRATIOERROR_NOERROR', 1: 'VCU_ACCPEDALRATIOERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_whltq_0x107_107::Vcu_accpedalratioerrorType Gwvcuwhltq0x107107::vcu_accpedalratioerror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Gw_vcu_whltq_0x107_107::Vcu_accpedalratioerrorType ret =  static_cast<Gw_vcu_whltq_0x107_107::Vcu_accpedalratioerrorType>(x);
  return ret;
}

// config detail: {'name': 'vcu_accpedalratio', 'offset': 0.0, 'precision': 0.3937, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|99.9998]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Gwvcuwhltq0x107107::vcu_accpedalratio(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.393700;
  return ret;
}

// config detail: {'name': 'vcu_accrawpedalratio', 'offset': 0.0, 'precision': 0.3937, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|99.9998]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Gwvcuwhltq0x107107::vcu_accrawpedalratio(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.393700;
  return ret;
}

// config detail: {'name': 'vcu_brkpedalstasus', 'enum': {0: 'VCU_BRKPEDALSTASUS_NOTPRESSED', 1: 'VCU_BRKPEDALSTASUS_PRESSED', 2: 'VCU_BRKPEDALSTASUS_RESERVED', 3: 'VCU_BRKPEDALSTASUS_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_whltq_0x107_107::Vcu_brkpedalstasusType Gwvcuwhltq0x107107::vcu_brkpedalstasus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  Gw_vcu_whltq_0x107_107::Vcu_brkpedalstasusType ret =  static_cast<Gw_vcu_whltq_0x107_107::Vcu_brkpedalstasusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_minindicatedtorqwhl', 'offset': -30000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-30000|30000]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwvcuwhltq0x107107::vcu_minindicatedtorqwhl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -30000.000000;
  return ret;
}

// config detail: {'name': 'vcu_maxindicatedtorqwhl', 'offset': -30000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-30000|30000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwvcuwhltq0x107107::vcu_maxindicatedtorqwhl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -30000.000000;
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
