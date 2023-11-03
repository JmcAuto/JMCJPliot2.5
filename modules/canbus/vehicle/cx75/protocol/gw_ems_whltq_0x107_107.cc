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

#include "modules/canbus/vehicle/cx75/protocol/gw_ems_whltq_0x107_107.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwemswhltq0x107107::Gwemswhltq0x107107() {}
const int32_t Gwemswhltq0x107107::ID = 0x107;

void Gwemswhltq0x107107::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_ems_whltq_0x107_107()->set_minindicatedtorqwhl(minindicatedtorqwhl(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_whltq_0x107_107()->set_ems_accrawpedalratio(ems_accrawpedalratio(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_whltq_0x107_107()->set_ems_accpedalratio(ems_accpedalratio(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_whltq_0x107_107()->set_dcm_ems_rollingcounter_0x107(dcm_ems_rollingcounter_0x107(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_whltq_0x107_107()->set_ems_kickdown(ems_kickdown(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_whltq_0x107_107()->set_ems_accpedalratioerror(ems_accpedalratioerror(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_whltq_0x107_107()->set_ems_brkpedalstasus(ems_brkpedalstasus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_whltq_0x107_107()->set_dcm_ems_checksum_0x107(dcm_ems_checksum_0x107(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_whltq_0x107_107()->set_maxindicatedtorqwhl(maxindicatedtorqwhl(bytes, length));
}

// config detail: {'name': 'minindicatedtorqwhl', 'enum': {65535: 'MININDICATEDTORQWHL_INVALID'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': -30000.0, 'physical_range': '[-30000|30000]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwemswhltq0x107107::minindicatedtorqwhl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'name': 'ems_accrawpedalratio', 'enum': {255: 'EMS_ACCRAWPEDALRATIO_INVALID'}, 'precision': 0.3937, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|99.9998]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
double Gwemswhltq0x107107::ems_accrawpedalratio(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'ems_accpedalratio', 'enum': {255: 'EMS_ACCPEDALRATIO_INVALID'}, 'precision': 0.3937, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|99.9998]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': '%'}
double Gwemswhltq0x107107::ems_accpedalratio(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'dcm_ems_rollingcounter_0x107', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwemswhltq0x107107::dcm_ems_rollingcounter_0x107(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ems_kickdown', 'enum': {0: 'EMS_KICKDOWN_NOTACTIVE', 1: 'EMS_KICKDOWN_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_whltq_0x107_107::Ems_kickdownType Gwemswhltq0x107107::ems_kickdown(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Gw_ems_whltq_0x107_107::Ems_kickdownType ret =  static_cast<Gw_ems_whltq_0x107_107::Ems_kickdownType>(x);
  return ret;
}

// config detail: {'name': 'ems_accpedalratioerror', 'enum': {0: 'EMS_ACCPEDALRATIOERROR_NOERROR', 1: 'EMS_ACCPEDALRATIOERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_whltq_0x107_107::Ems_accpedalratioerrorType Gwemswhltq0x107107::ems_accpedalratioerror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Gw_ems_whltq_0x107_107::Ems_accpedalratioerrorType ret =  static_cast<Gw_ems_whltq_0x107_107::Ems_accpedalratioerrorType>(x);
  return ret;
}

// config detail: {'name': 'ems_brkpedalstasus', 'enum': {0: 'EMS_BRKPEDALSTASUS_NOTPRESSED', 1: 'EMS_BRKPEDALSTASUS_PRESSED', 2: 'EMS_BRKPEDALSTASUS_RESERVED', 3: 'EMS_BRKPEDALSTASUS_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_whltq_0x107_107::Ems_brkpedalstasusType Gwemswhltq0x107107::ems_brkpedalstasus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  Gw_ems_whltq_0x107_107::Ems_brkpedalstasusType ret =  static_cast<Gw_ems_whltq_0x107_107::Ems_brkpedalstasusType>(x);
  return ret;
}

// config detail: {'name': 'dcm_ems_checksum_0x107', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwemswhltq0x107107::dcm_ems_checksum_0x107(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'maxindicatedtorqwhl', 'enum': {65535: 'MAXINDICATEDTORQWHL_INVALID'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': -30000.0, 'physical_range': '[-30000|30000]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwemswhltq0x107107::maxindicatedtorqwhl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
