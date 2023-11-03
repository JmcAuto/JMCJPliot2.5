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

#include "modules/canbus/vehicle/teshun/protocol/gw_abs_0x211_211.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwabs0x211211::Gwabs0x211211() {}
const int32_t Gwabs0x211211::ID = 0x211;

void Gwabs0x211211::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_abs_0x211_211()->set_checksum_0x211(checksum_0x211(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_0x211_211()->set_rollingcounter_0x211(rollingcounter_0x211(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_0x211_211()->set_abs_whlspdreristatus(abs_whlspdreristatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_0x211_211()->set_abs_whlspdrelestatus(abs_whlspdrelestatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_0x211_211()->set_abs_whlspdfrntristatus(abs_whlspdfrntristatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_0x211_211()->set_abs_whlspdfrntlestatus(abs_whlspdfrntlestatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_0x211_211()->set_abs_whlspdreri(abs_whlspdreri(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_0x211_211()->set_abs_whlspdrele(abs_whlspdrele(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_0x211_211()->set_abs_whlspdfrntri(abs_whlspdfrntri(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_0x211_211()->set_abs_whlspdfrntle(abs_whlspdfrntle(bytes, length));
}

// config detail: {'name': 'checksum_0x211', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwabs0x211211::checksum_0x211(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rollingcounter_0x211', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwabs0x211211::rollingcounter_0x211(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'abs_whlspdreristatus', 'enum': {0: 'ABS_WHLSPDRERISTATUS_VALID', 1: 'ABS_WHLSPDRERISTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_0x211_211::Abs_whlspdreristatusType Gwabs0x211211::abs_whlspdreristatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Gw_abs_0x211_211::Abs_whlspdreristatusType ret =  static_cast<Gw_abs_0x211_211::Abs_whlspdreristatusType>(x);
  return ret;
}

// config detail: {'name': 'abs_whlspdrelestatus', 'enum': {0: 'ABS_WHLSPDRELESTATUS_VALID', 1: 'ABS_WHLSPDRELESTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_0x211_211::Abs_whlspdrelestatusType Gwabs0x211211::abs_whlspdrelestatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Gw_abs_0x211_211::Abs_whlspdrelestatusType ret =  static_cast<Gw_abs_0x211_211::Abs_whlspdrelestatusType>(x);
  return ret;
}

// config detail: {'name': 'abs_whlspdfrntristatus', 'enum': {0: 'ABS_WHLSPDFRNTRISTATUS_VALID', 1: 'ABS_WHLSPDFRNTRISTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_0x211_211::Abs_whlspdfrntristatusType Gwabs0x211211::abs_whlspdfrntristatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Gw_abs_0x211_211::Abs_whlspdfrntristatusType ret =  static_cast<Gw_abs_0x211_211::Abs_whlspdfrntristatusType>(x);
  return ret;
}

// config detail: {'name': 'abs_whlspdfrntlestatus', 'enum': {0: 'ABS_WHLSPDFRNTLESTATUS_VALID', 1: 'ABS_WHLSPDFRNTLESTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_0x211_211::Abs_whlspdfrntlestatusType Gwabs0x211211::abs_whlspdfrntlestatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  Gw_abs_0x211_211::Abs_whlspdfrntlestatusType ret =  static_cast<Gw_abs_0x211_211::Abs_whlspdfrntlestatusType>(x);
  return ret;
}

// config detail: {'name': 'abs_whlspdreri', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 35, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
double Gwabs0x211211::abs_whlspdreri(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'name': 'abs_whlspdrele', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
double Gwabs0x211211::abs_whlspdrele(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'name': 'abs_whlspdfrntri', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 11, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
double Gwabs0x211211::abs_whlspdfrntri(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'name': 'abs_whlspdfrntle', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
double Gwabs0x211211::abs_whlspdfrntle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
