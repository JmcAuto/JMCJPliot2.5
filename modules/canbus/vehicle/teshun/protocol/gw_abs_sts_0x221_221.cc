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

#include "modules/canbus/vehicle/teshun/protocol/gw_abs_sts_0x221_221.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwabssts0x221221::Gwabssts0x221221() {}
const int32_t Gwabssts0x221221::ID = 0x221;

void Gwabssts0x221221::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_checksum_0x221(checksum_0x221(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_rolling_counter_0x221(rolling_counter_0x221(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_abs_whlmilgfrntlestatus(abs_whlmilgfrntlestatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_abs_whlmilgfrntristatus(abs_whlmilgfrntristatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_abs_vehspdlgtstatus(abs_vehspdlgtstatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_abs_vehspddirection(abs_vehspddirection(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_abs_vehspdlgt(abs_vehspdlgt(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_abs_ebdflgflt(abs_ebdflgflt(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_abs_absflgflt(abs_absflgflt(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_abs_absctrlactv(abs_absctrlactv(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_abs_whlmilgfrntri(abs_whlmilgfrntri(bytes, length));
  chassis->mutable_teshun()->mutable_gw_abs_sts_0x221_221()->set_abs_whlmilgfrntle(abs_whlmilgfrntle(bytes, length));
}

// config detail: {'name': 'checksum_0x221', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwabssts0x221221::checksum_0x221(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x221', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwabssts0x221221::rolling_counter_0x221(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'abs_whlmilgfrntlestatus', 'enum': {0: 'ABS_WHLMILGFRNTLESTATUS_VALID', 1: 'ABS_WHLMILGFRNTLESTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_sts_0x221_221::Abs_whlmilgfrntlestatusType Gwabssts0x221221::abs_whlmilgfrntlestatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Gw_abs_sts_0x221_221::Abs_whlmilgfrntlestatusType ret =  static_cast<Gw_abs_sts_0x221_221::Abs_whlmilgfrntlestatusType>(x);
  return ret;
}

// config detail: {'name': 'abs_whlmilgfrntristatus', 'enum': {0: 'ABS_WHLMILGFRNTRISTATUS_VALID', 1: 'ABS_WHLMILGFRNTRISTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_sts_0x221_221::Abs_whlmilgfrntristatusType Gwabssts0x221221::abs_whlmilgfrntristatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Gw_abs_sts_0x221_221::Abs_whlmilgfrntristatusType ret =  static_cast<Gw_abs_sts_0x221_221::Abs_whlmilgfrntristatusType>(x);
  return ret;
}

// config detail: {'name': 'abs_vehspdlgtstatus', 'enum': {0: 'ABS_VEHSPDLGTSTATUS_VALID', 1: 'ABS_VEHSPDLGTSTATUS_INVALID', 2: 'ABS_VEHSPDLGTSTATUS_INIT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_sts_0x221_221::Abs_vehspdlgtstatusType Gwabssts0x221221::abs_vehspdlgtstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  Gw_abs_sts_0x221_221::Abs_vehspdlgtstatusType ret =  static_cast<Gw_abs_sts_0x221_221::Abs_vehspdlgtstatusType>(x);
  return ret;
}

// config detail: {'name': 'abs_vehspddirection', 'enum': {0: 'ABS_VEHSPDDIRECTION_FORWARD', 1: 'ABS_VEHSPDDIRECTION_BACKWARD'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 36, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_sts_0x221_221::Abs_vehspddirectionType Gwabssts0x221221::abs_vehspddirection(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 1);

  Gw_abs_sts_0x221_221::Abs_vehspddirectionType ret =  static_cast<Gw_abs_sts_0x221_221::Abs_vehspddirectionType>(x);
  return ret;
}

// config detail: {'name': 'abs_vehspdlgt', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 35, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
double Gwabssts0x221221::abs_vehspdlgt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'name': 'abs_ebdflgflt', 'enum': {0: 'ABS_EBDFLGFLT_NO_FAILURE', 1: 'ABS_EBDFLGFLT_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_sts_0x221_221::Abs_ebdflgfltType Gwabssts0x221221::abs_ebdflgflt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 1);

  Gw_abs_sts_0x221_221::Abs_ebdflgfltType ret =  static_cast<Gw_abs_sts_0x221_221::Abs_ebdflgfltType>(x);
  return ret;
}

// config detail: {'name': 'abs_absflgflt', 'enum': {0: 'ABS_ABSFLGFLT_NO_FAILURE', 1: 'ABS_ABSFLGFLT_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 38, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_sts_0x221_221::Abs_absflgfltType Gwabssts0x221221::abs_absflgflt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 1);

  Gw_abs_sts_0x221_221::Abs_absflgfltType ret =  static_cast<Gw_abs_sts_0x221_221::Abs_absflgfltType>(x);
  return ret;
}

// config detail: {'name': 'abs_absctrlactv', 'enum': {0: 'ABS_ABSCTRLACTV_NOT_ACTIVE', 1: 'ABS_ABSCTRLACTV_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_abs_sts_0x221_221::Abs_absctrlactvType Gwabssts0x221221::abs_absctrlactv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  Gw_abs_sts_0x221_221::Abs_absctrlactvType ret =  static_cast<Gw_abs_sts_0x221_221::Abs_absctrlactvType>(x);
  return ret;
}

// config detail: {'name': 'abs_whlmilgfrntri', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwabssts0x221221::abs_whlmilgfrntri(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'abs_whlmilgfrntle', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwabssts0x221221::abs_whlmilgfrntle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
