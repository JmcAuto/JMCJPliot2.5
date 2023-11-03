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

#include "modules/canbus/vehicle/cx75/protocol/gw_ic_0x510_510.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwic0x510510::Gwic0x510510() {}
const int32_t Gwic0x510510::ID = 0x510;

void Gwic0x510510::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_ic_0x510_510()->set_ic_airbagtelltalebehavior(ic_airbagtelltalebehavior(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ic_0x510_510()->set_ic_vehbrkpump_err_ic(ic_vehbrkpump_err_ic(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ic_0x510_510()->set_ic_disfail(ic_disfail(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ic_0x510_510()->set_ic_qdashaccfail(ic_qdashaccfail(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ic_0x510_510()->set_ic_vehspd_hmi(ic_vehspd_hmi(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ic_0x510_510()->set_ic_rolling_counter_0x510(ic_rolling_counter_0x510(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ic_0x510_510()->set_ic_checksum_0x510(ic_checksum_0x510(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ic_0x510_510()->set_ic_odometermastervalue(ic_odometermastervalue(bytes, length));
}

// config detail: {'name': 'ic_airbagtelltalebehavior', 'enum': {0: 'IC_AIRBAGTELLTALEBEHAVIOR_NO_FAILURE_IN_LAMP_AND_LAMP_IS_OFF', 1: 'IC_AIRBAGTELLTALEBEHAVIOR_FAILURE_IN_LAMP', 2: 'IC_AIRBAGTELLTALEBEHAVIOR_NO_FAILURE_IN_THE_LAMP_LAMP_IS_ON', 3: 'IC_AIRBAGTELLTALEBEHAVIOR_NO_FAILURE_IN_THE_LAMP_LAMP_IS_BLINKING', 4: 'IC_AIRBAGTELLTALEBEHAVIOR_AIRBAGFAILSTS_SIGNAL_NOT_RECEIVED', 5: 'IC_AIRBAGTELLTALEBEHAVIOR_INVALID', 6: 'IC_AIRBAGTELLTALEBEHAVIOR_INVALID', 7: 'IC_AIRBAGTELLTALEBEHAVIOR_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 24, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ic_0x510_510::Ic_airbagtelltalebehaviorType Gwic0x510510::ic_airbagtelltalebehavior(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  Gw_ic_0x510_510::Ic_airbagtelltalebehaviorType ret =  static_cast<Gw_ic_0x510_510::Ic_airbagtelltalebehaviorType>(x);
  return ret;
}

// config detail: {'name': 'ic_vehbrkpump_err_ic', 'enum': {0: 'IC_VEHBRKPUMP_ERR_IC_NORMAL', 1: 'IC_VEHBRKPUMP_ERR_IC_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ic_0x510_510::Ic_vehbrkpump_err_icType Gwic0x510510::ic_vehbrkpump_err_ic(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 1);

  Gw_ic_0x510_510::Ic_vehbrkpump_err_icType ret =  static_cast<Gw_ic_0x510_510::Ic_vehbrkpump_err_icType>(x);
  return ret;
}

// config detail: {'name': 'ic_disfail', 'enum': {0: 'IC_DISFAIL_NO_ERROR', 1: 'IC_DISFAIL_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ic_0x510_510::Ic_disfailType Gwic0x510510::ic_disfail(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  Gw_ic_0x510_510::Ic_disfailType ret =  static_cast<Gw_ic_0x510_510::Ic_disfailType>(x);
  return ret;
}

// config detail: {'name': 'ic_qdashaccfail', 'enum': {0: 'IC_QDASHACCFAIL_NO_ERROR', 1: 'IC_QDASHACCFAIL_REVERSIBLE_ERROR', 2: 'IC_QDASHACCFAIL_IRREVERSIBLE_ERROR', 3: 'IC_QDASHACCFAIL_NOT_DEFINED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ic_0x510_510::Ic_qdashaccfailType Gwic0x510510::ic_qdashaccfail(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(6, 2);

  Gw_ic_0x510_510::Ic_qdashaccfailType ret =  static_cast<Gw_ic_0x510_510::Ic_qdashaccfailType>(x);
  return ret;
}

// config detail: {'name': 'ic_vehspd_hmi', 'enum': {511: 'IC_VEHSPD_HMI_INVALID'}, 'precision': 1.0, 'len': 9, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|500]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'kph'}
int Gwic0x510510::ic_vehspd_hmi(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'name': 'ic_rolling_counter_0x510', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwic0x510510::ic_rolling_counter_0x510(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ic_checksum_0x510', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwic0x510510::ic_checksum_0x510(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ic_odometermastervalue', 'enum': {16777215: 'IC_ODOMETERMASTERVALUE_INVALID'}, 'precision': 0.1, 'len': 24, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|999999]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'km'}
double Gwic0x510510::ic_odometermastervalue(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
