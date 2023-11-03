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

#include "modules/canbus/vehicle/teshun/protocol/gw_mcu_power_0x226_226.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwmcupower0x226226::Gwmcupower0x226226() {}
const int32_t Gwmcupower0x226226::ID = 0x226;

void Gwmcupower0x226226::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_mcu_power_0x226_226()->set_checksum_0x226(checksum_0x226(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_power_0x226_226()->set_rolling_counter_0x226(rolling_counter_0x226(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_power_0x226_226()->set_mcu_serialnum_est(mcu_serialnum_est(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_power_0x226_226()->set_mcu_tm04_trqmindyn(mcu_tm04_trqmindyn(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_power_0x226_226()->set_mcu_tm04_trqmaxdyn(mcu_tm04_trqmaxdyn(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_power_0x226_226()->set_mcu_maxtrq_est(mcu_maxtrq_est(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_power_0x226_226()->set_mcu_mintrq_est(mcu_mintrq_est(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_power_0x226_226()->set_mcu_sys_sts(mcu_sys_sts(bytes, length));
}

// config detail: {'name': 'checksum_0x226', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwmcupower0x226226::checksum_0x226(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x226', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwmcupower0x226226::rolling_counter_0x226(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'mcu_serialnum_est', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 47, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Gwmcupower0x226226::mcu_serialnum_est(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'mcu_tm04_trqmindyn', 'offset': -1023.0, 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range': '[-1023|1024]', 'bit': 46, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwmcupower0x226226::mcu_tm04_trqmindyn(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 7);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x + -1023.000000;
  return ret;
}

// config detail: {'name': 'mcu_tm04_trqmaxdyn', 'offset': -1023.0, 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range': '[-1023|1024]', 'bit': 4, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwmcupower0x226226::mcu_tm04_trqmaxdyn(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(2, 6);
  x <<= 6;
  x |= t;

  int ret = x + -1023.000000;
  return ret;
}

// config detail: {'name': 'mcu_maxtrq_est', 'offset': 0.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|2000]', 'bit': 27, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwmcupower0x226226::mcu_maxtrq_est(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'mcu_mintrq_est', 'offset': -2000.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2000|0]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwmcupower0x226226::mcu_mintrq_est(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x + -2000.000000;
  return ret;
}

// config detail: {'name': 'mcu_sys_sts', 'enum': {0: 'MCU_SYS_STS_INIT', 1: 'MCU_SYS_STS_OK', 2: 'MCU_SYS_STS_WARNING', 3: 'MCU_SYS_STS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mcu_power_0x226_226::Mcu_sys_stsType Gwmcupower0x226226::mcu_sys_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Gw_mcu_power_0x226_226::Mcu_sys_stsType ret =  static_cast<Gw_mcu_power_0x226_226::Mcu_sys_stsType>(x);
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
