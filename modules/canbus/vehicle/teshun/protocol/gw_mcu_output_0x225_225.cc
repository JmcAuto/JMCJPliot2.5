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

#include "modules/canbus/vehicle/teshun/protocol/gw_mcu_output_0x225_225.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwmcuoutput0x225225::Gwmcuoutput0x225225() {}
const int32_t Gwmcuoutput0x225225::ID = 0x225;

void Gwmcuoutput0x225225::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_mcu_output_0x225_225()->set_checksum_0x225(checksum_0x225(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_output_0x225_225()->set_rolling_counter_0x225(rolling_counter_0x225(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_output_0x225_225()->set_mcu_work_sts(mcu_work_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_output_0x225_225()->set_mcu_ctrmode_sts(mcu_ctrmode_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_output_0x225_225()->set_mcu_capacitor_sts(mcu_capacitor_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_output_0x225_225()->set_mcu_spd_meas(mcu_spd_meas(bytes, length));
  chassis->mutable_teshun()->mutable_gw_mcu_output_0x225_225()->set_mcu_trq_meas(mcu_trq_meas(bytes, length));
}

// config detail: {'name': 'checksum_0x225', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwmcuoutput0x225225::checksum_0x225(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x225', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwmcuoutput0x225225::rolling_counter_0x225(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'mcu_work_sts', 'enum': {0: 'MCU_WORK_STS_CONSUM', 1: 'MCU_WORK_STS_GENERATE', 2: 'MCU_WORK_STS_OFF', 3: 'MCU_WORK_STS_READY', 4: 'MCU_WORK_STS_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mcu_output_0x225_225::Mcu_work_stsType Gwmcuoutput0x225225::mcu_work_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 3);

  Gw_mcu_output_0x225_225::Mcu_work_stsType ret =  static_cast<Gw_mcu_output_0x225_225::Mcu_work_stsType>(x);
  return ret;
}

// config detail: {'name': 'mcu_ctrmode_sts', 'enum': {0: 'MCU_CTRMODE_STS_INITIALIZATION', 1: 'MCU_CTRMODE_STS_PRECHARGE', 2: 'MCU_CTRMODE_STS_DISABLE', 3: 'MCU_CTRMODE_STS_STANDBY', 4: 'MCU_CTRMODE_STS_ANTISLIP', 5: 'MCU_CTRMODE_STS_ALOFFSETCAL', 7: 'MCU_CTRMODE_STS_NCTLINT', 8: 'MCU_CTRMODE_STS_TRQCT', 9: 'MCU_CTRMODE_STS_ASCACTIVE', 11: 'MCU_CTRMODE_STS_AFTERRUN', 12: 'MCU_CTRMODE_STS_PREFAILURE', 13: 'MCU_CTRMODE_STS_FAILURE', 14: 'MCU_CTRMODE_STS_DISCHARGE'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mcu_output_0x225_225::Mcu_ctrmode_stsType Gwmcuoutput0x225225::mcu_ctrmode_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Gw_mcu_output_0x225_225::Mcu_ctrmode_stsType ret =  static_cast<Gw_mcu_output_0x225_225::Mcu_ctrmode_stsType>(x);
  return ret;
}

// config detail: {'name': 'mcu_capacitor_sts', 'enum': {0: 'MCU_CAPACITOR_STS_FORBIDCHARGE', 1: 'MCU_CAPACITOR_STS_WAITCHARGE', 2: 'MCU_CAPACITOR_STS_ALLOWDISCHARGE', 3: 'MCU_CAPACITOR_STS_FORBIDDISCHARGE', 4: 'MCU_CAPACITOR_STS_ERRORDISCHARGE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mcu_output_0x225_225::Mcu_capacitor_stsType Gwmcuoutput0x225225::mcu_capacitor_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 3);

  Gw_mcu_output_0x225_225::Mcu_capacitor_stsType ret =  static_cast<Gw_mcu_output_0x225_225::Mcu_capacitor_stsType>(x);
  return ret;
}

// config detail: {'name': 'mcu_spd_meas', 'offset': -15000.0, 'precision': 1.0, 'len': 15, 'is_signed_var': False, 'physical_range': '[-15000|15000]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': 'rpm'}
int Gwmcuoutput0x225225::mcu_spd_meas(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  int ret = x + -15000.000000;
  return ret;
}

// config detail: {'name': 'mcu_trq_meas', 'offset': -2000.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2000|2000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
int Gwmcuoutput0x225225::mcu_trq_meas(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x + -2000.000000;
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
