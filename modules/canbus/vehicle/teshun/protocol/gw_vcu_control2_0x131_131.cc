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

#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_control2_0x131_131.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwvcucontrol20x131131::Gwvcucontrol20x131131() {}
const int32_t Gwvcucontrol20x131131::ID = 0x131;

void Gwvcucontrol20x131131::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_vcu_control2_0x131_131()->set_checksum_0x131(checksum_0x131(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control2_0x131_131()->set_rolling_counter_0x131(rolling_counter_0x131(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control2_0x131_131()->set_vcu_sts_vcu(vcu_sts_vcu(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control2_0x131_131()->set_vcu_brake_flag(vcu_brake_flag(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control2_0x131_131()->set_vcu_tractor_flag(vcu_tractor_flag(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control2_0x131_131()->set_vcu_brkpedpos_meas_vcu(vcu_brkpedpos_meas_vcu(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control2_0x131_131()->set_vcu_pwt_mode_dash(vcu_pwt_mode_dash(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control2_0x131_131()->set_vcu_mcu_motor1_capdischarge_req(vcu_mcu_motor1_capdischarge_req(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control2_0x131_131()->set_vcu_mcu_motor1_trq_req(vcu_mcu_motor1_trq_req(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control2_0x131_131()->set_vcu_mcu_motor1_spd_req(vcu_mcu_motor1_spd_req(bytes, length));
}

// config detail: {'name': 'checksum_0x131', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcucontrol20x131131::checksum_0x131(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x131', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcucontrol20x131131::rolling_counter_0x131(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_sts_vcu', 'enum': {0: 'VCU_STS_VCU_INITIALIZING', 1: 'VCU_STS_VCU_READY', 2: 'VCU_STS_VCU_WARNING', 3: 'VCU_STS_VCU_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_control2_0x131_131::Vcu_sts_vcuType Gwvcucontrol20x131131::vcu_sts_vcu(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  Gw_vcu_control2_0x131_131::Vcu_sts_vcuType ret =  static_cast<Gw_vcu_control2_0x131_131::Vcu_sts_vcuType>(x);
  return ret;
}

// config detail: {'name': 'vcu_brake_flag', 'enum': {0: 'VCU_BRAKE_FLAG_NO_ACTIVE', 1: 'VCU_BRAKE_FLAG_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_control2_0x131_131::Vcu_brake_flagType Gwvcucontrol20x131131::vcu_brake_flag(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  Gw_vcu_control2_0x131_131::Vcu_brake_flagType ret =  static_cast<Gw_vcu_control2_0x131_131::Vcu_brake_flagType>(x);
  return ret;
}

// config detail: {'name': 'vcu_tractor_flag', 'enum': {0: 'VCU_TRACTOR_FLAG_NO_ACTIVE', 1: 'VCU_TRACTOR_FLAG_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_control2_0x131_131::Vcu_tractor_flagType Gwvcucontrol20x131131::vcu_tractor_flag(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 1);

  Gw_vcu_control2_0x131_131::Vcu_tractor_flagType ret =  static_cast<Gw_vcu_control2_0x131_131::Vcu_tractor_flagType>(x);
  return ret;
}

// config detail: {'name': 'vcu_brkpedpos_meas_vcu', 'offset': 0.0, 'precision': 0.1, 'len': 10, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 35, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Gwvcucontrol20x131131::vcu_brkpedpos_meas_vcu(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(2, 6);
  x <<= 6;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'vcu_pwt_mode_dash', 'enum': {0: 'VCU_PWT_MODE_DASH_ECOHEV', 1: 'VCU_PWT_MODE_DASH_ECO_EV', 2: 'VCU_PWT_MODE_DASH_PWRHEV', 3: 'VCU_PWT_MODE_DASH_PWREV', 4: 'VCU_PWT_MODE_DASH_EV_ECO', 5: 'VCU_PWT_MODE_DASH_EV_POWER', 7: 'VCU_PWT_MODE_DASH_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_control2_0x131_131::Vcu_pwt_mode_dashType Gwvcucontrol20x131131::vcu_pwt_mode_dash(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 3);

  Gw_vcu_control2_0x131_131::Vcu_pwt_mode_dashType ret =  static_cast<Gw_vcu_control2_0x131_131::Vcu_pwt_mode_dashType>(x);
  return ret;
}

// config detail: {'name': 'vcu_mcu_motor1_capdischarge_req', 'enum': {0: 'VCU_MCU_MOTOR1_CAPDISCHARGE_REQ_NO_REQUEST', 1: 'VCU_MCU_MOTOR1_CAPDISCHARGE_REQ_DISCHARGE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 36, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_control2_0x131_131::Vcu_mcu_motor1_capdischarge_reqType Gwvcucontrol20x131131::vcu_mcu_motor1_capdischarge_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 1);

  Gw_vcu_control2_0x131_131::Vcu_mcu_motor1_capdischarge_reqType ret =  static_cast<Gw_vcu_control2_0x131_131::Vcu_mcu_motor1_capdischarge_reqType>(x);
  return ret;
}

// config detail: {'name': 'vcu_mcu_motor1_trq_req', 'offset': -1000.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|1000]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
double Gwvcucontrol20x131131::vcu_mcu_motor1_trq_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -1000.000000;
  return ret;
}

// config detail: {'name': 'vcu_mcu_motor1_spd_req', 'offset': -10000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-10000|10000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'rpm'}
int Gwvcucontrol20x131131::vcu_mcu_motor1_spd_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -10000.000000;
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
