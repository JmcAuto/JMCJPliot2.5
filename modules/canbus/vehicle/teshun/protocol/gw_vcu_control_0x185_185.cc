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

#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_control_0x185_185.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwvcucontrol0x185185::Gwvcucontrol0x185185() {}
const int32_t Gwvcucontrol0x185185::ID = 0x185;

void Gwvcucontrol0x185185::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_vcu_control_0x185_185()->set_checksum_0x185(checksum_0x185(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control_0x185_185()->set_rolling_counter_0x185(rolling_counter_0x185(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control_0x185_185()->set_vcu_ems_engoperationstatus(vcu_ems_engoperationstatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control_0x185_185()->set_vcu_maxchgpwr_alw(vcu_maxchgpwr_alw(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control_0x185_185()->set_vcu_maxdischgpwr_alw(vcu_maxdischgpwr_alw(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control_0x185_185()->set_vcu_mcu_asd_req(vcu_mcu_asd_req(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_control_0x185_185()->set_vcu_mcu_motor1_ctrmode_req(vcu_mcu_motor1_ctrmode_req(bytes, length));
}

// config detail: {'name': 'checksum_0x185', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcucontrol0x185185::checksum_0x185(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x185', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcucontrol0x185185::rolling_counter_0x185(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_ems_engoperationstatus', 'enum': {0: 'VCU_EMS_ENGOPERATIONSTATUS_STOPPED', 1: 'VCU_EMS_ENGOPERATIONSTATUS_RUNNING', 2: 'VCU_EMS_ENGOPERATIONSTATUS_IDLE', 3: 'VCU_EMS_ENGOPERATIONSTATUS_DFCO', 4: 'VCU_EMS_ENGOPERATIONSTATUS_CRANKING', 5: 'VCU_EMS_ENGOPERATIONSTATUS_STALLING'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_control_0x185_185::Vcu_ems_engoperationstatusType Gwvcucontrol0x185185::vcu_ems_engoperationstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 3);

  Gw_vcu_control_0x185_185::Vcu_ems_engoperationstatusType ret =  static_cast<Gw_vcu_control_0x185_185::Vcu_ems_engoperationstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_maxchgpwr_alw', 'offset': 0.0, 'precision': 0.1, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|400]', 'bit': 11, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kw'}
double Gwvcucontrol0x185185::vcu_maxchgpwr_alw(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'vcu_maxdischgpwr_alw', 'offset': 0.0, 'precision': 0.1, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|400]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kw'}
double Gwvcucontrol0x185185::vcu_maxdischgpwr_alw(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'vcu_mcu_asd_req', 'enum': {0: 'VCU_MCU_ASD_REQ_ON', 1: 'VCU_MCU_ASD_REQ_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 36, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_control_0x185_185::Vcu_mcu_asd_reqType Gwvcucontrol0x185185::vcu_mcu_asd_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 1);

  Gw_vcu_control_0x185_185::Vcu_mcu_asd_reqType ret =  static_cast<Gw_vcu_control_0x185_185::Vcu_mcu_asd_reqType>(x);
  return ret;
}

// config detail: {'name': 'vcu_mcu_motor1_ctrmode_req', 'enum': {0: 'VCU_MCU_MOTOR1_CTRMODE_REQ_DISABLE', 1: 'VCU_MCU_MOTOR1_CTRMODE_REQ_STANDBY', 2: 'VCU_MCU_MOTOR1_CTRMODE_REQ_SPEEDMODE', 3: 'VCU_MCU_MOTOR1_CTRMODE_REQ_TORQUEMODE', 4: 'VCU_MCU_MOTOR1_CTRMODE_REQ_ANTISLIP', 5: 'VCU_MCU_MOTOR1_CTRMODE_REQ_ALOFFSETCAL'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_control_0x185_185::Vcu_mcu_motor1_ctrmode_reqType Gwvcucontrol0x185185::vcu_mcu_motor1_ctrmode_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 3);

  Gw_vcu_control_0x185_185::Vcu_mcu_motor1_ctrmode_reqType ret =  static_cast<Gw_vcu_control_0x185_185::Vcu_mcu_motor1_ctrmode_reqType>(x);
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
