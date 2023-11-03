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

#include "modules/canbus/vehicle/teshun/protocol/gw_scu_shiftersts_0xc8_c8.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwscushiftersts0xc8c8::Gwscushiftersts0xc8c8() {}
const int32_t Gwscushiftersts0xc8c8::ID = 0xC8;

void Gwscushiftersts0xc8c8::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_scu_shiftersts_0xc8_c8()->set_checksum_0xc8(checksum_0xc8(bytes, length));
  chassis->mutable_teshun()->mutable_gw_scu_shiftersts_0xc8_c8()->set_rolling_counter_0xc8(rolling_counter_0xc8(bytes, length));
  chassis->mutable_teshun()->mutable_gw_scu_shiftersts_0xc8_c8()->set_scu_shifterlockstatus(scu_shifterlockstatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_scu_shiftersts_0xc8_c8()->set_shifterlockstatus(shifterlockstatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_scu_shiftersts_0xc8_c8()->set_shifterpositionfailure(shifterpositionfailure(bytes, length));
  chassis->mutable_teshun()->mutable_gw_scu_shiftersts_0xc8_c8()->set_shifterposition(shifterposition(bytes, length));
  chassis->mutable_teshun()->mutable_gw_scu_shiftersts_0xc8_c8()->set_shifterpositioninv(shifterpositioninv(bytes, length));
  chassis->mutable_teshun()->mutable_gw_scu_shiftersts_0xc8_c8()->set_sys_sts_scu(sys_sts_scu(bytes, length));
}

// config detail: {'name': 'checksum_0xc8', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwscushiftersts0xc8c8::checksum_0xc8(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0xc8', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwscushiftersts0xc8c8::rolling_counter_0xc8(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'scu_shifterlockstatus', 'enum': {0: 'SCU_SHIFTERLOCKSTATUS_UNLOCKED', 1: 'SCU_SHIFTERLOCKSTATUS_LOCKED', 3: 'SCU_SHIFTERLOCKSTATUS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_scu_shiftersts_0xc8_c8::Scu_shifterlockstatusType Gwscushiftersts0xc8c8::scu_shifterlockstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 2);

  Gw_scu_shiftersts_0xc8_c8::Scu_shifterlockstatusType ret =  static_cast<Gw_scu_shiftersts_0xc8_c8::Scu_shifterlockstatusType>(x);
  return ret;
}

// config detail: {'name': 'shifterlockstatus', 'enum': {0: 'SHIFTERLOCKSTATUS_UNLOCKED', 1: 'SHIFTERLOCKSTATUS_LOCKED', 3: 'SHIFTERLOCKSTATUS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_scu_shiftersts_0xc8_c8::ShifterlockstatusType Gwscushiftersts0xc8c8::shifterlockstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Gw_scu_shiftersts_0xc8_c8::ShifterlockstatusType ret =  static_cast<Gw_scu_shiftersts_0xc8_c8::ShifterlockstatusType>(x);
  return ret;
}

// config detail: {'name': 'shifterpositionfailure', 'enum': {0: 'SHIFTERPOSITIONFAILURE_NOFAULT', 1: 'SHIFTERPOSITIONFAILURE_MODESELECTORSENSORFAIL', 2: 'SHIFTERPOSITIONFAILURE_ROTARYPOSITIONSENSORFAIL', 3: 'SHIFTERPOSITIONFAILURE_CANBUSCOMMUNICATION', 4: 'SHIFTERPOSITIONFAILURE_SOLENOIDFAIL'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_scu_shiftersts_0xc8_c8::ShifterpositionfailureType Gwscushiftersts0xc8c8::shifterpositionfailure(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Gw_scu_shiftersts_0xc8_c8::ShifterpositionfailureType ret =  static_cast<Gw_scu_shiftersts_0xc8_c8::ShifterpositionfailureType>(x);
  return ret;
}

// config detail: {'name': 'shifterposition', 'enum': {0: 'SHIFTERPOSITION_ERROR', 3: 'SHIFTERPOSITION_MANUAL_MODE', 5: 'SHIFTERPOSITION_DRIVE', 6: 'SHIFTERPOSITION_NEUTRAL', 7: 'SHIFTERPOSITION_REVERSE', 8: 'SHIFTERPOSITION_PARK', 9: 'SHIFTERPOSITION_UPSHIFT', 10: 'SHIFTERPOSITION_DOWNSHIFT'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_scu_shiftersts_0xc8_c8::ShifterpositionType Gwscushiftersts0xc8c8::shifterposition(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 4);

  Gw_scu_shiftersts_0xc8_c8::ShifterpositionType ret =  static_cast<Gw_scu_shiftersts_0xc8_c8::ShifterpositionType>(x);
  return ret;
}

// config detail: {'name': 'shifterpositioninv', 'enum': {5: 'SHIFTERPOSITIONINV_DOWNSHIFT', 6: 'SHIFTERPOSITIONINV_UPSHIFT', 7: 'SHIFTERPOSITIONINV_PARK', 8: 'SHIFTERPOSITIONINV_REVERSE', 9: 'SHIFTERPOSITIONINV_NEUTRAL', 10: 'SHIFTERPOSITIONINV_DRIVE', 12: 'SHIFTERPOSITIONINV_MANUAL_MODE', 15: 'SHIFTERPOSITIONINV_ERROR'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_scu_shiftersts_0xc8_c8::ShifterpositioninvType Gwscushiftersts0xc8c8::shifterpositioninv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 4);

  Gw_scu_shiftersts_0xc8_c8::ShifterpositioninvType ret =  static_cast<Gw_scu_shiftersts_0xc8_c8::ShifterpositioninvType>(x);
  return ret;
}

// config detail: {'name': 'sys_sts_scu', 'enum': {0: 'SYS_STS_SCU_INIT', 1: 'SYS_STS_SCU_OK', 2: 'SYS_STS_SCU_WARNING', 3: 'SYS_STS_SCU_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_scu_shiftersts_0xc8_c8::Sys_sts_scuType Gwscushiftersts0xc8c8::sys_sts_scu(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Gw_scu_shiftersts_0xc8_c8::Sys_sts_scuType ret =  static_cast<Gw_scu_shiftersts_0xc8_c8::Sys_sts_scuType>(x);
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
