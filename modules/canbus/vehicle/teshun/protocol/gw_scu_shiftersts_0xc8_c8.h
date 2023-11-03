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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_SCU_SHIFTERSTS_0XC8_C8_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_SCU_SHIFTERSTS_0XC8_C8_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwscushiftersts0xc8c8 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwscushiftersts0xc8c8();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0xC8', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0xc8(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_Counter_0xC8', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0xc8(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SCU_ShifterLockStatus', 'enum': {0: 'SCU_SHIFTERLOCKSTATUS_UNLOCKED', 1: 'SCU_SHIFTERLOCKSTATUS_LOCKED', 3: 'SCU_SHIFTERLOCKSTATUS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_scu_shiftersts_0xc8_c8::Scu_shifterlockstatusType scu_shifterlockstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ShifterLockStatus', 'enum': {0: 'SHIFTERLOCKSTATUS_UNLOCKED', 1: 'SHIFTERLOCKSTATUS_LOCKED', 3: 'SHIFTERLOCKSTATUS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_scu_shiftersts_0xc8_c8::ShifterlockstatusType shifterlockstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ShifterPositionFailure', 'enum': {0: 'SHIFTERPOSITIONFAILURE_NOFAULT', 1: 'SHIFTERPOSITIONFAILURE_MODESELECTORSENSORFAIL', 2: 'SHIFTERPOSITIONFAILURE_ROTARYPOSITIONSENSORFAIL', 3: 'SHIFTERPOSITIONFAILURE_CANBUSCOMMUNICATION', 4: 'SHIFTERPOSITIONFAILURE_SOLENOIDFAIL'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_scu_shiftersts_0xc8_c8::ShifterpositionfailureType shifterpositionfailure(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ShifterPosition', 'enum': {0: 'SHIFTERPOSITION_ERROR', 3: 'SHIFTERPOSITION_MANUAL_MODE', 5: 'SHIFTERPOSITION_DRIVE', 6: 'SHIFTERPOSITION_NEUTRAL', 7: 'SHIFTERPOSITION_REVERSE', 8: 'SHIFTERPOSITION_PARK', 9: 'SHIFTERPOSITION_UPSHIFT', 10: 'SHIFTERPOSITION_DOWNSHIFT'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_scu_shiftersts_0xc8_c8::ShifterpositionType shifterposition(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ShifterPositionInv', 'enum': {5: 'SHIFTERPOSITIONINV_DOWNSHIFT', 6: 'SHIFTERPOSITIONINV_UPSHIFT', 7: 'SHIFTERPOSITIONINV_PARK', 8: 'SHIFTERPOSITIONINV_REVERSE', 9: 'SHIFTERPOSITIONINV_NEUTRAL', 10: 'SHIFTERPOSITIONINV_DRIVE', 12: 'SHIFTERPOSITIONINV_MANUAL_MODE', 15: 'SHIFTERPOSITIONINV_ERROR'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_scu_shiftersts_0xc8_c8::ShifterpositioninvType shifterpositioninv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Sys_STS_SCU', 'enum': {0: 'SYS_STS_SCU_INIT', 1: 'SYS_STS_SCU_OK', 2: 'SYS_STS_SCU_WARNING', 3: 'SYS_STS_SCU_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_scu_shiftersts_0xc8_c8::Sys_sts_scuType sys_sts_scu(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_SCU_SHIFTERSTS_0XC8_C8_H_
