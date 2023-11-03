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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_CONTROL2_0X131_131_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_CONTROL2_0X131_131_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwvcucontrol20x131131 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwvcucontrol20x131131();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x131', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x131(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_Counter_0x131', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x131(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_STS_VCU', 'enum': {0: 'VCU_STS_VCU_INITIALIZING', 1: 'VCU_STS_VCU_READY', 2: 'VCU_STS_VCU_WARNING', 3: 'VCU_STS_VCU_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_control2_0x131_131::Vcu_sts_vcuType vcu_sts_vcu(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_Brake_flag', 'enum': {0: 'VCU_BRAKE_FLAG_NO_ACTIVE', 1: 'VCU_BRAKE_FLAG_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_control2_0x131_131::Vcu_brake_flagType vcu_brake_flag(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_Tractor_flag', 'enum': {0: 'VCU_TRACTOR_FLAG_NO_ACTIVE', 1: 'VCU_TRACTOR_FLAG_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_control2_0x131_131::Vcu_tractor_flagType vcu_tractor_flag(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BrkPedPos_MEAS_VCU', 'offset': 0.0, 'precision': 0.1, 'len': 10, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 35, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  double vcu_brkpedpos_meas_vcu(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_PWT_MODE_DASH', 'enum': {0: 'VCU_PWT_MODE_DASH_ECOHEV', 1: 'VCU_PWT_MODE_DASH_ECO_EV', 2: 'VCU_PWT_MODE_DASH_PWRHEV', 3: 'VCU_PWT_MODE_DASH_PWREV', 4: 'VCU_PWT_MODE_DASH_EV_ECO', 5: 'VCU_PWT_MODE_DASH_EV_POWER', 7: 'VCU_PWT_MODE_DASH_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_control2_0x131_131::Vcu_pwt_mode_dashType vcu_pwt_mode_dash(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_MCU_Motor1_CapDischarge_REQ', 'enum': {0: 'VCU_MCU_MOTOR1_CAPDISCHARGE_REQ_NO_REQUEST', 1: 'VCU_MCU_MOTOR1_CAPDISCHARGE_REQ_DISCHARGE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 36, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_control2_0x131_131::Vcu_mcu_motor1_capdischarge_reqType vcu_mcu_motor1_capdischarge_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_MCU_Motor1_Trq_REQ', 'offset': -1000.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|1000]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
  double vcu_mcu_motor1_trq_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_MCU_Motor1_Spd_REQ', 'offset': -10000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-10000|10000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'rpm'}
  int vcu_mcu_motor1_spd_req(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_VCU_CONTROL2_0X131_131_H_
