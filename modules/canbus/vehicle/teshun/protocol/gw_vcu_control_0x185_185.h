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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_CONTROL_0X185_185_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_CONTROL_0X185_185_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwvcucontrol0x185185 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwvcucontrol0x185185();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x185', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x185(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_Counter_0x185', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x185(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_EMS_EngOperationStatus', 'enum': {0: 'VCU_EMS_ENGOPERATIONSTATUS_STOPPED', 1: 'VCU_EMS_ENGOPERATIONSTATUS_RUNNING', 2: 'VCU_EMS_ENGOPERATIONSTATUS_IDLE', 3: 'VCU_EMS_ENGOPERATIONSTATUS_DFCO', 4: 'VCU_EMS_ENGOPERATIONSTATUS_CRANKING', 5: 'VCU_EMS_ENGOPERATIONSTATUS_STALLING'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_control_0x185_185::Vcu_ems_engoperationstatusType vcu_ems_engoperationstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_MaxChgPwr_ALW', 'offset': 0.0, 'precision': 0.1, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|400]', 'bit': 11, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kw'}
  double vcu_maxchgpwr_alw(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_MaxDischgPwr_ALW', 'offset': 0.0, 'precision': 0.1, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|400]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kw'}
  double vcu_maxdischgpwr_alw(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_MCU_ASD_REQ', 'enum': {0: 'VCU_MCU_ASD_REQ_ON', 1: 'VCU_MCU_ASD_REQ_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 36, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_control_0x185_185::Vcu_mcu_asd_reqType vcu_mcu_asd_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_MCU_Motor1_CtrMode_REQ', 'enum': {0: 'VCU_MCU_MOTOR1_CTRMODE_REQ_DISABLE', 1: 'VCU_MCU_MOTOR1_CTRMODE_REQ_STANDBY', 2: 'VCU_MCU_MOTOR1_CTRMODE_REQ_SPEEDMODE', 3: 'VCU_MCU_MOTOR1_CTRMODE_REQ_TORQUEMODE', 4: 'VCU_MCU_MOTOR1_CTRMODE_REQ_ANTISLIP', 5: 'VCU_MCU_MOTOR1_CTRMODE_REQ_ALOFFSETCAL'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_control_0x185_185::Vcu_mcu_motor1_ctrmode_reqType vcu_mcu_motor1_ctrmode_req(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_VCU_CONTROL_0X185_185_H_
