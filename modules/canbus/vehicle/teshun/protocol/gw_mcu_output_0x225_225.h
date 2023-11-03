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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_MCU_OUTPUT_0X225_225_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_MCU_OUTPUT_0X225_225_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwmcuoutput0x225225 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwmcuoutput0x225225();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x225', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x225(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_Counter_0x225', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x225(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_Work_STS', 'enum': {0: 'MCU_WORK_STS_CONSUM', 1: 'MCU_WORK_STS_GENERATE', 2: 'MCU_WORK_STS_OFF', 3: 'MCU_WORK_STS_READY', 4: 'MCU_WORK_STS_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mcu_output_0x225_225::Mcu_work_stsType mcu_work_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_CtrMode_STS', 'enum': {0: 'MCU_CTRMODE_STS_INITIALIZATION', 1: 'MCU_CTRMODE_STS_PRECHARGE', 2: 'MCU_CTRMODE_STS_DISABLE', 3: 'MCU_CTRMODE_STS_STANDBY', 4: 'MCU_CTRMODE_STS_ANTISLIP', 5: 'MCU_CTRMODE_STS_ALOFFSETCAL', 7: 'MCU_CTRMODE_STS_NCTLINT', 8: 'MCU_CTRMODE_STS_TRQCT', 9: 'MCU_CTRMODE_STS_ASCACTIVE', 11: 'MCU_CTRMODE_STS_AFTERRUN', 12: 'MCU_CTRMODE_STS_PREFAILURE', 13: 'MCU_CTRMODE_STS_FAILURE', 14: 'MCU_CTRMODE_STS_DISCHARGE'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mcu_output_0x225_225::Mcu_ctrmode_stsType mcu_ctrmode_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_Capacitor_STS', 'enum': {0: 'MCU_CAPACITOR_STS_FORBIDCHARGE', 1: 'MCU_CAPACITOR_STS_WAITCHARGE', 2: 'MCU_CAPACITOR_STS_ALLOWDISCHARGE', 3: 'MCU_CAPACITOR_STS_FORBIDDISCHARGE', 4: 'MCU_CAPACITOR_STS_ERRORDISCHARGE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mcu_output_0x225_225::Mcu_capacitor_stsType mcu_capacitor_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_Spd_MEAS', 'offset': -15000.0, 'precision': 1.0, 'len': 15, 'is_signed_var': False, 'physical_range': '[-15000|15000]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': 'rpm'}
  int mcu_spd_meas(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_Trq_MEAS', 'offset': -2000.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2000|2000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  int mcu_trq_meas(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_MCU_OUTPUT_0X225_225_H_
