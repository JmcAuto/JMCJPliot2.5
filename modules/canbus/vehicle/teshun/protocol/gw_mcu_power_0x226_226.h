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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_MCU_POWER_0X226_226_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_MCU_POWER_0X226_226_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwmcupower0x226226 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwmcupower0x226226();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x226', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x226(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_Counter_0x226', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x226(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_SerialNum_EST', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 47, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
  bool mcu_serialnum_est(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_TM04_TrqMinDyn', 'offset': -1023.0, 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range': '[-1023|1024]', 'bit': 46, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  int mcu_tm04_trqmindyn(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_TM04_TrqMaxDyn', 'offset': -1023.0, 'precision': 1.0, 'len': 11, 'is_signed_var': False, 'physical_range': '[-1023|1024]', 'bit': 4, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  int mcu_tm04_trqmaxdyn(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_MaxTrq_EST', 'offset': 0.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|2000]', 'bit': 27, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  int mcu_maxtrq_est(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_MinTrq_EST', 'offset': -2000.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2000|0]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  int mcu_mintrq_est(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MCU_Sys_STS', 'enum': {0: 'MCU_SYS_STS_INIT', 1: 'MCU_SYS_STS_OK', 2: 'MCU_SYS_STS_WARNING', 3: 'MCU_SYS_STS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mcu_power_0x226_226::Mcu_sys_stsType mcu_sys_sts(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_MCU_POWER_0X226_226_H_
