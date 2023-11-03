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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_BMS_STS_0X181_181_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_BMS_STS_0X181_181_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwbmssts0x181181 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwbmssts0x181181();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x181', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x181(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_Counter_0x181', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x181(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_ERR_LEV', 'enum': {0: 'BMS_ERR_LEV_NO_ERROR', 1: 'BMS_ERR_LEV_LEVEL1_ERROR', 2: 'BMS_ERR_LEV_LEVEL2_ERROR', 3: 'BMS_ERR_LEV_LEVEL3_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_bms_sts_0x181_181::Bms_err_levType bms_err_lev(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_IDU_Status', 'enum': {0: 'BMS_IDU_STATUS_NORMAL', 1: 'BMS_IDU_STATUS_LEVEL1_600_A6_B8_V', 2: 'BMS_IDU_STATUS_LEVEL2_500_A6_B8_V', 3: 'BMS_IDU_STATUS_LEVEL3_CHARGE_100_A6_B8_V_DRIVE_400_A6_B8_V'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_bms_sts_0x181_181::Bms_idu_statusType bms_idu_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_BatBalance_ERR', 'enum': {0: 'BMS_BATBALANCE_ERR_NORMAL', 1: 'BMS_BATBALANCE_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_bms_sts_0x181_181::Bms_batbalance_errType bms_batbalance_err(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_SocActual_EST', 'offset': 0.0, 'precision': 0.25, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  double bms_socactual_est(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_PackCur_MEAS', 'offset': -800.0, 'precision': 0.1, 'len': 14, 'is_signed_var': False, 'physical_range': '[-800|800]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'A'}
  double bms_packcur_meas(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_Chg_STS', 'enum': {0: 'BMS_CHG_STS_NOT_READY', 1: 'BMS_CHG_STS_READY_TO_CHARGING', 2: 'BMS_CHG_STS_CHARGING', 3: 'BMS_CHG_STS_CHARGEERROR', 4: 'BMS_CHG_STS_CHARGEOK'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_bms_sts_0x181_181::Bms_chg_stsType bms_chg_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_PackVol_MEAS', 'offset': 0.0, 'precision': 0.1, 'len': 13, 'is_signed_var': False, 'physical_range': '[0|800]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'V'}
  double bms_packvol_meas(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_Balance_STS', 'enum': {0: 'BMS_BALANCE_STS_INACTIVE', 1: 'BMS_BALANCE_STS_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_bms_sts_0x181_181::Bms_balance_stsType bms_balance_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_PackNum_EST', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int bms_packnum_est(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_HvDown_REQ', 'enum': {0: 'BMS_HVDOWN_REQ_NO_REQUEST', 1: 'BMS_HVDOWN_REQ_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_bms_sts_0x181_181::Bms_hvdown_reqType bms_hvdown_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_HVOnOff_STS', 'enum': {0: 'BMS_HVONOFF_STS_HV_OFF', 1: 'BMS_HVONOFF_STS_PRECHARGE', 2: 'BMS_HVONOFF_STS_HV_ON', 3: 'BMS_HVONOFF_STS_FAIL_TO_HV_ON'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_bms_sts_0x181_181::Bms_hvonoff_stsType bms_hvonoff_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BMS_Sys_STS', 'enum': {0: 'BMS_SYS_STS_INIT', 1: 'BMS_SYS_STS_OK', 2: 'BMS_SYS_STS_WARNING', 3: 'BMS_SYS_STS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_bms_sts_0x181_181::Bms_sys_stsType bms_sys_sts(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_BMS_STS_0X181_181_H_
