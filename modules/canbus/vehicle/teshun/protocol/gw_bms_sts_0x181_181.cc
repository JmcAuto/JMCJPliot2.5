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

#include "modules/canbus/vehicle/teshun/protocol/gw_bms_sts_0x181_181.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwbmssts0x181181::Gwbmssts0x181181() {}
const int32_t Gwbmssts0x181181::ID = 0x181;

void Gwbmssts0x181181::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_checksum_0x181(checksum_0x181(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_rolling_counter_0x181(rolling_counter_0x181(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_err_lev(bms_err_lev(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_idu_status(bms_idu_status(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_batbalance_err(bms_batbalance_err(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_socactual_est(bms_socactual_est(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_packcur_meas(bms_packcur_meas(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_chg_sts(bms_chg_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_packvol_meas(bms_packvol_meas(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_balance_sts(bms_balance_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_packnum_est(bms_packnum_est(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_hvdown_req(bms_hvdown_req(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_hvonoff_sts(bms_hvonoff_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_sts_0x181_181()->set_bms_sys_sts(bms_sys_sts(bytes, length));
}

// config detail: {'name': 'checksum_0x181', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwbmssts0x181181::checksum_0x181(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x181', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwbmssts0x181181::rolling_counter_0x181(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'bms_err_lev', 'enum': {0: 'BMS_ERR_LEV_NO_ERROR', 1: 'BMS_ERR_LEV_LEVEL1_ERROR', 2: 'BMS_ERR_LEV_LEVEL2_ERROR', 3: 'BMS_ERR_LEV_LEVEL3_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_bms_sts_0x181_181::Bms_err_levType Gwbmssts0x181181::bms_err_lev(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 2);

  Gw_bms_sts_0x181_181::Bms_err_levType ret =  static_cast<Gw_bms_sts_0x181_181::Bms_err_levType>(x);
  return ret;
}

// config detail: {'name': 'bms_idu_status', 'enum': {0: 'BMS_IDU_STATUS_NORMAL', 1: 'BMS_IDU_STATUS_LEVEL1_600_A6_B8_V', 2: 'BMS_IDU_STATUS_LEVEL2_500_A6_B8_V', 3: 'BMS_IDU_STATUS_LEVEL3_CHARGE_100_A6_B8_V_DRIVE_400_A6_B8_V'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_bms_sts_0x181_181::Bms_idu_statusType Gwbmssts0x181181::bms_idu_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Gw_bms_sts_0x181_181::Bms_idu_statusType ret =  static_cast<Gw_bms_sts_0x181_181::Bms_idu_statusType>(x);
  return ret;
}

// config detail: {'name': 'bms_batbalance_err', 'enum': {0: 'BMS_BATBALANCE_ERR_NORMAL', 1: 'BMS_BATBALANCE_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_bms_sts_0x181_181::Bms_batbalance_errType Gwbmssts0x181181::bms_batbalance_err(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Gw_bms_sts_0x181_181::Bms_batbalance_errType ret =  static_cast<Gw_bms_sts_0x181_181::Bms_batbalance_errType>(x);
  return ret;
}

// config detail: {'name': 'bms_socactual_est', 'offset': 0.0, 'precision': 0.25, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Gwbmssts0x181181::bms_socactual_est(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  double ret = x * 0.250000;
  return ret;
}

// config detail: {'name': 'bms_packcur_meas', 'offset': -800.0, 'precision': 0.1, 'len': 14, 'is_signed_var': False, 'physical_range': '[-800|800]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'A'}
double Gwbmssts0x181181::bms_packcur_meas(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(2, 6);
  x <<= 6;
  x |= t;

  double ret = x * 0.100000 + -800.000000;
  return ret;
}

// config detail: {'name': 'bms_chg_sts', 'enum': {0: 'BMS_CHG_STS_NOT_READY', 1: 'BMS_CHG_STS_READY_TO_CHARGING', 2: 'BMS_CHG_STS_CHARGING', 3: 'BMS_CHG_STS_CHARGEERROR', 4: 'BMS_CHG_STS_CHARGEOK'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_bms_sts_0x181_181::Bms_chg_stsType Gwbmssts0x181181::bms_chg_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 3);

  Gw_bms_sts_0x181_181::Bms_chg_stsType ret =  static_cast<Gw_bms_sts_0x181_181::Bms_chg_stsType>(x);
  return ret;
}

// config detail: {'name': 'bms_packvol_meas', 'offset': 0.0, 'precision': 0.1, 'len': 13, 'is_signed_var': False, 'physical_range': '[0|800]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'V'}
double Gwbmssts0x181181::bms_packvol_meas(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'bms_balance_sts', 'enum': {0: 'BMS_BALANCE_STS_INACTIVE', 1: 'BMS_BALANCE_STS_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_bms_sts_0x181_181::Bms_balance_stsType Gwbmssts0x181181::bms_balance_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Gw_bms_sts_0x181_181::Bms_balance_stsType ret =  static_cast<Gw_bms_sts_0x181_181::Bms_balance_stsType>(x);
  return ret;
}

// config detail: {'name': 'bms_packnum_est', 'offset': 0.0, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 2, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwbmssts0x181181::bms_packnum_est(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 2);

  int ret = x;
  return ret;
}

// config detail: {'name': 'bms_hvdown_req', 'enum': {0: 'BMS_HVDOWN_REQ_NO_REQUEST', 1: 'BMS_HVDOWN_REQ_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_bms_sts_0x181_181::Bms_hvdown_reqType Gwbmssts0x181181::bms_hvdown_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  Gw_bms_sts_0x181_181::Bms_hvdown_reqType ret =  static_cast<Gw_bms_sts_0x181_181::Bms_hvdown_reqType>(x);
  return ret;
}

// config detail: {'name': 'bms_hvonoff_sts', 'enum': {0: 'BMS_HVONOFF_STS_HV_OFF', 1: 'BMS_HVONOFF_STS_PRECHARGE', 2: 'BMS_HVONOFF_STS_HV_ON', 3: 'BMS_HVONOFF_STS_FAIL_TO_HV_ON'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_bms_sts_0x181_181::Bms_hvonoff_stsType Gwbmssts0x181181::bms_hvonoff_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 2);

  Gw_bms_sts_0x181_181::Bms_hvonoff_stsType ret =  static_cast<Gw_bms_sts_0x181_181::Bms_hvonoff_stsType>(x);
  return ret;
}

// config detail: {'name': 'bms_sys_sts', 'enum': {0: 'BMS_SYS_STS_INIT', 1: 'BMS_SYS_STS_OK', 2: 'BMS_SYS_STS_WARNING', 3: 'BMS_SYS_STS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_bms_sts_0x181_181::Bms_sys_stsType Gwbmssts0x181181::bms_sys_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Gw_bms_sts_0x181_181::Bms_sys_stsType ret =  static_cast<Gw_bms_sts_0x181_181::Bms_sys_stsType>(x);
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
