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

#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_sts_0x218_218.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwvcusts0x218218::Gwvcusts0x218218() {}
const int32_t Gwvcusts0x218218::ID = 0x218;

void Gwvcusts0x218218::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_checksum_0x218(checksum_0x218(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_rolling_counter_0x218(rolling_counter_0x218(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_nlockrequest(vcu_nlockrequest(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_plockrequest(vcu_plockrequest(bytes, length));
 // chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_vehicle_mode(vcu_vehicle_mode(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_engaddfuel_rq(vcu_engaddfuel_rq(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_fan_rq(vcu_fan_rq(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_engstart_rq(vcu_engstart_rq(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_dcu_parkrequest(vcu_dcu_parkrequest(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_actualgearlevelpositionvalid(vcu_actualgearlevelpositionvalid(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_actualgearlevelposition(vcu_actualgearlevelposition(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_bms_chgstart_alw(vcu_bms_chgstart_alw(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_bms_hvonoff_req(vcu_bms_hvonoff_req(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_vehfailgrade_err(vcu_vehfailgrade_err(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_running_mode(vcu_running_mode(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_bp_sleep_allowed(vcu_bp_sleep_allowed(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_hvil_in(vcu_hvil_in(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_hvil_out(vcu_hvil_out(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_bp_input_cooling_temp(vcu_bp_input_cooling_temp(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_sts_0x218_218()->set_vcu_motor_input_cooling_temp(vcu_motor_input_cooling_temp(bytes, length));
}

// config detail: {'name': 'checksum_0x218', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcusts0x218218::checksum_0x218(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x218', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcusts0x218218::rolling_counter_0x218(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_nlockrequest', 'enum': {0: 'VCU_NLOCKREQUEST_NO_USE', 1: 'VCU_NLOCKREQUEST_LOCK', 2: 'VCU_NLOCKREQUEST_UNLOCK', 3: 'VCU_NLOCKREQUEST_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_nlockrequestType Gwvcusts0x218218::vcu_nlockrequest(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 2);

  Gw_vcu_sts_0x218_218::Vcu_nlockrequestType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_nlockrequestType>(x);
  return ret;
}

// config detail: {'name': 'vcu_plockrequest', 'enum': {0: 'VCU_PLOCKREQUEST_NO_USE', 1: 'VCU_PLOCKREQUEST_LOCK', 2: 'VCU_PLOCKREQUEST_UNLOCK', 3: 'VCU_PLOCKREQUEST_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_plockrequestType Gwvcusts0x218218::vcu_plockrequest(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  Gw_vcu_sts_0x218_218::Vcu_plockrequestType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_plockrequestType>(x);
  return ret;
}

// config detail: {'name': 'vcu_vehicle_mode', 'enum': {1: 'VCU_VEHICLE_MODE_STANDBY_IG_OFF_MODE', 2: 'VCU_VEHICLE_MODE_VEHICLE_RESET_MODE', 3: 'VCU_VEHICLE_MODE_HV_ACTIVATION_MODE', 4: 'VCU_VEHICLE_MODE_DRIVING_MODE', 5: 'VCU_VEHICLE_MODE_HV_TERMINATION_MODE', 6: 'VCU_VEHICLE_MODE_CHARGING_MODE', 7: 'VCU_VEHICLE_MODE_RESERVED', 8: 'VCU_VEHICLE_MODE_EMER_DRIVING_MODE'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
// Gw_vcu_sts_0x218_218::Vcu_vehicle_modeType Gwvcusts0x218218::vcu_vehicle_mode(const std::uint8_t* bytes, int32_t length) const {
//   Byte t0(bytes + 4);
//   int32_t x = t0.get_byte(4, 4);

//   Gw_vcu_sts_0x218_218::Vcu_vehicle_modeType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_vehicle_modeType>(x);
//   return ret;
// }

// config detail: {'name': 'vcu_engaddfuel_rq', 'enum': {0: 'VCU_ENGADDFUEL_RQ_NO_RQ', 1: 'VCU_ENGADDFUEL_RQ_RQ'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_engaddfuel_rqType Gwvcusts0x218218::vcu_engaddfuel_rq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 1);

  Gw_vcu_sts_0x218_218::Vcu_engaddfuel_rqType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_engaddfuel_rqType>(x);
  return ret;
}

// config detail: {'name': 'vcu_fan_rq', 'enum': {0: 'VCU_FAN_RQ_NO_RQ', 1: 'VCU_FAN_RQ_FAN_LOW_RQ', 4: 'VCU_FAN_RQ_FAN_HIGH_RQ'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_fan_rqType Gwvcusts0x218218::vcu_fan_rq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 3);

  Gw_vcu_sts_0x218_218::Vcu_fan_rqType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_fan_rqType>(x);
  return ret;
}

// config detail: {'name': 'vcu_engstart_rq', 'enum': {0: 'VCU_ENGSTART_RQ_NO_RQ', 1: 'VCU_ENGSTART_RQ_RQ'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_engstart_rqType Gwvcusts0x218218::vcu_engstart_rq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 1);

  Gw_vcu_sts_0x218_218::Vcu_engstart_rqType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_engstart_rqType>(x);
  return ret;
}

// config detail: {'name': 'vcu_dcu_parkrequest', 'enum': {0: 'VCU_DCU_PARKREQUEST_OFF', 1: 'VCU_DCU_PARKREQUEST_PARK', 2: 'VCU_DCU_PARKREQUEST_UNPARK', 3: 'VCU_DCU_PARKREQUEST_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 44, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_dcu_parkrequestType Gwvcusts0x218218::vcu_dcu_parkrequest(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(3, 2);

  Gw_vcu_sts_0x218_218::Vcu_dcu_parkrequestType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_dcu_parkrequestType>(x);
  return ret;
}

// config detail: {'name': 'vcu_actualgearlevelpositionvalid', 'enum': {0: 'VCU_ACTUALGEARLEVELPOSITIONVALID_VALID', 1: 'VCU_ACTUALGEARLEVELPOSITIONVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_actualgearlevelpositionvalidType Gwvcusts0x218218::vcu_actualgearlevelpositionvalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Gw_vcu_sts_0x218_218::Vcu_actualgearlevelpositionvalidType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_actualgearlevelpositionvalidType>(x);
  return ret;
}

// config detail: {'name': 'vcu_actualgearlevelposition', 'enum': {0: 'VCU_ACTUALGEARLEVELPOSITION_INITIAL', 1: 'VCU_ACTUALGEARLEVELPOSITION_P_PARK', 2: 'VCU_ACTUALGEARLEVELPOSITION_R_REVERSE', 3: 'VCU_ACTUALGEARLEVELPOSITION_N_NEUTRAL', 4: 'VCU_ACTUALGEARLEVELPOSITION_D_DRIVE', 5: 'VCU_ACTUALGEARLEVELPOSITION_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_actualgearlevelpositionType Gwvcusts0x218218::vcu_actualgearlevelposition(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(5, 3);

  Gw_vcu_sts_0x218_218::Vcu_actualgearlevelpositionType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_actualgearlevelpositionType>(x);
  return ret;
}

// config detail: {'name': 'vcu_bms_chgstart_alw', 'enum': {0: 'VCU_BMS_CHGSTART_ALW_FORBID', 1: 'VCU_BMS_CHGSTART_ALW_ALLOW'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_bms_chgstart_alwType Gwvcusts0x218218::vcu_bms_chgstart_alw(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 1);

  Gw_vcu_sts_0x218_218::Vcu_bms_chgstart_alwType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_bms_chgstart_alwType>(x);
  return ret;
}

// config detail: {'name': 'vcu_bms_hvonoff_req', 'enum': {0: 'VCU_BMS_HVONOFF_REQ_FORBID', 1: 'VCU_BMS_HVONOFF_REQ_ALLOW'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_bms_hvonoff_reqType Gwvcusts0x218218::vcu_bms_hvonoff_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  Gw_vcu_sts_0x218_218::Vcu_bms_hvonoff_reqType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_bms_hvonoff_reqType>(x);
  return ret;
}

// config detail: {'name': 'vcu_vehfailgrade_err', 'enum': {0: 'VCU_VEHFAILGRADE_ERR_NORMAL', 1: 'VCU_VEHFAILGRADE_ERR_LEVEL1', 2: 'VCU_VEHFAILGRADE_ERR_LEVEL2', 3: 'VCU_VEHFAILGRADE_ERR_LEVEL3'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_vehfailgrade_errType Gwvcusts0x218218::vcu_vehfailgrade_err(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 2);

  Gw_vcu_sts_0x218_218::Vcu_vehfailgrade_errType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_vehfailgrade_errType>(x);
  return ret;
}

// config detail: {'name': 'vcu_running_mode', 'enum': {0: 'VCU_RUNNING_MODE_STANDBY', 1: 'VCU_RUNNING_MODE_EV_MODE', 2: 'VCU_RUNNING_MODE_HYBIRD_MODE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_running_modeType Gwvcusts0x218218::vcu_running_mode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 2);

  Gw_vcu_sts_0x218_218::Vcu_running_modeType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_running_modeType>(x);
  return ret;
}

// config detail: {'name': 'vcu_bp_sleep_allowed', 'enum': {0: 'VCU_BP_SLEEP_ALLOWED_NOT_ALLOWED', 1: 'VCU_BP_SLEEP_ALLOWED_ALLOWED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_bp_sleep_allowedType Gwvcusts0x218218::vcu_bp_sleep_allowed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  Gw_vcu_sts_0x218_218::Vcu_bp_sleep_allowedType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_bp_sleep_allowedType>(x);
  return ret;
}

// config detail: {'name': 'vcu_hvil_in', 'enum': {0: 'VCU_HVIL_IN_OFF', 1: 'VCU_HVIL_IN_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_hvil_inType Gwvcusts0x218218::vcu_hvil_in(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  Gw_vcu_sts_0x218_218::Vcu_hvil_inType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_hvil_inType>(x);
  return ret;
}

// config detail: {'name': 'vcu_hvil_out', 'enum': {0: 'VCU_HVIL_OUT_OFF', 1: 'VCU_HVIL_OUT_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_sts_0x218_218::Vcu_hvil_outType Gwvcusts0x218218::vcu_hvil_out(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 1);

  Gw_vcu_sts_0x218_218::Vcu_hvil_outType ret =  static_cast<Gw_vcu_sts_0x218_218::Vcu_hvil_outType>(x);
  return ret;
}

// config detail: {'name': 'vcu_bp_input_cooling_temp', 'offset': -40.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-40|215]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': '\\A1\\E6'}
int Gwvcusts0x218218::vcu_bp_input_cooling_temp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'name': 'vcu_motor_input_cooling_temp', 'offset': -40.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-40|215]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': '\\A1\\E6'}
int Gwvcusts0x218218::vcu_motor_input_cooling_temp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
