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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_STS_0X218_218_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_STS_0X218_218_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwvcusts0x218218 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwvcusts0x218218();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x218', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x218(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_Counter_0x218', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x218(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_NLockRequest', 'enum': {0: 'VCU_NLOCKREQUEST_NO_USE', 1: 'VCU_NLOCKREQUEST_LOCK', 2: 'VCU_NLOCKREQUEST_UNLOCK', 3: 'VCU_NLOCKREQUEST_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_nlockrequestType vcu_nlockrequest(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_PLockRequest', 'enum': {0: 'VCU_PLOCKREQUEST_NO_USE', 1: 'VCU_PLOCKREQUEST_LOCK', 2: 'VCU_PLOCKREQUEST_UNLOCK', 3: 'VCU_PLOCKREQUEST_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_plockrequestType vcu_plockrequest(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_Vehicle_Mode', 'enum': {1: 'VCU_VEHICLE_MODE_STANDBY_IG_OFF_MODE', 2: 'VCU_VEHICLE_MODE_VEHICLE_RESET_MODE', 3: 'VCU_VEHICLE_MODE_HV_ACTIVATION_MODE', 4: 'VCU_VEHICLE_MODE_DRIVING_MODE', 5: 'VCU_VEHICLE_MODE_HV_TERMINATION_MODE', 6: 'VCU_VEHICLE_MODE_CHARGING_MODE', 7: 'VCU_VEHICLE_MODE_RESERVED', 8: 'VCU_VEHICLE_MODE_EMER_DRIVING_MODE'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  //Gw_vcu_sts_0x218_218::Vcu_vehicle_modeType vcu_vehicle_mode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_EngaddFuel_RQ', 'enum': {0: 'VCU_ENGADDFUEL_RQ_NO_RQ', 1: 'VCU_ENGADDFUEL_RQ_RQ'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_engaddfuel_rqType vcu_engaddfuel_rq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_FAN_RQ', 'enum': {0: 'VCU_FAN_RQ_NO_RQ', 1: 'VCU_FAN_RQ_FAN_LOW_RQ', 4: 'VCU_FAN_RQ_FAN_HIGH_RQ'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_fan_rqType vcu_fan_rq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_EngStart_RQ', 'enum': {0: 'VCU_ENGSTART_RQ_NO_RQ', 1: 'VCU_ENGSTART_RQ_RQ'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_engstart_rqType vcu_engstart_rq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_DCU_ParkRequest', 'enum': {0: 'VCU_DCU_PARKREQUEST_OFF', 1: 'VCU_DCU_PARKREQUEST_PARK', 2: 'VCU_DCU_PARKREQUEST_UNPARK', 3: 'VCU_DCU_PARKREQUEST_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 44, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_dcu_parkrequestType vcu_dcu_parkrequest(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ActualGearLevelPositionValid', 'enum': {0: 'VCU_ACTUALGEARLEVELPOSITIONVALID_VALID', 1: 'VCU_ACTUALGEARLEVELPOSITIONVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_actualgearlevelpositionvalidType vcu_actualgearlevelpositionvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ActualGearLevelPosition', 'enum': {0: 'VCU_ACTUALGEARLEVELPOSITION_INITIAL', 1: 'VCU_ACTUALGEARLEVELPOSITION_P_PARK', 2: 'VCU_ACTUALGEARLEVELPOSITION_R_REVERSE', 3: 'VCU_ACTUALGEARLEVELPOSITION_N_NEUTRAL', 4: 'VCU_ACTUALGEARLEVELPOSITION_D_DRIVE', 5: 'VCU_ACTUALGEARLEVELPOSITION_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_actualgearlevelpositionType vcu_actualgearlevelposition(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BMS_ChgStart_ALW', 'enum': {0: 'VCU_BMS_CHGSTART_ALW_FORBID', 1: 'VCU_BMS_CHGSTART_ALW_ALLOW'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_bms_chgstart_alwType vcu_bms_chgstart_alw(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BMS_HVOnOff_REQ', 'enum': {0: 'VCU_BMS_HVONOFF_REQ_FORBID', 1: 'VCU_BMS_HVONOFF_REQ_ALLOW'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_bms_hvonoff_reqType vcu_bms_hvonoff_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VehFailGrade_ERR', 'enum': {0: 'VCU_VEHFAILGRADE_ERR_NORMAL', 1: 'VCU_VEHFAILGRADE_ERR_LEVEL1', 2: 'VCU_VEHFAILGRADE_ERR_LEVEL2', 3: 'VCU_VEHFAILGRADE_ERR_LEVEL3'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_vehfailgrade_errType vcu_vehfailgrade_err(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_Running_Mode', 'enum': {0: 'VCU_RUNNING_MODE_STANDBY', 1: 'VCU_RUNNING_MODE_EV_MODE', 2: 'VCU_RUNNING_MODE_HYBIRD_MODE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_running_modeType vcu_running_mode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BP_Sleep_Allowed', 'enum': {0: 'VCU_BP_SLEEP_ALLOWED_NOT_ALLOWED', 1: 'VCU_BP_SLEEP_ALLOWED_ALLOWED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_bp_sleep_allowedType vcu_bp_sleep_allowed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_HVIL_In', 'enum': {0: 'VCU_HVIL_IN_OFF', 1: 'VCU_HVIL_IN_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_hvil_inType vcu_hvil_in(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_HVIL_OUT', 'enum': {0: 'VCU_HVIL_OUT_OFF', 1: 'VCU_HVIL_OUT_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_sts_0x218_218::Vcu_hvil_outType vcu_hvil_out(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BP_Input_Cooling_Temp', 'offset': -40.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-40|215]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': '\\A1\\E6'}
  int vcu_bp_input_cooling_temp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_Motor_Input_Cooling_Temp', 'offset': -40.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-40|215]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': '\\A1\\E6'}
  int vcu_motor_input_cooling_temp(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_VCU_STS_0X218_218_H_
