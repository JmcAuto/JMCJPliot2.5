/******************************************************************************
 * Copyright 2017 The jmc_auto Authors. All Rights Reserved.
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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_EPS_0X260_260_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_EPS_0X260_260_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Eps0x260260 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Eps0x260260();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'EPS_SASSoftLimitRightFlg', 'enum': {0: 'EPS_SASSOFTLIMITRIGHTFLG_NO_LEARNED', 1: 'EPS_SASSOFTLIMITRIGHTFLG_PRIMARY_LEARNED_ONLY_FOR_CEPS', 2: 'EPS_SASSOFTLIMITRIGHTFLG_LEARNED', 3: 'EPS_SASSOFTLIMITRIGHTFLG_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_sassoftlimitrightflgType eps_sassoftlimitrightflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_N_LoadInfo', 'offset': 0.0, 'precision': 1.0, 'len': 7, 'is_signed_var': False, 'physical_range': '[0|127]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': 'A'}
  int eps_n_loadinfo(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_St_EmergencyPowerLimit', 'enum': {0: 'EPS_ST_EMERGENCYPOWERLIMIT_NORMAL', 1: 'EPS_ST_EMERGENCYPOWERLIMIT_POWER_DENSITY_LIMIT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_st_emergencypowerlimitType eps_st_emergencypowerlimit(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_St_EmergencyMotor', 'enum': {0: 'EPS_ST_EMERGENCYMOTOR_NORMAL', 1: 'EPS_ST_EMERGENCYMOTOR_SERVO_MOTOR_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_st_emergencymotorType eps_st_emergencymotor(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_St_EmergencyECU', 'enum': {0: 'EPS_ST_EMERGENCYECU_NORMAL', 1: 'EPS_ST_EMERGENCYECU_ECU_INNER_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_st_emergencyecuType eps_st_emergencyecu(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_St_EmergencyCANInterface', 'enum': {0: 'EPS_ST_EMERGENCYCANINTERFACE_NORMAL', 1: 'EPS_ST_EMERGENCYCANINTERFACE_CAN_INTERFACE_IS_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_st_emergencycaninterfaceType eps_st_emergencycaninterface(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_St_EmergencySensor', 'enum': {0: 'EPS_ST_EMERGENCYSENSOR_NORMAL', 1: 'EPS_ST_EMERGENCYSENSOR_SENSOR_IS_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_st_emergencysensorType eps_st_emergencysensor(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_St_EmergencyOverTemp', 'enum': {0: 'EPS_ST_EMERGENCYOVERTEMP_NORMAL', 1: 'EPS_ST_EMERGENCYOVERTEMP_OVER_TEMPERATURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_st_emergencyovertempType eps_st_emergencyovertemp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_St_EmergencyOverVoltage', 'enum': {0: 'EPS_ST_EMERGENCYOVERVOLTAGE_NORMAL', 1: 'EPS_ST_EMERGENCYOVERVOLTAGE_OVER_VOLTAGE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_st_emergencyovervoltageType eps_st_emergencyovervoltage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_St_EmergencyUnderVoltage', 'enum': {0: 'EPS_ST_EMERGENCYUNDERVOLTAGE_NORMAL', 1: 'EPS_ST_EMERGENCYUNDERVOLTAGE_UNDER_VOLTAGE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_st_emergencyundervoltageType eps_st_emergencyundervoltage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_F_ECUTempValid', 'enum': {0: 'EPS_F_ECUTEMPVALID_INVALID', 1: 'EPS_F_ECUTEMPVALID_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_f_ecutempvalidType eps_f_ecutempvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_SASSoftLimitLeftFlg', 'enum': {0: 'EPS_SASSOFTLIMITLEFTFLG_NO_LEARNED', 1: 'EPS_SASSOFTLIMITLEFTFLG_PRIMARY_LEARNED_ONLY_FOR_CEPS', 2: 'EPS_SASSOFTLIMITLEFTFLG_LEARNED', 3: 'EPS_SASSOFTLIMITLEFTFLG_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_sassoftlimitleftflgType eps_sassoftlimitleftflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_N_ECUTemp', 'offset': -70.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-70|185]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Degree'}
  int eps_n_ecutemp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_N_PerformanceRedu', 'offset': 0.0, 'precision': 0.5, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  double eps_n_performanceredu(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_St_DTCFlag', 'enum': {0: 'EPS_ST_DTCFLAG_NO_DTC_EXIST', 1: 'EPS_ST_DTCFLAG_DTC_EXIST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_st_dtcflagType eps_st_dtcflag(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x260', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x260(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_SASIndexSts', 'enum': {0: 'EPS_SASINDEXSTS_STEERWHEEL_NOT_AT_MIDDLE_POSITION', 1: 'EPS_SASINDEXSTS_STEERWHEEL_AT_MIDDLE_POSITION'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_sasindexstsType eps_sasindexsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_S_WarningLampYellow', 'enum': {0: 'EPS_S_WARNINGLAMPYELLOW_CLOSE', 1: 'EPS_S_WARNINGLAMPYELLOW_OPEN'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_s_warninglampyellowType eps_s_warninglampyellow(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x260', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x260(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_S_SafeLampRed', 'enum': {0: 'EPS_S_SAFELAMPRED_CLOSE', 1: 'EPS_S_SAFELAMPRED_OPEN'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_s_safelampredType eps_s_safelampred(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EPS_F_LoadInfo', 'enum': {0: 'EPS_F_LOADINFO_NO_FAULT', 1: 'EPS_F_LOADINFO_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Eps_0x260_260::Eps_f_loadinfoType eps_f_loadinfo(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_EPS_0X260_260_H_
