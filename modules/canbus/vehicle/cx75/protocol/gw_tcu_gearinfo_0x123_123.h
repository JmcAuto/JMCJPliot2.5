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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_TCU_GEARINFO_0X123_123_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_TCU_GEARINFO_0X123_123_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwtcugearinfo0x123123 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwtcugearinfo0x123123();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'TCU_DisplayGear', 'enum': {0: 'TCU_DISPLAYGEAR_CURRENT_GEAR_N', 1: 'TCU_DISPLAYGEAR_GEAR_1', 2: 'TCU_DISPLAYGEAR_GEAR_2', 3: 'TCU_DISPLAYGEAR_GEAR_3', 4: 'TCU_DISPLAYGEAR_GEAR_4', 5: 'TCU_DISPLAYGEAR_GEAR_5', 6: 'TCU_DISPLAYGEAR_GEAR_6', 7: 'TCU_DISPLAYGEAR_GEAR_7', 8: 'TCU_DISPLAYGEAR_GEAR_8', 9: 'TCU_DISPLAYGEAR_CURRENT_GEAR_D', 10: 'TCU_DISPLAYGEAR_CURRENT_GEAR_L', 11: 'TCU_DISPLAYGEAR_CURRENT_GEAR_R', 12: 'TCU_DISPLAYGEAR_RESERVED', 13: 'TCU_DISPLAYGEAR_CURRENT_GEAR_P', 14: 'TCU_DISPLAYGEAR_RESERVED', 15: 'TCU_DISPLAYGEAR_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_displaygearType tcu_displaygear(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_GearSelectorReq', 'enum': {0: 'TCU_GEARSELECTORREQ_P', 1: 'TCU_GEARSELECTORREQ_L_RESERVED', 2: 'TCU_GEARSELECTORREQ_2_RESERVED', 3: 'TCU_GEARSELECTORREQ_3_RESERVED', 5: 'TCU_GEARSELECTORREQ_D', 6: 'TCU_GEARSELECTORREQ_N', 7: 'TCU_GEARSELECTORREQ_R', 8: 'TCU_GEARSELECTORREQ_M', 15: 'TCU_GEARSELECTORREQ_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_gearselectorreqType tcu_gearselectorreq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_TransPmotoparkReqInvalid', 'enum': {0: 'TCU_TRANSPMOTOPARKREQINVALID_VALID', 1: 'TCU_TRANSPMOTOPARKREQINVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_transpmotoparkreqinvalidType tcu_transpmotoparkreqinvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_GearShiftInProgress', 'enum': {0: 'TCU_GEARSHIFTINPROGRESS_NOGEARSHIFTINPROGRESS', 1: 'TCU_GEARSHIFTINPROGRESS_GEARSHIFTINPROGRESS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_gearshiftinprogressType tcu_gearshiftinprogress(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_ShiftLeverPos_SBWM', 'enum': {0: 'TCU_SHIFTLEVERPOS_SBWM_P', 1: 'TCU_SHIFTLEVERPOS_SBWM_D', 2: 'TCU_SHIFTLEVERPOS_SBWM_N', 3: 'TCU_SHIFTLEVERPOS_SBWM_R', 4: 'TCU_SHIFTLEVERPOS_SBWM_M', 5: 'TCU_SHIFTLEVERPOS_SBWM_BLANK', 6: 'TCU_SHIFTLEVERPOS_SBWM_RESERVED', 7: 'TCU_SHIFTLEVERPOS_SBWM_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ' '}
  Gw_tcu_gearinfo_0x123_123::Tcu_shiftleverpos_sbwmType tcu_shiftleverpos_sbwm(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_TargetGearReq', 'enum': {0: 'TCU_TARGETGEARREQ_DRIVETRAINOPEN_GEARN', 1: 'TCU_TARGETGEARREQ_1STGEAR', 2: 'TCU_TARGETGEARREQ_2NDGEAR', 3: 'TCU_TARGETGEARREQ_3RDGEAR', 4: 'TCU_TARGETGEARREQ_4THGEAR', 5: 'TCU_TARGETGEARREQ_5THGEAR', 6: 'TCU_TARGETGEARREQ_6THGEAR', 7: 'TCU_TARGETGEARREQ_7THGEAR', 8: 'TCU_TARGETGEARREQ_8THGEAR', 14: 'TCU_TARGETGEARREQ_REVERSEGEAR', 15: 'TCU_TARGETGEARREQ_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_targetgearreqType tcu_targetgearreq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_available', 'enum': {0: 'TCU_AVAILABLE_NOTAVAILABLE', 1: 'TCU_AVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_availableType tcu_available(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_DriverIntervention', 'enum': {0: 'TCU_DRIVERINTERVENTION_NOINTERVENTION', 1: 'TCU_DRIVERINTERVENTION_DRIVERINTERVENTION'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_driverinterventionType tcu_driverintervention(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_TrsmFaultFlag', 'enum': {0: 'TCU_TRSMFAULTFLAG_NORMAL', 1: 'TCU_TRSMFAULTFLAG_FAILURE_NOLIMP_HOME', 2: 'TCU_TRSMFAULTFLAG_FAIL_LIMP_HOMEACTIVATED', 3: 'TCU_TRSMFAULTFLAG_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_trsmfaultflagType tcu_trsmfaultflag(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_StGearMode', 'enum': {0: 'TCU_STGEARMODE_STANDARD', 1: 'TCU_STGEARMODE_SNOW', 2: 'TCU_STGEARMODE_MANUAL_RESERVED', 3: 'TCU_STGEARMODE_SPORT', 4: 'TCU_STGEARMODE_ECO', 5: 'TCU_STGEARMODE_4L', 7: 'TCU_STGEARMODE_WETMUD', 8: 'TCU_STGEARMODE_SAND', 9: 'TCU_STGEARMODE_GHAT', 13: 'TCU_STGEARMODE_MANUAL', 14: 'TCU_STGEARMODE_ATSFAULTTCU', 15: 'TCU_STGEARMODE_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_stgearmodeType tcu_stgearmode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_Driving_mode_req', 'enum': {0: 'TCU_DRIVING_MODE_REQ_NORMAL', 1: 'TCU_DRIVING_MODE_REQ_SNOW', 2: 'TCU_DRIVING_MODE_REQ_NOUSED', 3: 'TCU_DRIVING_MODE_REQ_SPORT', 4: 'TCU_DRIVING_MODE_REQ_ECO', 5: 'TCU_DRIVING_MODE_REQ_4L', 6: 'TCU_DRIVING_MODE_REQ_ROCK', 7: 'TCU_DRIVING_MODE_REQ_WETMUD', 8: 'TCU_DRIVING_MODE_REQ_SAND', 14: 'TCU_DRIVING_MODE_REQ_SWITCHFAULT', 15: 'TCU_DRIVING_MODE_REQ_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 43, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_driving_mode_reqType tcu_driving_mode_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_EPBLockReq', 'enum': {0: 'TCU_EPBLOCKREQ_NO_REQUEST', 1: 'TCU_EPBLOCKREQ_REQUEST_PARK_BRAKE_ENGAGE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 44, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_epblockreqType tcu_epblockreq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_highresistance', 'enum': {0: 'TCU_HIGHRESISTANCE_NO_HIGHRESISTANCE', 1: 'TCU_HIGHRESISTANCE_HIGHRESISTANCE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_highresistanceType tcu_highresistance(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_DrivingModeChange_Fault_flag', 'enum': {0: 'TCU_DRIVINGMODECHANGE_FAULT_FLAG_NOFAULT', 1: 'TCU_DRIVINGMODECHANGE_FAULT_FLAG_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 46, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_drivingmodechange_fault_flagType tcu_drivingmodechange_fault_flag(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_ShiftLevertoPReqInvalid', 'enum': {0: 'TCU_SHIFTLEVERTOPREQINVALID_VALID', 1: 'TCU_SHIFTLEVERTOPREQINVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_shiftlevertopreqinvalidType tcu_shiftlevertopreqinvalid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_Status_AliveCounter0x123', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int tcu_status_alivecounter0x123(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_transPmotorparkReq', 'enum': {0: 'TCU_TRANSPMOTORPARKREQ_NOREQUEST', 1: 'TCU_TRANSPMOTORPARKREQ_UNPARKREQUEST', 2: 'TCU_TRANSPMOTORPARKREQ_PARKREQUEST', 3: 'TCU_TRANSPMOTORPARKREQ_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_transpmotorparkreqType tcu_transpmotorparkreq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_ShiftLevertoPReq', 'enum': {0: 'TCU_SHIFTLEVERTOPREQ_NOREQUEST', 1: 'TCU_SHIFTLEVERTOPREQ_PARKREQUEST', 2: 'TCU_SHIFTLEVERTOPREQ_RESERVED', 3: 'TCU_SHIFTLEVERTOPREQ_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_shiftlevertopreqType tcu_shiftlevertopreq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_Status_Checksum0x123', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int tcu_status_checksum0x123(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TCU_CurrentGearPosition', 'enum': {0: 'TCU_CURRENTGEARPOSITION_DRIVETRAINOPEN_GEARN', 1: 'TCU_CURRENTGEARPOSITION_1STGEAR', 2: 'TCU_CURRENTGEARPOSITION_2NDGEAR', 3: 'TCU_CURRENTGEARPOSITION_3RDGEAR', 4: 'TCU_CURRENTGEARPOSITION_4THGEAR', 5: 'TCU_CURRENTGEARPOSITION_5THGEAR', 6: 'TCU_CURRENTGEARPOSITION_6THGEAR', 7: 'TCU_CURRENTGEARPOSITION_7THGEAR', 8: 'TCU_CURRENTGEARPOSITION_8THGEAR', 14: 'TCU_CURRENTGEARPOSITION_REVERSEGEAR', 15: 'TCU_CURRENTGEARPOSITION_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tcu_gearinfo_0x123_123::Tcu_currentgearpositionType tcu_currentgearposition(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_TCU_GEARINFO_0X123_123_H_
