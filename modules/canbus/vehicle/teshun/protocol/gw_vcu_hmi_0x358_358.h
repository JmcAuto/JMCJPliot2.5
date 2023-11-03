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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_HMI_0X358_358_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_HMI_0X358_358_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwvcuhmi0x358358 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwvcuhmi0x358358();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'VCU_Checksum_0x358', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int vcu_checksum_0x358(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_RollingCounter_0x358', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int vcu_rollingcounter_0x358(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VehicleHVStatus', 'enum': {0: 'VEHICLEHVSTATUS_NOT_READY', 1: 'VEHICLEHVSTATUS_HV_ON', 2: 'VEHICLEHVSTATUS_READYTODRIVE', 3: 'VEHICLEHVSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::VehiclehvstatusType vehiclehvstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'PackHeatCircuitError', 'enum': {0: 'PACKHEATCIRCUITERROR_NORMAL', 1: 'PACKHEATCIRCUITERROR_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 44, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::PackheatcircuiterrorType packheatcircuiterror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'PackHeatStatus', 'enum': {0: 'PACKHEATSTATUS_INACTIVE', 1: 'PACKHEATSTATUS_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::PackheatstatusType packheatstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'PackCoolingCircuitError', 'enum': {0: 'PACKCOOLINGCIRCUITERROR_NORMAL', 1: 'PACKCOOLINGCIRCUITERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 10, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::PackcoolingcircuiterrorType packcoolingcircuiterror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MotorCoolingCircuitError', 'enum': {0: 'MOTORCOOLINGCIRCUITERROR_NORMAL', 1: 'MOTORCOOLINGCIRCUITERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::MotorcoolingcircuiterrorType motorcoolingcircuiterror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BatteryVoltageError', 'enum': {0: 'VCU_BATTERYVOLTAGEERROR_NO_WARNING', 1: 'VCU_BATTERYVOLTAGEERROR_WARNING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_batteryvoltageerrorType vcu_batteryvoltageerror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BatterVoltage', 'offset': 0.0, 'precision': 0.07, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|17.78]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'V'}
  double vcu_battervoltage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_TargCruiseSpeed', 'offset': 0.0, 'precision': 0.5, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
  double vcu_targcruisespeed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_CruiseControlStatus', 'enum': {0: 'VCU_CRUISECONTROLSTATUS_CRUISECONTROLOOFF', 1: 'VCU_CRUISECONTROLSTATUS_ACTIVE', 2: 'VCU_CRUISECONTROLSTATUS_STANDBY', 3: 'VCU_CRUISECONTROLSTATUS_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 46, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_cruisecontrolstatusType vcu_cruisecontrolstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_DriverStatus_STS', 'enum': {0: 'VCU_DRIVERSTATUS_STS_NO_DRIVE', 1: 'VCU_DRIVERSTATUS_STS_DRIVE_MODE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_driverstatus_stsType vcu_driverstatus_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_DriveMode_STS', 'enum': {0: 'VCU_DRIVEMODE_STS_INVALID', 1: 'VCU_DRIVEMODE_STS_EV', 2: 'VCU_DRIVEMODE_STS_HEV', 3: 'VCU_DRIVEMODE_STS_FUEL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_drivemode_stsType vcu_drivemode_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_VehChg_STS', 'enum': {0: 'VCU_VEHCHG_STS_INVALID', 1: 'VCU_VEHCHG_STS_STOPCHARGE', 2: 'VCU_VEHCHG_STS_DRIVECHARGE', 3: 'VCU_VEHCHG_STS_NOCHARGE', 4: 'VCU_VEHCHG_STS_CHARGECOMPLETED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_vehchg_stsType vcu_vehchg_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_TBOX_Veh_STS', 'enum': {0: 'VCU_TBOX_VEH_STS_INVALID', 1: 'VCU_TBOX_VEH_STS_START', 2: 'VCU_TBOX_VEH_STS_OFF', 3: 'VCU_TBOX_VEH_STS_OTHER'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_tbox_veh_stsType vcu_tbox_veh_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ICM_EnergyRecoveryMode', 'enum': {0: 'VCU_ICM_ENERGYRECOVERYMODE_NO', 1: 'VCU_ICM_ENERGYRECOVERYMODE_LEVEL1', 2: 'VCU_ICM_ENERGYRECOVERYMODE_LEVEL2', 3: 'VCU_ICM_ENERGYRECOVERYMODE_LEVEL3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_icm_energyrecoverymodeType vcu_icm_energyrecoverymode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ICM_VehHVIL_ERR', 'enum': {0: 'VCU_ICM_VEHHVIL_ERR_NORMAL', 1: 'VCU_ICM_VEHHVIL_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_icm_vehhvil_errType vcu_icm_vehhvil_err(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ICM_Mot_ERR', 'enum': {0: 'VCU_ICM_MOT_ERR_NORMAL', 1: 'VCU_ICM_MOT_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_icm_mot_errType vcu_icm_mot_err(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ICM_PackVoltOff_ERR', 'enum': {0: 'VCU_ICM_PACKVOLTOFF_ERR_NORMAL', 1: 'VCU_ICM_PACKVOLTOFF_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_icm_packvoltoff_errType vcu_icm_packvoltoff_err(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ICM_ChgGearLv', 'enum': {0: 'VCU_ICM_CHGGEARLV_NO_WARNING', 1: 'VCU_ICM_CHGGEARLV_PLEASE_SET_GEARPOSITION_TO_PARK'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_icm_chggearlvType vcu_icm_chggearlv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ICM_PackSys_ERR', 'enum': {0: 'VCU_ICM_PACKSYS_ERR_NORMAL', 1: 'VCU_ICM_PACKSYS_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_icm_packsys_errType vcu_icm_packsys_err(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ICM_DrvSys_ERR', 'enum': {0: 'VCU_ICM_DRVSYS_ERR_NORMAL', 1: 'VCU_ICM_DRVSYS_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_icm_drvsys_errType vcu_icm_drvsys_err(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ICM_Veh_ERR', 'enum': {0: 'VCU_ICM_VEH_ERR_NORMAL', 1: 'VCU_ICM_VEH_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_hmi_0x358_358::Vcu_icm_veh_errType vcu_icm_veh_err(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_VCU_HMI_0X358_358_H_
