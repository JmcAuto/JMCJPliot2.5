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

#include "modules/canbus/vehicle/teshun/protocol/gw_vcu_hmi_0x358_358.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwvcuhmi0x358358::Gwvcuhmi0x358358() {}
const int32_t Gwvcuhmi0x358358::ID = 0x358;

void Gwvcuhmi0x358358::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_checksum_0x358(vcu_checksum_0x358(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_rollingcounter_0x358(vcu_rollingcounter_0x358(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vehiclehvstatus(vehiclehvstatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_packheatcircuiterror(packheatcircuiterror(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_packheatstatus(packheatstatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_packcoolingcircuiterror(packcoolingcircuiterror(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_motorcoolingcircuiterror(motorcoolingcircuiterror(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_batteryvoltageerror(vcu_batteryvoltageerror(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_battervoltage(vcu_battervoltage(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_targcruisespeed(vcu_targcruisespeed(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_cruisecontrolstatus(vcu_cruisecontrolstatus(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_driverstatus_sts(vcu_driverstatus_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_drivemode_sts(vcu_drivemode_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_vehchg_sts(vcu_vehchg_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_tbox_veh_sts(vcu_tbox_veh_sts(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_icm_energyrecoverymode(vcu_icm_energyrecoverymode(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_icm_vehhvil_err(vcu_icm_vehhvil_err(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_icm_mot_err(vcu_icm_mot_err(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_icm_packvoltoff_err(vcu_icm_packvoltoff_err(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_icm_chggearlv(vcu_icm_chggearlv(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_icm_packsys_err(vcu_icm_packsys_err(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_icm_drvsys_err(vcu_icm_drvsys_err(bytes, length));
  chassis->mutable_teshun()->mutable_gw_vcu_hmi_0x358_358()->set_vcu_icm_veh_err(vcu_icm_veh_err(bytes, length));
}

// config detail: {'name': 'vcu_checksum_0x358', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcuhmi0x358358::vcu_checksum_0x358(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vcu_rollingcounter_0x358', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwvcuhmi0x358358::vcu_rollingcounter_0x358(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'vehiclehvstatus', 'enum': {0: 'VEHICLEHVSTATUS_NOT_READY', 1: 'VEHICLEHVSTATUS_HV_ON', 2: 'VEHICLEHVSTATUS_READYTODRIVE', 3: 'VEHICLEHVSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::VehiclehvstatusType Gwvcuhmi0x358358::vehiclehvstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 2);

  Gw_vcu_hmi_0x358_358::VehiclehvstatusType ret =  static_cast<Gw_vcu_hmi_0x358_358::VehiclehvstatusType>(x);
  return ret;
}

// config detail: {'name': 'packheatcircuiterror', 'enum': {0: 'PACKHEATCIRCUITERROR_NORMAL', 1: 'PACKHEATCIRCUITERROR_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 44, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::PackheatcircuiterrorType Gwvcuhmi0x358358::packheatcircuiterror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(3, 2);

  Gw_vcu_hmi_0x358_358::PackheatcircuiterrorType ret =  static_cast<Gw_vcu_hmi_0x358_358::PackheatcircuiterrorType>(x);
  return ret;
}

// config detail: {'name': 'packheatstatus', 'enum': {0: 'PACKHEATSTATUS_INACTIVE', 1: 'PACKHEATSTATUS_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::PackheatstatusType Gwvcuhmi0x358358::packheatstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  Gw_vcu_hmi_0x358_358::PackheatstatusType ret =  static_cast<Gw_vcu_hmi_0x358_358::PackheatstatusType>(x);
  return ret;
}

// config detail: {'name': 'packcoolingcircuiterror', 'enum': {0: 'PACKCOOLINGCIRCUITERROR_NORMAL', 1: 'PACKCOOLINGCIRCUITERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 10, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::PackcoolingcircuiterrorType Gwvcuhmi0x358358::packcoolingcircuiterror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  Gw_vcu_hmi_0x358_358::PackcoolingcircuiterrorType ret =  static_cast<Gw_vcu_hmi_0x358_358::PackcoolingcircuiterrorType>(x);
  return ret;
}

// config detail: {'name': 'motorcoolingcircuiterror', 'enum': {0: 'MOTORCOOLINGCIRCUITERROR_NORMAL', 1: 'MOTORCOOLINGCIRCUITERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::MotorcoolingcircuiterrorType Gwvcuhmi0x358358::motorcoolingcircuiterror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  Gw_vcu_hmi_0x358_358::MotorcoolingcircuiterrorType ret =  static_cast<Gw_vcu_hmi_0x358_358::MotorcoolingcircuiterrorType>(x);
  return ret;
}

// config detail: {'name': 'vcu_batteryvoltageerror', 'enum': {0: 'VCU_BATTERYVOLTAGEERROR_NO_WARNING', 1: 'VCU_BATTERYVOLTAGEERROR_WARNING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_batteryvoltageerrorType Gwvcuhmi0x358358::vcu_batteryvoltageerror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  Gw_vcu_hmi_0x358_358::Vcu_batteryvoltageerrorType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_batteryvoltageerrorType>(x);
  return ret;
}

// config detail: {'name': 'vcu_battervoltage', 'offset': 0.0, 'precision': 0.07, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|17.78]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'V'}
double Gwvcuhmi0x358358::vcu_battervoltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.070000;
  return ret;
}

// config detail: {'name': 'vcu_targcruisespeed', 'offset': 0.0, 'precision': 0.5, 'len': 9, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Km/h'}
double Gwvcuhmi0x358358::vcu_targcruisespeed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  double ret = x * 0.500000;
  return ret;
}

// config detail: {'name': 'vcu_cruisecontrolstatus', 'enum': {0: 'VCU_CRUISECONTROLSTATUS_CRUISECONTROLOOFF', 1: 'VCU_CRUISECONTROLSTATUS_ACTIVE', 2: 'VCU_CRUISECONTROLSTATUS_STANDBY', 3: 'VCU_CRUISECONTROLSTATUS_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 46, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_cruisecontrolstatusType Gwvcuhmi0x358358::vcu_cruisecontrolstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(5, 2);

  Gw_vcu_hmi_0x358_358::Vcu_cruisecontrolstatusType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_cruisecontrolstatusType>(x);
  return ret;
}

// config detail: {'name': 'vcu_driverstatus_sts', 'enum': {0: 'VCU_DRIVERSTATUS_STS_NO_DRIVE', 1: 'VCU_DRIVERSTATUS_STS_DRIVE_MODE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_driverstatus_stsType Gwvcuhmi0x358358::vcu_driverstatus_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Gw_vcu_hmi_0x358_358::Vcu_driverstatus_stsType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_driverstatus_stsType>(x);
  return ret;
}

// config detail: {'name': 'vcu_drivemode_sts', 'enum': {0: 'VCU_DRIVEMODE_STS_INVALID', 1: 'VCU_DRIVEMODE_STS_EV', 2: 'VCU_DRIVEMODE_STS_HEV', 3: 'VCU_DRIVEMODE_STS_FUEL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_drivemode_stsType Gwvcuhmi0x358358::vcu_drivemode_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 2);

  Gw_vcu_hmi_0x358_358::Vcu_drivemode_stsType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_drivemode_stsType>(x);
  return ret;
}

// config detail: {'name': 'vcu_vehchg_sts', 'enum': {0: 'VCU_VEHCHG_STS_INVALID', 1: 'VCU_VEHCHG_STS_STOPCHARGE', 2: 'VCU_VEHCHG_STS_DRIVECHARGE', 3: 'VCU_VEHCHG_STS_NOCHARGE', 4: 'VCU_VEHCHG_STS_CHARGECOMPLETED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_vehchg_stsType Gwvcuhmi0x358358::vcu_vehchg_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 3);

  Gw_vcu_hmi_0x358_358::Vcu_vehchg_stsType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_vehchg_stsType>(x);
  return ret;
}

// config detail: {'name': 'vcu_tbox_veh_sts', 'enum': {0: 'VCU_TBOX_VEH_STS_INVALID', 1: 'VCU_TBOX_VEH_STS_START', 2: 'VCU_TBOX_VEH_STS_OFF', 3: 'VCU_TBOX_VEH_STS_OTHER'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_tbox_veh_stsType Gwvcuhmi0x358358::vcu_tbox_veh_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 2);

  Gw_vcu_hmi_0x358_358::Vcu_tbox_veh_stsType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_tbox_veh_stsType>(x);
  return ret;
}

// config detail: {'name': 'vcu_icm_energyrecoverymode', 'enum': {0: 'VCU_ICM_ENERGYRECOVERYMODE_NO', 1: 'VCU_ICM_ENERGYRECOVERYMODE_LEVEL1', 2: 'VCU_ICM_ENERGYRECOVERYMODE_LEVEL2', 3: 'VCU_ICM_ENERGYRECOVERYMODE_LEVEL3'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_icm_energyrecoverymodeType Gwvcuhmi0x358358::vcu_icm_energyrecoverymode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 3);

  Gw_vcu_hmi_0x358_358::Vcu_icm_energyrecoverymodeType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_icm_energyrecoverymodeType>(x);
  return ret;
}

// config detail: {'name': 'vcu_icm_vehhvil_err', 'enum': {0: 'VCU_ICM_VEHHVIL_ERR_NORMAL', 1: 'VCU_ICM_VEHHVIL_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_icm_vehhvil_errType Gwvcuhmi0x358358::vcu_icm_vehhvil_err(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Gw_vcu_hmi_0x358_358::Vcu_icm_vehhvil_errType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_icm_vehhvil_errType>(x);
  return ret;
}

// config detail: {'name': 'vcu_icm_mot_err', 'enum': {0: 'VCU_ICM_MOT_ERR_NORMAL', 1: 'VCU_ICM_MOT_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_icm_mot_errType Gwvcuhmi0x358358::vcu_icm_mot_err(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Gw_vcu_hmi_0x358_358::Vcu_icm_mot_errType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_icm_mot_errType>(x);
  return ret;
}

// config detail: {'name': 'vcu_icm_packvoltoff_err', 'enum': {0: 'VCU_ICM_PACKVOLTOFF_ERR_NORMAL', 1: 'VCU_ICM_PACKVOLTOFF_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_icm_packvoltoff_errType Gwvcuhmi0x358358::vcu_icm_packvoltoff_err(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  Gw_vcu_hmi_0x358_358::Vcu_icm_packvoltoff_errType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_icm_packvoltoff_errType>(x);
  return ret;
}

// config detail: {'name': 'vcu_icm_chggearlv', 'enum': {0: 'VCU_ICM_CHGGEARLV_NO_WARNING', 1: 'VCU_ICM_CHGGEARLV_PLEASE_SET_GEARPOSITION_TO_PARK'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_icm_chggearlvType Gwvcuhmi0x358358::vcu_icm_chggearlv(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 2);

  Gw_vcu_hmi_0x358_358::Vcu_icm_chggearlvType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_icm_chggearlvType>(x);
  return ret;
}

// config detail: {'name': 'vcu_icm_packsys_err', 'enum': {0: 'VCU_ICM_PACKSYS_ERR_NORMAL', 1: 'VCU_ICM_PACKSYS_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_icm_packsys_errType Gwvcuhmi0x358358::vcu_icm_packsys_err(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  Gw_vcu_hmi_0x358_358::Vcu_icm_packsys_errType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_icm_packsys_errType>(x);
  return ret;
}

// config detail: {'name': 'vcu_icm_drvsys_err', 'enum': {0: 'VCU_ICM_DRVSYS_ERR_NORMAL', 1: 'VCU_ICM_DRVSYS_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_icm_drvsys_errType Gwvcuhmi0x358358::vcu_icm_drvsys_err(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  Gw_vcu_hmi_0x358_358::Vcu_icm_drvsys_errType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_icm_drvsys_errType>(x);
  return ret;
}

// config detail: {'name': 'vcu_icm_veh_err', 'enum': {0: 'VCU_ICM_VEH_ERR_NORMAL', 1: 'VCU_ICM_VEH_ERR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_vcu_hmi_0x358_358::Vcu_icm_veh_errType Gwvcuhmi0x358358::vcu_icm_veh_err(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Gw_vcu_hmi_0x358_358::Vcu_icm_veh_errType ret =  static_cast<Gw_vcu_hmi_0x358_358::Vcu_icm_veh_errType>(x);
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
