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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_EMS_STS_0X151_151_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_EMS_STS_0X151_151_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwemssts0x151151 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwemssts0x151151();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'EMS_EngBarometricPressure', 'enum': {255: 'EMS_ENGBAROMETRICPRESSURE_INVALID'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|254]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'KPa'}
  int ems_engbarometricpressure(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_VacuumPressure', 'enum': {1023: 'EMS_VACUUMPRESSURE_INVALID'}, 'precision': 0.1, 'len': 10, 'is_signed_var': False, 'offset': -92.7, 'physical_range': '[-92.7|5]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Kpa'}
  double ems_vacuumpressure(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_TargCruiseSpeed', 'enum': {511: 'EMS_TARGCRUISESPEED_INVALID'}, 'precision': 0.5, 'len': 9, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Km h'}
  double ems_targcruisespeed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_ATSDrivingModeStatus', 'enum': {0: 'EMS_ATSDRIVINGMODESTATUS_STANDARD', 1: 'EMS_ATSDRIVINGMODESTATUS_SPORT', 2: 'EMS_ATSDRIVINGMODESTATUS_ECO', 3: 'EMS_ATSDRIVINGMODESTATUS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_sts_0x151_151::Ems_atsdrivingmodestatusType ems_atsdrivingmodestatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_EngineStopStartStatus', 'enum': {0: 'EMS_ENGINESTOPSTARTSTATUS_NON_START_STOPMODE', 1: 'EMS_ENGINESTOPSTARTSTATUS_ENGINESTANDBY', 2: 'EMS_ENGINESTOPSTARTSTATUS_ENGINESTOPPED', 3: 'EMS_ENGINESTOPSTARTSTATUS_STARTERRESTART', 4: 'EMS_ENGINESTOPSTARTSTATUS_ENGINERESTART', 5: 'EMS_ENGINESTOPSTARTSTATUS_ENGINEOPERATION', 6: 'EMS_ENGINESTOPSTARTSTATUS_ENGINEAUTO_STOPPING', 7: 'EMS_ENGINESTOPSTARTSTATUS_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_sts_0x151_151::Ems_enginestopstartstatusType ems_enginestopstartstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DCM_EMS_RollingCounter_0x151', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int dcm_ems_rollingcounter_0x151(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_CruiseControlStatus', 'enum': {0: 'EMS_CRUISECONTROLSTATUS_CRUISECONTROLOOFF', 1: 'EMS_CRUISECONTROLSTATUS_ACTIVE', 2: 'EMS_CRUISECONTROLSTATUS_STANDBY', 3: 'EMS_CRUISECONTROLSTATUS_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_sts_0x151_151::Ems_cruisecontrolstatusType ems_cruisecontrolstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_DrivingModeChange_Fault_flag', 'enum': {0: 'EMS_DRIVINGMODECHANGE_FAULT_FLAG_NORMAL', 1: 'EMS_DRIVINGMODECHANGE_FAULT_FLAG_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_sts_0x151_151::Ems_drivingmodechange_fault_flagType ems_drivingmodechange_fault_flag(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DCM_EMS_Checksum_0x151', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int dcm_ems_checksum_0x151(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_EngOffTime', 'enum': {65535: 'EMS_ENGOFFTIME_INVALID'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|65534]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 's'}
  int ems_engofftime(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_EMS_STS_0X151_151_H_
