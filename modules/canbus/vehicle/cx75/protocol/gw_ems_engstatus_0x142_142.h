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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_EMS_ENGSTATUS_0X142_142_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_EMS_ENGSTATUS_0X142_142_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwemsengstatus0x142142 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwemsengstatus0x142142();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'EMS_IgnitionTiming', 'enum': {60: 'EMS_IGNITIONTIMING_INVALID'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': -60.0, 'physical_range': '[-60|60]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': '\xa1\xe3[deg]'}
  int ems_ignitiontiming(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_SetlEngIdleSpeed', 'enum': {8191: 'EMS_SETLENGIDLESPEED_INVALID'}, 'precision': 0.5, 'len': 13, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4095]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'rpm'}
  double ems_setlengidlespeed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_EngOperationStatus', 'enum': {0: 'EMS_ENGOPERATIONSTATUS_STOPPED', 1: 'EMS_ENGOPERATIONSTATUS_RUNNING', 2: 'EMS_ENGOPERATIONSTATUS_IDLE', 3: 'EMS_ENGOPERATIONSTATUS_DFCO_RESERVED', 4: 'EMS_ENGOPERATIONSTATUS_CRANKING', 5: 'EMS_ENGOPERATIONSTATUS_STALLING'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_engstatus_0x142_142::Ems_engoperationstatusType ems_engoperationstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_AirCompressorStatus', 'enum': {0: 'EMS_AIRCOMPRESSORSTATUS_OFF', 1: 'EMS_AIRCOMPRESSORSTATUS_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 38, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_engstatus_0x142_142::Ems_aircompressorstatusType ems_aircompressorstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_ThrottlePlatePositionError', 'enum': {0: 'EMS_THROTTLEPLATEPOSITIONERROR_NOERROR', 1: 'EMS_THROTTLEPLATEPOSITIONERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_engstatus_0x142_142::Ems_throttleplatepositionerrorType ems_throttleplatepositionerror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_StartStopMessage', 'enum': {0: 'EMS_STARTSTOPMESSAGE_NOWARNINGMESSAGE', 1: 'EMS_STARTSTOPMESSAGE_CLIMATEREQUEST', 2: 'EMS_STARTSTOPMESSAGE_BRAKELOW', 3: 'EMS_STARTSTOPMESSAGE_START_STOPOFF', 4: 'EMS_STARTSTOPMESSAGE_BATTERYLOW', 5: 'EMS_STARTSTOPMESSAGE_ECTLOW', 6: 'EMS_STARTSTOPMESSAGE_APAINHIBIT', 7: 'EMS_STARTSTOPMESSAGE_ACCINHIBIT', 8: 'EMS_STARTSTOPMESSAGE_TCUINHIBIT', 9: 'EMS_STARTSTOPMESSAGE_STARTPROTECT', 10: 'EMS_STARTSTOPMESSAGE_HOODOPEN', 11: 'EMS_STARTSTOPMESSAGE_DRVIEDOOROPNE', 12: 'EMS_STARTSTOPMESSAGE_STEERINGANGELHIGH', 13: 'EMS_STARTSTOPMESSAGE_STARTSTOPFAILURE', 14: 'EMS_STARTSTOPMESSAGE_MANUALLYRESTART', 15: 'EMS_STARTSTOPMESSAGE_RESERVED'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_engstatus_0x142_142::Ems_startstopmessageType ems_startstopmessage(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DCM_EMS_RollingCounter_0x142', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int dcm_ems_rollingcounter_0x142(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_IgnSwtSts', 'enum': {0: 'EMS_IGNSWTSTS_IGNITIONOFF', 1: 'EMS_IGNSWTSTS_IGNITIONON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_engstatus_0x142_142::Ems_ignswtstsType ems_ignswtsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DCM_EMS_Checksum_0x142', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int dcm_ems_checksum_0x142(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_EngThrottlePosition', 'enum': {255: 'EMS_ENGTHROTTLEPOSITION_INVALID'}, 'precision': 0.392, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|99.568]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': '%'}
  double ems_engthrottleposition(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_EMS_ENGSTATUS_0X142_142_H_
