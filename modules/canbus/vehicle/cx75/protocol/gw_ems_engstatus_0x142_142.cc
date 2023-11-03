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

#include "modules/canbus/vehicle/cx75/protocol/gw_ems_engstatus_0x142_142.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwemsengstatus0x142142::Gwemsengstatus0x142142() {}
const int32_t Gwemsengstatus0x142142::ID = 0x142;

void Gwemsengstatus0x142142::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_ems_engstatus_0x142_142()->set_ems_ignitiontiming(ems_ignitiontiming(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_engstatus_0x142_142()->set_ems_setlengidlespeed(ems_setlengidlespeed(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_engstatus_0x142_142()->set_ems_engoperationstatus(ems_engoperationstatus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_engstatus_0x142_142()->set_ems_aircompressorstatus(ems_aircompressorstatus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_engstatus_0x142_142()->set_ems_throttleplatepositionerror(ems_throttleplatepositionerror(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_engstatus_0x142_142()->set_ems_startstopmessage(ems_startstopmessage(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_engstatus_0x142_142()->set_dcm_ems_rollingcounter_0x142(dcm_ems_rollingcounter_0x142(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_engstatus_0x142_142()->set_ems_ignswtsts(ems_ignswtsts(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_engstatus_0x142_142()->set_dcm_ems_checksum_0x142(dcm_ems_checksum_0x142(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_engstatus_0x142_142()->set_ems_engthrottleposition(ems_engthrottleposition(bytes, length));
}

// config detail: {'name': 'ems_ignitiontiming', 'enum': {60: 'EMS_IGNITIONTIMING_INVALID'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': -60.0, 'physical_range': '[-60|60]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': '\xa1\xe3[deg]'}
int Gwemsengstatus0x142142::ems_ignitiontiming(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'name': 'ems_setlengidlespeed', 'enum': {8191: 'EMS_SETLENGIDLESPEED_INVALID'}, 'precision': 0.5, 'len': 13, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|4095]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'rpm'}
double Gwemsengstatus0x142142::ems_setlengidlespeed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'ems_engoperationstatus', 'enum': {0: 'EMS_ENGOPERATIONSTATUS_STOPPED', 1: 'EMS_ENGOPERATIONSTATUS_RUNNING', 2: 'EMS_ENGOPERATIONSTATUS_IDLE', 3: 'EMS_ENGOPERATIONSTATUS_DFCO_RESERVED', 4: 'EMS_ENGOPERATIONSTATUS_CRANKING', 5: 'EMS_ENGOPERATIONSTATUS_STALLING'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|5]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_engstatus_0x142_142::Ems_engoperationstatusType Gwemsengstatus0x142142::ems_engoperationstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 3);

  Gw_ems_engstatus_0x142_142::Ems_engoperationstatusType ret =  static_cast<Gw_ems_engstatus_0x142_142::Ems_engoperationstatusType>(x);
  return ret;
}

// config detail: {'name': 'ems_aircompressorstatus', 'enum': {0: 'EMS_AIRCOMPRESSORSTATUS_OFF', 1: 'EMS_AIRCOMPRESSORSTATUS_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 38, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_engstatus_0x142_142::Ems_aircompressorstatusType Gwemsengstatus0x142142::ems_aircompressorstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 1);

  Gw_ems_engstatus_0x142_142::Ems_aircompressorstatusType ret =  static_cast<Gw_ems_engstatus_0x142_142::Ems_aircompressorstatusType>(x);
  return ret;
}

// config detail: {'name': 'ems_throttleplatepositionerror', 'enum': {0: 'EMS_THROTTLEPLATEPOSITIONERROR_NOERROR', 1: 'EMS_THROTTLEPLATEPOSITIONERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_engstatus_0x142_142::Ems_throttleplatepositionerrorType Gwemsengstatus0x142142::ems_throttleplatepositionerror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  Gw_ems_engstatus_0x142_142::Ems_throttleplatepositionerrorType ret =  static_cast<Gw_ems_engstatus_0x142_142::Ems_throttleplatepositionerrorType>(x);
  return ret;
}

// config detail: {'name': 'ems_startstopmessage', 'enum': {0: 'EMS_STARTSTOPMESSAGE_NOWARNINGMESSAGE', 1: 'EMS_STARTSTOPMESSAGE_CLIMATEREQUEST', 2: 'EMS_STARTSTOPMESSAGE_BRAKELOW', 3: 'EMS_STARTSTOPMESSAGE_START_STOPOFF', 4: 'EMS_STARTSTOPMESSAGE_BATTERYLOW', 5: 'EMS_STARTSTOPMESSAGE_ECTLOW', 6: 'EMS_STARTSTOPMESSAGE_APAINHIBIT', 7: 'EMS_STARTSTOPMESSAGE_ACCINHIBIT', 8: 'EMS_STARTSTOPMESSAGE_TCUINHIBIT', 9: 'EMS_STARTSTOPMESSAGE_STARTPROTECT', 10: 'EMS_STARTSTOPMESSAGE_HOODOPEN', 11: 'EMS_STARTSTOPMESSAGE_DRVIEDOOROPNE', 12: 'EMS_STARTSTOPMESSAGE_STEERINGANGELHIGH', 13: 'EMS_STARTSTOPMESSAGE_STARTSTOPFAILURE', 14: 'EMS_STARTSTOPMESSAGE_MANUALLYRESTART', 15: 'EMS_STARTSTOPMESSAGE_RESERVED'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_engstatus_0x142_142::Ems_startstopmessageType Gwemsengstatus0x142142::ems_startstopmessage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 4);

  Gw_ems_engstatus_0x142_142::Ems_startstopmessageType ret =  static_cast<Gw_ems_engstatus_0x142_142::Ems_startstopmessageType>(x);
  return ret;
}

// config detail: {'name': 'dcm_ems_rollingcounter_0x142', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwemsengstatus0x142142::dcm_ems_rollingcounter_0x142(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ems_ignswtsts', 'enum': {0: 'EMS_IGNSWTSTS_IGNITIONOFF', 1: 'EMS_IGNSWTSTS_IGNITIONON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_engstatus_0x142_142::Ems_ignswtstsType Gwemsengstatus0x142142::ems_ignswtsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Gw_ems_engstatus_0x142_142::Ems_ignswtstsType ret =  static_cast<Gw_ems_engstatus_0x142_142::Ems_ignswtstsType>(x);
  return ret;
}

// config detail: {'name': 'dcm_ems_checksum_0x142', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwemsengstatus0x142142::dcm_ems_checksum_0x142(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ems_engthrottleposition', 'enum': {255: 'EMS_ENGTHROTTLEPOSITION_INVALID'}, 'precision': 0.392, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|99.568]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': '%'}
double Gwemsengstatus0x142142::ems_engthrottleposition(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret =  static_cast<double>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
