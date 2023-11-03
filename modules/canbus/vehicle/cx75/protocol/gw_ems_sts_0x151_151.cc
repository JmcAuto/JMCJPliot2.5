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

#include "modules/canbus/vehicle/cx75/protocol/gw_ems_sts_0x151_151.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwemssts0x151151::Gwemssts0x151151() {}
const int32_t Gwemssts0x151151::ID = 0x151;

void Gwemssts0x151151::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_ems_sts_0x151_151()->set_ems_engbarometricpressure(ems_engbarometricpressure(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_sts_0x151_151()->set_ems_vacuumpressure(ems_vacuumpressure(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_sts_0x151_151()->set_ems_targcruisespeed(ems_targcruisespeed(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_sts_0x151_151()->set_ems_atsdrivingmodestatus(ems_atsdrivingmodestatus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_sts_0x151_151()->set_ems_enginestopstartstatus(ems_enginestopstartstatus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_sts_0x151_151()->set_dcm_ems_rollingcounter_0x151(dcm_ems_rollingcounter_0x151(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_sts_0x151_151()->set_ems_cruisecontrolstatus(ems_cruisecontrolstatus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_sts_0x151_151()->set_ems_drivingmodechange_fault_flag(ems_drivingmodechange_fault_flag(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_sts_0x151_151()->set_dcm_ems_checksum_0x151(dcm_ems_checksum_0x151(bytes, length));
  chassis->mutable_cx75()->mutable_gw_ems_sts_0x151_151()->set_ems_engofftime(ems_engofftime(bytes, length));
}

// config detail: {'name': 'ems_engbarometricpressure', 'enum': {255: 'EMS_ENGBAROMETRICPRESSURE_INVALID'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|254]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'KPa'}
int Gwemssts0x151151::ems_engbarometricpressure(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'name': 'ems_vacuumpressure', 'enum': {1023: 'EMS_VACUUMPRESSURE_INVALID'}, 'precision': 0.1, 'len': 10, 'is_signed_var': False, 'offset': -92.7, 'physical_range': '[-92.7|5]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Kpa'}
double Gwemssts0x151151::ems_vacuumpressure(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'ems_targcruisespeed', 'enum': {511: 'EMS_TARGCRUISESPEED_INVALID'}, 'precision': 0.5, 'len': 9, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Km h'}
double Gwemssts0x151151::ems_targcruisespeed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret =  static_cast<double>(x);
  return ret;
}

// config detail: {'name': 'ems_atsdrivingmodestatus', 'enum': {0: 'EMS_ATSDRIVINGMODESTATUS_STANDARD', 1: 'EMS_ATSDRIVINGMODESTATUS_SPORT', 2: 'EMS_ATSDRIVINGMODESTATUS_ECO', 3: 'EMS_ATSDRIVINGMODESTATUS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_sts_0x151_151::Ems_atsdrivingmodestatusType Gwemssts0x151151::ems_atsdrivingmodestatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 2);

  Gw_ems_sts_0x151_151::Ems_atsdrivingmodestatusType ret =  static_cast<Gw_ems_sts_0x151_151::Ems_atsdrivingmodestatusType>(x);
  return ret;
}

// config detail: {'name': 'ems_enginestopstartstatus', 'enum': {0: 'EMS_ENGINESTOPSTARTSTATUS_NON_START_STOPMODE', 1: 'EMS_ENGINESTOPSTARTSTATUS_ENGINESTANDBY', 2: 'EMS_ENGINESTOPSTARTSTATUS_ENGINESTOPPED', 3: 'EMS_ENGINESTOPSTARTSTATUS_STARTERRESTART', 4: 'EMS_ENGINESTOPSTARTSTATUS_ENGINERESTART', 5: 'EMS_ENGINESTOPSTARTSTATUS_ENGINEOPERATION', 6: 'EMS_ENGINESTOPSTARTSTATUS_ENGINEAUTO_STOPPING', 7: 'EMS_ENGINESTOPSTARTSTATUS_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_sts_0x151_151::Ems_enginestopstartstatusType Gwemssts0x151151::ems_enginestopstartstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 3);

  Gw_ems_sts_0x151_151::Ems_enginestopstartstatusType ret =  static_cast<Gw_ems_sts_0x151_151::Ems_enginestopstartstatusType>(x);
  return ret;
}

// config detail: {'name': 'dcm_ems_rollingcounter_0x151', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwemssts0x151151::dcm_ems_rollingcounter_0x151(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ems_cruisecontrolstatus', 'enum': {0: 'EMS_CRUISECONTROLSTATUS_CRUISECONTROLOOFF', 1: 'EMS_CRUISECONTROLSTATUS_ACTIVE', 2: 'EMS_CRUISECONTROLSTATUS_STANDBY', 3: 'EMS_CRUISECONTROLSTATUS_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_sts_0x151_151::Ems_cruisecontrolstatusType Gwemssts0x151151::ems_cruisecontrolstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 2);

  Gw_ems_sts_0x151_151::Ems_cruisecontrolstatusType ret =  static_cast<Gw_ems_sts_0x151_151::Ems_cruisecontrolstatusType>(x);
  return ret;
}

// config detail: {'name': 'ems_drivingmodechange_fault_flag', 'enum': {0: 'EMS_DRIVINGMODECHANGE_FAULT_FLAG_NORMAL', 1: 'EMS_DRIVINGMODECHANGE_FAULT_FLAG_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_ems_sts_0x151_151::Ems_drivingmodechange_fault_flagType Gwemssts0x151151::ems_drivingmodechange_fault_flag(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Gw_ems_sts_0x151_151::Ems_drivingmodechange_fault_flagType ret =  static_cast<Gw_ems_sts_0x151_151::Ems_drivingmodechange_fault_flagType>(x);
  return ret;
}

// config detail: {'name': 'dcm_ems_checksum_0x151', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwemssts0x151151::dcm_ems_checksum_0x151(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'ems_engofftime', 'enum': {65535: 'EMS_ENGOFFTIME_INVALID'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|65534]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 's'}
int Gwemssts0x151151::ems_engofftime(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret =  static_cast<int>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
