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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_EMS_TQ_0X101_101_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_EMS_TQ_0X101_101_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwemstq0x101101 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwemstq0x101101();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'EMS_IndicatedRealEngTorq', 'enum': {2047: 'EMS_INDICATEDREALENGTORQ_INVALID'}, 'precision': 0.5, 'len': 12, 'is_signed_var': False, 'offset': -1000.0, 'physical_range': '[-1000|1000]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  double ems_indicatedrealengtorq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_EngSpeed', 'enum': {32767: 'EMS_ENGSPEED_INVALID'}, 'precision': 0.5, 'len': 15, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|16383]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'rpm'}
  double ems_engspeed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_EngSpeedError', 'enum': {0: 'EMS_ENGSPEEDERROR_NOERROR', 1: 'EMS_ENGSPEEDERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_tq_0x101_101::Ems_engspeederrorType ems_engspeederror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_RngModTorqCrSLeadMin', 'enum': {2047: 'EMS_RNGMODTORQCRSLEADMIN_INVALID'}, 'precision': 0.5, 'len': 12, 'is_signed_var': False, 'offset': -1000.0, 'physical_range': '[-1000|1000]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  double ems_rngmodtorqcrsleadmin(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DCM_EMS_RollingCounter_0x101', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int dcm_ems_rollingcounter_0x101(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DCM_EMS_Checksum_0x101', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int dcm_ems_checksum_0x101(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_RngModTorqCrSLeadMax', 'enum': {2047: 'EMS_RNGMODTORQCRSLEADMAX_INVALID'}, 'precision': 0.5, 'len': 12, 'is_signed_var': False, 'offset': -1000.0, 'physical_range': '[-1000|1000]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  double ems_rngmodtorqcrsleadmax(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_EMS_TQ_0X101_101_H_
