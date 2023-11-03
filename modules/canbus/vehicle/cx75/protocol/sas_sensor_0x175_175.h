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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_SAS_SENSOR_0X175_175_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_SAS_SENSOR_0X175_175_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Sassensor0x175175 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Sassensor0x175175();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'SAS_Raw_SteerWheelAngle', 'enum': {65535: 'SAS_RAW_STEERWHEELANGLE_INVALID'}, 'precision': 0.0238, 'len': 16, 'is_signed_var': False, 'offset': -780.0, 'physical_range': '[-780|779.7092]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'deg'}
  double sas_raw_steerwheelangle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SAS_SteerWheelRotSpd', 'enum': {16383: 'SAS_STEERWHEELROTSPD_INVALID'}, 'precision': 0.125, 'len': 14, 'is_signed_var': False, 'offset': -1024.0, 'physical_range': '[-1024|1023.75]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'deg s'}
  double sas_steerwheelrotspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SAS_TrimmingSts', 'enum': {0: 'SAS_TRIMMINGSTS_NOT_TRIMMED', 1: 'SAS_TRIMMINGSTS_TRIMMED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sas_sensor_0x175_175::Sas_trimmingstsType sas_trimmingsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SAS_SteerWheelRotSpdStatus', 'enum': {0: 'SAS_STEERWHEELROTSPDSTATUS_VALID', 1: 'SAS_STEERWHEELROTSPDSTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sas_sensor_0x175_175::Sas_steerwheelrotspdstatusType sas_steerwheelrotspdstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x175', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x175(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SAS_SASStsCal', 'enum': {0: 'SAS_SASSTSCAL_SENSOR_NOT_CALIBRATED', 1: 'SAS_SASSTSCAL_SENSOR_CALIBRATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sas_sensor_0x175_175::Sas_sasstscalType sas_sasstscal(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SAS_Raw_SASStsSnsr', 'enum': {0: 'SAS_RAW_SASSTSSNSR_SENSOR_VALUE_INVALID', 1: 'SAS_RAW_SASSTSSNSR_SENSOR_VALUE_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sas_sensor_0x175_175::Sas_raw_sasstssnsrType sas_raw_sasstssnsr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SAS_SASStsSnsr', 'enum': {0: 'SAS_SASSTSSNSR_SENSOR_VALUE_INVALID', 1: 'SAS_SASSTSSNSR_SENSOR_VALUE_VALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sas_sensor_0x175_175::Sas_sasstssnsrType sas_sasstssnsr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x175', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x175(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SAS_SteerWheelAngle', 'enum': {65535: 'SAS_STEERWHEELANGLE_INVALID'}, 'precision': 0.0238, 'len': 16, 'is_signed_var': False, 'offset': -780.0, 'physical_range': '[-780|779.7092]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'deg'}
  double sas_steerwheelangle(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_SAS_SENSOR_0X175_175_H_
