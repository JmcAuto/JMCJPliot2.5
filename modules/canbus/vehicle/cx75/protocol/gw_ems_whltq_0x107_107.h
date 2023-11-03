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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_EMS_WHLTQ_0X107_107_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_EMS_WHLTQ_0X107_107_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwemswhltq0x107107 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwemswhltq0x107107();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'MinIndicatedTorqWhl', 'enum': {65535: 'MININDICATEDTORQWHL_INVALID'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': -30000.0, 'physical_range': '[-30000|30000]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  int minindicatedtorqwhl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_AccRAWPedalRatio', 'enum': {255: 'EMS_ACCRAWPEDALRATIO_INVALID'}, 'precision': 0.3937, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|99.9998]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  double ems_accrawpedalratio(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_AccPedalRatio', 'enum': {255: 'EMS_ACCPEDALRATIO_INVALID'}, 'precision': 0.3937, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|99.9998]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': '%'}
  double ems_accpedalratio(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DCM_EMS_RollingCounter_0x107', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int dcm_ems_rollingcounter_0x107(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_KickDown', 'enum': {0: 'EMS_KICKDOWN_NOTACTIVE', 1: 'EMS_KICKDOWN_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_whltq_0x107_107::Ems_kickdownType ems_kickdown(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_ACCPedalRatioError', 'enum': {0: 'EMS_ACCPEDALRATIOERROR_NOERROR', 1: 'EMS_ACCPEDALRATIOERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_whltq_0x107_107::Ems_accpedalratioerrorType ems_accpedalratioerror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'EMS_BrkPedalStasus', 'enum': {0: 'EMS_BRKPEDALSTASUS_NOTPRESSED', 1: 'EMS_BRKPEDALSTASUS_PRESSED', 2: 'EMS_BRKPEDALSTASUS_RESERVED', 3: 'EMS_BRKPEDALSTASUS_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ems_whltq_0x107_107::Ems_brkpedalstasusType ems_brkpedalstasus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DCM_EMS_Checksum_0x107', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int dcm_ems_checksum_0x107(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MaxIndicatedTorqWhl', 'enum': {65535: 'MAXINDICATEDTORQWHL_INVALID'}, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'offset': -30000.0, 'physical_range': '[-30000|30000]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  int maxindicatedtorqwhl(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_EMS_WHLTQ_0X107_107_H_
