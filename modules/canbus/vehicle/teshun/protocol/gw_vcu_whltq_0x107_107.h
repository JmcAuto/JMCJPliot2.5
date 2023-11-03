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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_WHLTQ_0X107_107_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_GW_VCU_WHLTQ_0X107_107_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Gwvcuwhltq0x107107 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwvcuwhltq0x107107();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Checksum_0x107', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x107(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x107', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x107(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_KickDown', 'enum': {0: 'VCU_KICKDOWN_NOTACTIVE', 1: 'VCU_KICKDOWN_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_whltq_0x107_107::Vcu_kickdownType vcu_kickdown(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_ACCPedalRatioError', 'enum': {0: 'VCU_ACCPEDALRATIOERROR_NOERROR', 1: 'VCU_ACCPEDALRATIOERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_whltq_0x107_107::Vcu_accpedalratioerrorType vcu_accpedalratioerror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_AccPedalRatio', 'offset': 0.0, 'precision': 0.3937, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|99.9998]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  double vcu_accpedalratio(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_AccRAWPedalRatio', 'offset': 0.0, 'precision': 0.3937, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|99.9998]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
  double vcu_accrawpedalratio(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_BrkPedalStasus', 'enum': {0: 'VCU_BRKPEDALSTASUS_NOTPRESSED', 1: 'VCU_BRKPEDALSTASUS_PRESSED', 2: 'VCU_BRKPEDALSTASUS_RESERVED', 3: 'VCU_BRKPEDALSTASUS_ERROR'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_vcu_whltq_0x107_107::Vcu_brkpedalstasusType vcu_brkpedalstasus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_MinIndicatedTorqWhl', 'offset': -30000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-30000|30000]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  int vcu_minindicatedtorqwhl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VCU_MaxIndicatedTorqWhl', 'offset': -30000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-30000|30000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  int vcu_maxindicatedtorqwhl(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_GW_VCU_WHLTQ_0X107_107_H_
