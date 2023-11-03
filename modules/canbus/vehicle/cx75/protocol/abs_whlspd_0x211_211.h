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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_ABS_WHLSPD_0X211_211_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_ABS_WHLSPD_0X211_211_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Abswhlspd0x211211 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Abswhlspd0x211211();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ABS_WhlSpdFrntRi', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 11, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
  double abs_whlspdfrntri(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_WhlSpdReLe', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
  double abs_whlspdrele(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_WhlSpdReRi', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 35, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
  double abs_whlspdreri(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RollingCounter_0x211', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rollingcounter_0x211(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_WhlSpdReRiStatus', 'enum': {0: 'ABS_WHLSPDRERISTATUS_VALID', 1: 'ABS_WHLSPDRERISTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_whlspd_0x211_211::Abs_whlspdreristatusType abs_whlspdreristatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_WhlSpdReLeStatus', 'enum': {0: 'ABS_WHLSPDRELESTATUS_VALID', 1: 'ABS_WHLSPDRELESTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_whlspd_0x211_211::Abs_whlspdrelestatusType abs_whlspdrelestatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_WhlSpdFrntRiStatus', 'enum': {0: 'ABS_WHLSPDFRNTRISTATUS_VALID', 1: 'ABS_WHLSPDFRNTRISTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_whlspd_0x211_211::Abs_whlspdfrntristatusType abs_whlspdfrntristatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_WhlSpdFrntLeStatus', 'enum': {0: 'ABS_WHLSPDFRNTLESTATUS_VALID', 1: 'ABS_WHLSPDFRNTLESTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_whlspd_0x211_211::Abs_whlspdfrntlestatusType abs_whlspdfrntlestatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x211', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x211(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_WhlSpdFrntLe', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
  double abs_whlspdfrntle(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_ABS_WHLSPD_0X211_211_H_
