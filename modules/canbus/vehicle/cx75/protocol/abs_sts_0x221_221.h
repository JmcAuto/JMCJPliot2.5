/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ABS_STS_0X221_221_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ABS_STS_0X221_221_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Abssts0x221221 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Abssts0x221221();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ABS_WhlMilgFrntRi', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int abs_whlmilgfrntri(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_VehSpdLgt', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.34375]', 'bit': 35, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
  double abs_vehspdlgt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_VehSpdDirection', 'enum': {0: 'ABS_VEHSPDDIRECTION_FORWARD', 1: 'ABS_VEHSPDDIRECTION_BACKWARD'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 36, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_sts_0x221_221::Abs_vehspddirectionType abs_vehspddirection(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_EbdFlgFlt', 'enum': {0: 'ABS_EBDFLGFLT_NO_FAILURE', 1: 'ABS_EBDFLGFLT_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_sts_0x221_221::Abs_ebdflgfltType abs_ebdflgflt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_AbsFlgFlt', 'enum': {0: 'ABS_ABSFLGFLT_NO_FAILURE', 1: 'ABS_ABSFLGFLT_FAILURE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 38, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_sts_0x221_221::Abs_absflgfltType abs_absflgflt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_AbsCtrlActv', 'enum': {0: 'ABS_ABSCTRLACTV_NOT_ACTIVE', 1: 'ABS_ABSCTRLACTV_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_sts_0x221_221::Abs_absctrlactvType abs_absctrlactv(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'RollingCounter_0x221', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rollingcounter_0x221(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_WhlMilgFrntLeStatus', 'enum': {0: 'ABS_WHLMILGFRNTLESTATUS_VALID', 1: 'ABS_WHLMILGFRNTLESTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_sts_0x221_221::Abs_whlmilgfrntlestatusType abs_whlmilgfrntlestatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_WhlMilgFrntRiStatus', 'enum': {0: 'ABS_WHLMILGFRNTRISTATUS_VALID', 1: 'ABS_WHLMILGFRNTRISTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_sts_0x221_221::Abs_whlmilgfrntristatusType abs_whlmilgfrntristatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_VehSpdLgtStatus', 'enum': {0: 'ABS_VEHSPDLGTSTATUS_VALID', 1: 'ABS_VEHSPDLGTSTATUS_INVALID', 2: 'ABS_VEHSPDLGTSTATUS_INIT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Abs_sts_0x221_221::Abs_vehspdlgtstatusType abs_vehspdlgtstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x221', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x221(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ABS_WhlMilgFrntLe', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int abs_whlmilgfrntle(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_ABS_STS_0X221_221_H_
