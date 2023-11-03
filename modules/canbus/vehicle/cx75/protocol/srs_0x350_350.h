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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_SRS_0X350_350_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_SRS_0X350_350_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Srs0x350350 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Srs0x350350();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'SRS_SBR_SecondRowRight', 'enum': {0: 'SRS_SBR_SECONDROWRIGHT_LAMP_OFF', 1: 'SRS_SBR_SECONDROWRIGHT_LAMP_FLASHING_RESERVED', 2: 'SRS_SBR_SECONDROWRIGHT_LAMP_ON', 3: 'SRS_SBR_SECONDROWRIGHT_FAULT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Srs_0x350_350::Srs_sbr_secondrowrightType srs_sbr_secondrowright(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SRS_SBR_SecondRowMid', 'enum': {0: 'SRS_SBR_SECONDROWMID_LAMP_OFF', 1: 'SRS_SBR_SECONDROWMID_LAMP_FLASHING_RESERVED', 2: 'SRS_SBR_SECONDROWMID_LAMP_ON', 3: 'SRS_SBR_SECONDROWMID_FAULT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Srs_0x350_350::Srs_sbr_secondrowmidType srs_sbr_secondrowmid(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SRS_SBR_SecondRowLeft', 'enum': {0: 'SRS_SBR_SECONDROWLEFT_LAMP_OFF', 1: 'SRS_SBR_SECONDROWLEFT_LAMP_FLASHING_RESERVED', 2: 'SRS_SBR_SECONDROWLEFT_LAMP_ON', 3: 'SRS_SBR_SECONDROWLEFT_FAULT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Srs_0x350_350::Srs_sbr_secondrowleftType srs_sbr_secondrowleft(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SRS_SBR_PAMsenger', 'enum': {0: 'SRS_SBR_PAMSENGER_LAMP_OFF', 1: 'SRS_SBR_PAMSENGER_LAMP_FLASHING_RESERVED', 2: 'SRS_SBR_PAMSENGER_LAMP_ON', 3: 'SRS_SBR_PAMSENGER_FAULT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Srs_0x350_350::Srs_sbr_pamsengerType srs_sbr_pamsenger(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x350', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x350(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SRS_SBR_Driver', 'enum': {0: 'SRS_SBR_DRIVER_LAMP_OFF', 1: 'SRS_SBR_DRIVER_LAMP_FLASHING_RESERVED', 2: 'SRS_SBR_DRIVER_LAMP_ON', 3: 'SRS_SBR_DRIVER_FAULT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Srs_0x350_350::Srs_sbr_driverType srs_sbr_driver(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x350', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x350(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SRS_CrashOutputSts', 'enum': {0: 'SRS_CRASHOUTPUTSTS_NO_CRASH', 1: 'SRS_CRASHOUTPUTSTS_CRASH'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Srs_0x350_350::Srs_crashoutputstsType srs_crashoutputsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SRS_AirBagFailSts', 'enum': {0: 'SRS_AIRBAGFAILSTS_LAMP_OFF', 1: 'SRS_AIRBAGFAILSTS_LAMP_FLASH', 2: 'SRS_AIRBAGFAILSTS_LAMP_ON', 3: 'SRS_AIRBAGFAILSTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Srs_0x350_350::Srs_airbagfailstsType srs_airbagfailsts(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_SRS_0X350_350_H_
