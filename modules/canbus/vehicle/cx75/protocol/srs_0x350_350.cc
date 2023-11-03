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

#include "modules/canbus/vehicle/cx75/protocol/srs_0x350_350.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Srs0x350350::Srs0x350350() {}
const int32_t Srs0x350350::ID = 0x350;

void Srs0x350350::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_srs_0x350_350()->set_srs_sbr_secondrowright(srs_sbr_secondrowright(bytes, length));
  chassis->mutable_cx75()->mutable_srs_0x350_350()->set_srs_sbr_secondrowmid(srs_sbr_secondrowmid(bytes, length));
  chassis->mutable_cx75()->mutable_srs_0x350_350()->set_srs_sbr_secondrowleft(srs_sbr_secondrowleft(bytes, length));
  chassis->mutable_cx75()->mutable_srs_0x350_350()->set_srs_sbr_pamsenger(srs_sbr_pamsenger(bytes, length));
  chassis->mutable_cx75()->mutable_srs_0x350_350()->set_rolling_counter_0x350(rolling_counter_0x350(bytes, length));
  chassis->mutable_cx75()->mutable_srs_0x350_350()->set_srs_sbr_driver(srs_sbr_driver(bytes, length));
  chassis->mutable_cx75()->mutable_srs_0x350_350()->set_checksum_0x350(checksum_0x350(bytes, length));
  chassis->mutable_cx75()->mutable_srs_0x350_350()->set_srs_crashoutputsts(srs_crashoutputsts(bytes, length));
  chassis->mutable_cx75()->mutable_srs_0x350_350()->set_srs_airbagfailsts(srs_airbagfailsts(bytes, length));
}

// config detail: {'name': 'srs_sbr_secondrowright', 'enum': {0: 'SRS_SBR_SECONDROWRIGHT_LAMP_OFF', 1: 'SRS_SBR_SECONDROWRIGHT_LAMP_FLASHING_RESERVED', 2: 'SRS_SBR_SECONDROWRIGHT_LAMP_ON', 3: 'SRS_SBR_SECONDROWRIGHT_FAULT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Srs_0x350_350::Srs_sbr_secondrowrightType Srs0x350350::srs_sbr_secondrowright(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  Srs_0x350_350::Srs_sbr_secondrowrightType ret =  static_cast<Srs_0x350_350::Srs_sbr_secondrowrightType>(x);
  return ret;
}

// config detail: {'name': 'srs_sbr_secondrowmid', 'enum': {0: 'SRS_SBR_SECONDROWMID_LAMP_OFF', 1: 'SRS_SBR_SECONDROWMID_LAMP_FLASHING_RESERVED', 2: 'SRS_SBR_SECONDROWMID_LAMP_ON', 3: 'SRS_SBR_SECONDROWMID_FAULT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Srs_0x350_350::Srs_sbr_secondrowmidType Srs0x350350::srs_sbr_secondrowmid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 2);

  Srs_0x350_350::Srs_sbr_secondrowmidType ret =  static_cast<Srs_0x350_350::Srs_sbr_secondrowmidType>(x);
  return ret;
}

// config detail: {'name': 'srs_sbr_secondrowleft', 'enum': {0: 'SRS_SBR_SECONDROWLEFT_LAMP_OFF', 1: 'SRS_SBR_SECONDROWLEFT_LAMP_FLASHING_RESERVED', 2: 'SRS_SBR_SECONDROWLEFT_LAMP_ON', 3: 'SRS_SBR_SECONDROWLEFT_FAULT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Srs_0x350_350::Srs_sbr_secondrowleftType Srs0x350350::srs_sbr_secondrowleft(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 2);

  Srs_0x350_350::Srs_sbr_secondrowleftType ret =  static_cast<Srs_0x350_350::Srs_sbr_secondrowleftType>(x);
  return ret;
}

// config detail: {'name': 'srs_sbr_pamsenger', 'enum': {0: 'SRS_SBR_PAMSENGER_LAMP_OFF', 1: 'SRS_SBR_PAMSENGER_LAMP_FLASHING_RESERVED', 2: 'SRS_SBR_PAMSENGER_LAMP_ON', 3: 'SRS_SBR_PAMSENGER_FAULT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Srs_0x350_350::Srs_sbr_pamsengerType Srs0x350350::srs_sbr_pamsenger(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 2);

  Srs_0x350_350::Srs_sbr_pamsengerType ret =  static_cast<Srs_0x350_350::Srs_sbr_pamsengerType>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x350', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Srs0x350350::rolling_counter_0x350(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'srs_sbr_driver', 'enum': {0: 'SRS_SBR_DRIVER_LAMP_OFF', 1: 'SRS_SBR_DRIVER_LAMP_FLASHING_RESERVED', 2: 'SRS_SBR_DRIVER_LAMP_ON', 3: 'SRS_SBR_DRIVER_FAULT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Srs_0x350_350::Srs_sbr_driverType Srs0x350350::srs_sbr_driver(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 2);

  Srs_0x350_350::Srs_sbr_driverType ret =  static_cast<Srs_0x350_350::Srs_sbr_driverType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x350', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Srs0x350350::checksum_0x350(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'srs_crashoutputsts', 'enum': {0: 'SRS_CRASHOUTPUTSTS_NO_CRASH', 1: 'SRS_CRASHOUTPUTSTS_CRASH'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Srs_0x350_350::Srs_crashoutputstsType Srs0x350350::srs_crashoutputsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Srs_0x350_350::Srs_crashoutputstsType ret =  static_cast<Srs_0x350_350::Srs_crashoutputstsType>(x);
  return ret;
}

// config detail: {'name': 'srs_airbagfailsts', 'enum': {0: 'SRS_AIRBAGFAILSTS_LAMP_OFF', 1: 'SRS_AIRBAGFAILSTS_LAMP_FLASH', 2: 'SRS_AIRBAGFAILSTS_LAMP_ON', 3: 'SRS_AIRBAGFAILSTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Srs_0x350_350::Srs_airbagfailstsType Srs0x350350::srs_airbagfailsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  Srs_0x350_350::Srs_airbagfailstsType ret =  static_cast<Srs_0x350_350::Srs_airbagfailstsType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
