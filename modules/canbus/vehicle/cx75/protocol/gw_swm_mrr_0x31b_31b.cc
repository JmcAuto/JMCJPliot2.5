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

#include "modules/canbus/vehicle/cx75/protocol/gw_swm_mrr_0x31b_31b.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwswmmrr0x31b31b::Gwswmmrr0x31b31b() {}
const int32_t Gwswmmrr0x31b31b::ID = 0x31B;

void Gwswmmrr0x31b31b::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_acctaugapsetplus(swm_acctaugapsetplus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_accvsetminus(swm_accvsetminus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_shiftpadrequp(swm_shiftpadrequp(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_acclimphomests(swm_acclimphomests(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_laneassistswitch(swm_laneassistswitch(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_acctaugapsetminus(swm_acctaugapsetminus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_accvsetplus(swm_accvsetplus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_shiftpadflt(swm_shiftpadflt(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_accdeactivate(swm_accdeactivate(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_accresume(swm_accresume(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_accset(swm_accset(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_rolling_counter_0x31b(rolling_counter_0x31b(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_accenableswitch(swm_accenableswitch(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_checksum_0x31b(checksum_0x31b(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_accresume_qt(swm_accresume_qt(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_tjaswitch(swm_tjaswitch(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_mrr_0x31b_31b()->set_swm_shiftpadreqdown(swm_shiftpadreqdown(bytes, length));
}

// config detail: {'name': 'swm_acctaugapsetplus', 'enum': {0: 'SWM_ACCTAUGAPSETPLUS_NO_PRESS', 1: 'SWM_ACCTAUGAPSETPLUS_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_acctaugapsetplusType Gwswmmrr0x31b31b::swm_acctaugapsetplus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Gw_swm_mrr_0x31b_31b::Swm_acctaugapsetplusType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_acctaugapsetplusType>(x);
  return ret;
}

// config detail: {'name': 'swm_accvsetminus', 'enum': {0: 'SWM_ACCVSETMINUS_NO_PRESS', 1: 'SWM_ACCVSETMINUS_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_accvsetminusType Gwswmmrr0x31b31b::swm_accvsetminus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Gw_swm_mrr_0x31b_31b::Swm_accvsetminusType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_accvsetminusType>(x);
  return ret;
}

// config detail: {'name': 'swm_shiftpadrequp', 'enum': {0: 'SWM_SHIFTPADREQUP_NO_PRESS', 1: 'SWM_SHIFTPADREQUP_PRESS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 10, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_shiftpadrequpType Gwswmmrr0x31b31b::swm_shiftpadrequp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  Gw_swm_mrr_0x31b_31b::Swm_shiftpadrequpType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_shiftpadrequpType>(x);
  return ret;
}

// config detail: {'name': 'swm_acclimphomests', 'enum': {0: 'SWM_ACCLIMPHOMESTS_NORMAL', 1: 'SWM_ACCLIMPHOMESTS_LIMPHOME'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_acclimphomestsType Gwswmmrr0x31b31b::swm_acclimphomests(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  Gw_swm_mrr_0x31b_31b::Swm_acclimphomestsType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_acclimphomestsType>(x);
  return ret;
}

// config detail: {'name': 'swm_laneassistswitch', 'enum': {0: 'SWM_LANEASSISTSWITCH_PREVENT_LANEASSIST_CONTROL', 1: 'SWM_LANEASSISTSWITCH_ENABLE_LANEASSIST_CONTROL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_laneassistswitchType Gwswmmrr0x31b31b::swm_laneassistswitch(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  Gw_swm_mrr_0x31b_31b::Swm_laneassistswitchType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_laneassistswitchType>(x);
  return ret;
}

// config detail: {'name': 'swm_acctaugapsetminus', 'enum': {0: 'SWM_ACCTAUGAPSETMINUS_NO_PRESS', 1: 'SWM_ACCTAUGAPSETMINUS_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_acctaugapsetminusType Gwswmmrr0x31b31b::swm_acctaugapsetminus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Gw_swm_mrr_0x31b_31b::Swm_acctaugapsetminusType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_acctaugapsetminusType>(x);
  return ret;
}

// config detail: {'name': 'swm_accvsetplus', 'enum': {0: 'SWM_ACCVSETPLUS_NO_PRESS', 1: 'SWM_ACCVSETPLUS_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_accvsetplusType Gwswmmrr0x31b31b::swm_accvsetplus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  Gw_swm_mrr_0x31b_31b::Swm_accvsetplusType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_accvsetplusType>(x);
  return ret;
}

// config detail: {'name': 'swm_shiftpadflt', 'enum': {0: 'SWM_SHIFTPADFLT_NO_FAULT', 1: 'SWM_SHIFTPADFLT_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_shiftpadfltType Gwswmmrr0x31b31b::swm_shiftpadflt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  Gw_swm_mrr_0x31b_31b::Swm_shiftpadfltType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_shiftpadfltType>(x);
  return ret;
}

// config detail: {'name': 'swm_accdeactivate', 'enum': {0: 'SWM_ACCDEACTIVATE_NO_PRESS', 1: 'SWM_ACCDEACTIVATE_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_accdeactivateType Gwswmmrr0x31b31b::swm_accdeactivate(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  Gw_swm_mrr_0x31b_31b::Swm_accdeactivateType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_accdeactivateType>(x);
  return ret;
}

// config detail: {'name': 'swm_accresume', 'enum': {0: 'SWM_ACCRESUME_NO_PRESS', 1: 'SWM_ACCRESUME_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_accresumeType Gwswmmrr0x31b31b::swm_accresume(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  Gw_swm_mrr_0x31b_31b::Swm_accresumeType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_accresumeType>(x);
  return ret;
}

// config detail: {'name': 'swm_accset', 'enum': {0: 'SWM_ACCSET_NO_PRESS', 1: 'SWM_ACCSET_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_accsetType Gwswmmrr0x31b31b::swm_accset(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  Gw_swm_mrr_0x31b_31b::Swm_accsetType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_accsetType>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x31b', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwswmmrr0x31b31b::rolling_counter_0x31b(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'swm_accenableswitch', 'enum': {0: 'SWM_ACCENABLESWITCH_PREVENT_ACC_CONTROL', 1: 'SWM_ACCENABLESWITCH_ENABLE_ACC_CONTROL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_accenableswitchType Gwswmmrr0x31b31b::swm_accenableswitch(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  Gw_swm_mrr_0x31b_31b::Swm_accenableswitchType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_accenableswitchType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x31b', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwswmmrr0x31b31b::checksum_0x31b(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'swm_accresume_qt', 'enum': {0: 'SWM_ACCRESUME_QT_VALID', 1: 'SWM_ACCRESUME_QT_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_accresume_qtType Gwswmmrr0x31b31b::swm_accresume_qt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(7, 1);

  Gw_swm_mrr_0x31b_31b::Swm_accresume_qtType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_accresume_qtType>(x);
  return ret;
}

// config detail: {'name': 'swm_tjaswitch', 'enum': {0: 'SWM_TJASWITCH_PREVENT_TJA_CONTROL', 1: 'SWM_TJASWITCH_ENABLE_TJA_CONTROL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_tjaswitchType Gwswmmrr0x31b31b::swm_tjaswitch(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Gw_swm_mrr_0x31b_31b::Swm_tjaswitchType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_tjaswitchType>(x);
  return ret;
}

// config detail: {'name': 'swm_shiftpadreqdown', 'enum': {0: 'SWM_SHIFTPADREQDOWN_NO_PRESS', 1: 'SWM_SHIFTPADREQDOWN_PRESS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_mrr_0x31b_31b::Swm_shiftpadreqdownType Gwswmmrr0x31b31b::swm_shiftpadreqdown(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  Gw_swm_mrr_0x31b_31b::Swm_shiftpadreqdownType ret =  static_cast<Gw_swm_mrr_0x31b_31b::Swm_shiftpadreqdownType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
