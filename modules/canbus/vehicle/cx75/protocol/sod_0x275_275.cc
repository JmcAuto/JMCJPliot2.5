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

#include "modules/canbus/vehicle/cx75/protocol/sod_0x275_275.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Sod0x275275::Sod0x275275() {}
const int32_t Sod0x275275::ID = 0x275;

void Sod0x275275::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_lca_warningreqright(sod_lca_warningreqright(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_blis_active(sod_blis_active(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_blis_display(sod_blis_display(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_lca_warningreqleft(sod_lca_warningreqleft(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_doa_warningreqleftrear(sod_doa_warningreqleftrear(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_doa_warningreqleftfront(sod_doa_warningreqleftfront(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_cta_warningreqleft(sod_cta_warningreqleft(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_doa_state(sod_doa_state(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_cta_state(sod_cta_state(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_sodlca_state(sod_sodlca_state(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_doa_offtelltale(doa_offtelltale(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_doa_warningreqrightrear(sod_doa_warningreqrightrear(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_cta_offtelltale(cta_offtelltale(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sodlca_offtelltale(sodlca_offtelltale(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_doa_warningreqrightfront(sod_doa_warningreqrightfront(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_rolling_counter_0x275(rolling_counter_0x275(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_checksum_0x275(checksum_0x275(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_cta_warningreqright(sod_cta_warningreqright(bytes, length));
  chassis->mutable_cx75()->mutable_sod_0x275_275()->set_sod_autoalignmentflag(sod_autoalignmentflag(bytes, length));
}

// config detail: {'name': 'sod_lca_warningreqright', 'enum': {0: 'SOD_LCA_WARNINGREQRIGHT_NO_WARNING', 1: 'SOD_LCA_WARNINGREQRIGHT_WARNING_LEVEL_1', 2: 'SOD_LCA_WARNINGREQRIGHT_WARNING_LEVEL_2', 3: 'SOD_LCA_WARNINGREQRIGHT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_lca_warningreqrightType Sod0x275275::sod_lca_warningreqright(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Sod_0x275_275::Sod_lca_warningreqrightType ret =  static_cast<Sod_0x275_275::Sod_lca_warningreqrightType>(x);
  return ret;
}

// config detail: {'name': 'sod_blis_active', 'enum': {0: 'SOD_BLIS_ACTIVE_NO_ACTIVE', 1: 'SOD_BLIS_ACTIVE_STANBY', 2: 'SOD_BLIS_ACTIVE_ACTIVE', 3: 'SOD_BLIS_ACTIVE_FAIL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_blis_activeType Sod0x275275::sod_blis_active(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  Sod_0x275_275::Sod_blis_activeType ret =  static_cast<Sod_0x275_275::Sod_blis_activeType>(x);
  return ret;
}

// config detail: {'name': 'sod_blis_display', 'enum': {0: 'SOD_BLIS_DISPLAY_NO_DISPLAY', 1: 'SOD_BLIS_DISPLAY_SOD_LCA_CTA_DOA_ERROR', 2: 'SOD_BLIS_DISPLAY_SOD_LCA_CTA_DOA_BLINDNESS', 3: 'SOD_BLIS_DISPLAY_SOD_LCA_CTA_DOA_UNCALIBRATION', 4: 'SOD_BLIS_DISPLAY_TEMPORARY_ERROR', 5: 'SOD_BLIS_DISPLAY_RESERVED', 6: 'SOD_BLIS_DISPLAY_RESERVED', 7: 'SOD_BLIS_DISPLAY_RESERVED', 8: 'SOD_BLIS_DISPLAY_RESERVED', 9: 'SOD_BLIS_DISPLAY_RESERVED', 10: 'SOD_BLIS_DISPLAY_RESERVED', 11: 'SOD_BLIS_DISPLAY_RESERVED', 12: 'SOD_BLIS_DISPLAY_RESERVED', 13: 'SOD_BLIS_DISPLAY_RESERVED', 14: 'SOD_BLIS_DISPLAY_RESERVED', 15: 'SOD_BLIS_DISPLAY_RESERVED'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_blis_displayType Sod0x275275::sod_blis_display(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 4);

  Sod_0x275_275::Sod_blis_displayType ret =  static_cast<Sod_0x275_275::Sod_blis_displayType>(x);
  return ret;
}

// config detail: {'name': 'sod_lca_warningreqleft', 'enum': {0: 'SOD_LCA_WARNINGREQLEFT_NO_WARNING', 1: 'SOD_LCA_WARNINGREQLEFT_WARNING_LEVEL_1', 2: 'SOD_LCA_WARNINGREQLEFT_WARNING_LEVEL_2', 3: 'SOD_LCA_WARNINGREQLEFT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_lca_warningreqleftType Sod0x275275::sod_lca_warningreqleft(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 2);

  Sod_0x275_275::Sod_lca_warningreqleftType ret =  static_cast<Sod_0x275_275::Sod_lca_warningreqleftType>(x);
  return ret;
}

// config detail: {'name': 'sod_doa_warningreqleftrear', 'enum': {0: 'SOD_DOA_WARNINGREQLEFTREAR_NO_WARNING', 1: 'SOD_DOA_WARNINGREQLEFTREAR_WARNING_LEVEL_1', 2: 'SOD_DOA_WARNINGREQLEFTREAR_WARNING_LEVEL_2', 3: 'SOD_DOA_WARNINGREQLEFTREAR_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_doa_warningreqleftrearType Sod0x275275::sod_doa_warningreqleftrear(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 2);

  Sod_0x275_275::Sod_doa_warningreqleftrearType ret =  static_cast<Sod_0x275_275::Sod_doa_warningreqleftrearType>(x);
  return ret;
}

// config detail: {'name': 'sod_doa_warningreqleftfront', 'enum': {0: 'SOD_DOA_WARNINGREQLEFTFRONT_NO_WARNING', 1: 'SOD_DOA_WARNINGREQLEFTFRONT_WARNING_LEVEL_1', 2: 'SOD_DOA_WARNINGREQLEFTFRONT_WARNING_LEVEL_2', 3: 'SOD_DOA_WARNINGREQLEFTFRONT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_doa_warningreqleftfrontType Sod0x275275::sod_doa_warningreqleftfront(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 2);

  Sod_0x275_275::Sod_doa_warningreqleftfrontType ret =  static_cast<Sod_0x275_275::Sod_doa_warningreqleftfrontType>(x);
  return ret;
}

// config detail: {'name': 'sod_cta_warningreqleft', 'enum': {0: 'SOD_CTA_WARNINGREQLEFT_NO_WARNING', 1: 'SOD_CTA_WARNINGREQLEFT_RESERVED', 2: 'SOD_CTA_WARNINGREQLEFT_WARNING_LEVEL_2', 3: 'SOD_CTA_WARNINGREQLEFT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_cta_warningreqleftType Sod0x275275::sod_cta_warningreqleft(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 2);

  Sod_0x275_275::Sod_cta_warningreqleftType ret =  static_cast<Sod_0x275_275::Sod_cta_warningreqleftType>(x);
  return ret;
}

// config detail: {'name': 'sod_doa_state', 'enum': {0: 'SOD_DOA_STATE_INACTIVE', 1: 'SOD_DOA_STATE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_doa_stateType Sod0x275275::sod_doa_state(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  Sod_0x275_275::Sod_doa_stateType ret =  static_cast<Sod_0x275_275::Sod_doa_stateType>(x);
  return ret;
}

// config detail: {'name': 'sod_cta_state', 'enum': {0: 'SOD_CTA_STATE_INACTIVE', 1: 'SOD_CTA_STATE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_cta_stateType Sod0x275275::sod_cta_state(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 1);

  Sod_0x275_275::Sod_cta_stateType ret =  static_cast<Sod_0x275_275::Sod_cta_stateType>(x);
  return ret;
}

// config detail: {'name': 'sod_sodlca_state', 'enum': {0: 'SOD_SODLCA_STATE_INACTIVE', 1: 'SOD_SODLCA_STATE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_sodlca_stateType Sod0x275275::sod_sodlca_state(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 1);

  Sod_0x275_275::Sod_sodlca_stateType ret =  static_cast<Sod_0x275_275::Sod_sodlca_stateType>(x);
  return ret;
}

// config detail: {'name': 'doa_offtelltale', 'enum': {0: 'DOA_OFFTELLTALE_OFFTELLTLAE_OFF', 1: 'DOA_OFFTELLTALE_OFFTELLTALE_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Doa_offtelltaleType Sod0x275275::doa_offtelltale(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  Sod_0x275_275::Doa_offtelltaleType ret =  static_cast<Sod_0x275_275::Doa_offtelltaleType>(x);
  return ret;
}

// config detail: {'name': 'sod_doa_warningreqrightrear', 'enum': {0: 'SOD_DOA_WARNINGREQRIGHTREAR_NO_WARNING', 1: 'SOD_DOA_WARNINGREQRIGHTREAR_WARNING_LEVEL_1', 2: 'SOD_DOA_WARNINGREQRIGHTREAR_WARNING_LEVEL_2', 3: 'SOD_DOA_WARNINGREQRIGHTREAR_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_doa_warningreqrightrearType Sod0x275275::sod_doa_warningreqrightrear(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 2);

  Sod_0x275_275::Sod_doa_warningreqrightrearType ret =  static_cast<Sod_0x275_275::Sod_doa_warningreqrightrearType>(x);
  return ret;
}

// config detail: {'name': 'cta_offtelltale', 'enum': {0: 'CTA_OFFTELLTALE_OFFTELLTLAE_OFF', 1: 'CTA_OFFTELLTALE_OFFTELLTALE_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 30, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Cta_offtelltaleType Sod0x275275::cta_offtelltale(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(6, 1);

  Sod_0x275_275::Cta_offtelltaleType ret =  static_cast<Sod_0x275_275::Cta_offtelltaleType>(x);
  return ret;
}

// config detail: {'name': 'sodlca_offtelltale', 'enum': {0: 'SODLCA_OFFTELLTALE_OFFTELLTLAE_OFF', 1: 'SODLCA_OFFTELLTALE_OFFTELLTALE_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sodlca_offtelltaleType Sod0x275275::sodlca_offtelltale(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(7, 1);

  Sod_0x275_275::Sodlca_offtelltaleType ret =  static_cast<Sod_0x275_275::Sodlca_offtelltaleType>(x);
  return ret;
}

// config detail: {'name': 'sod_doa_warningreqrightfront', 'enum': {0: 'SOD_DOA_WARNINGREQRIGHTFRONT_NO_WARNING', 1: 'SOD_DOA_WARNINGREQRIGHTFRONT_WARNING_LEVEL_1', 2: 'SOD_DOA_WARNINGREQRIGHTFRONT_WARNING_LEVEL_2', 3: 'SOD_DOA_WARNINGREQRIGHTFRONT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_doa_warningreqrightfrontType Sod0x275275::sod_doa_warningreqrightfront(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 2);

  Sod_0x275_275::Sod_doa_warningreqrightfrontType ret =  static_cast<Sod_0x275_275::Sod_doa_warningreqrightfrontType>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x275', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Sod0x275275::rolling_counter_0x275(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'checksum_0x275', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Sod0x275275::checksum_0x275(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'sod_cta_warningreqright', 'enum': {0: 'SOD_CTA_WARNINGREQRIGHT_NO_WARNING', 1: 'SOD_CTA_WARNINGREQRIGHT_RESERVED', 2: 'SOD_CTA_WARNINGREQRIGHT_WARNING_LEVEL_2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_cta_warningreqrightType Sod0x275275::sod_cta_warningreqright(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Sod_0x275_275::Sod_cta_warningreqrightType ret =  static_cast<Sod_0x275_275::Sod_cta_warningreqrightType>(x);
  return ret;
}

// config detail: {'name': 'sod_autoalignmentflag', 'enum': {0: 'SOD_AUTOALIGNMENTFLAG_NO_SUCCESS', 1: 'SOD_AUTOALIGNMENTFLAG_SUCCESSFUL', 3: 'SOD_AUTOALIGNMENTFLAG_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Sod_0x275_275::Sod_autoalignmentflagType Sod0x275275::sod_autoalignmentflag(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Sod_0x275_275::Sod_autoalignmentflagType ret =  static_cast<Sod_0x275_275::Sod_autoalignmentflagType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
