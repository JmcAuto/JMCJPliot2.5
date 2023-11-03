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

#include "modules/canbus/vehicle/cx75/protocol/gw_mp5_0x530_530.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwmp50x530530::Gwmp50x530530() {}
const int32_t Gwmp50x530530::ID = 0x530;

void Gwmp50x530530::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_fcw_sensitive(mp5_fcw_sensitive(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_icmenushift_button(mp5_icmenushift_button(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_aeb_on_off(mp5_aeb_on_off(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_fcw_on_off(mp5_fcw_on_off(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_bsdswitchsts(mp5_bsdswitchsts(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_afs_swtichsts(mp5_afs_swtichsts(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_laneassittypereq(mp5_laneassittypereq(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_icmenuactive_button(mp5_icmenuactive_button(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_alarmstatus(mp5_alarmstatus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_alarmornot(mp5_alarmornot(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_paakreset_req(mp5_paakreset_req(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_cta_active(mp5_cta_active(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_bsdlca_active(mp5_bsdlca_active(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_tsrmenureq(mp5_tsrmenureq(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_ldwwarning(mp5_ldwwarning(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_ldwsensitvity(mp5_ldwsensitvity(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_avm_angle(mp5_avm_angle(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_avm_rotationchange(mp5_avm_rotationchange(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_ihcmenureq(mp5_ihcmenureq(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_doa_active(mp5_doa_active(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_apa_available_sts(mp5_apa_available_sts(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_tpmsautolocationsetting(mp5_tpmsautolocationsetting(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_avm_rotationdirection(mp5_avm_rotationdirection(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_avm_button(mp5_avm_button(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_0x530_530()->set_mp5_view_choice(mp5_view_choice(bytes, length));
}

// config detail: {'name': 'mp5_fcw_sensitive', 'enum': {0: 'MP5_FCW_SENSITIVE_UNAVAILABLE', 1: 'MP5_FCW_SENSITIVE_LEVEL1_LOW_SENSITIVE', 2: 'MP5_FCW_SENSITIVE_LEVEL2_NORMAL', 3: 'MP5_FCW_SENSITIVE_LEVEL3_HIGH_SENSITIVE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_fcw_sensitiveType Gwmp50x530530::mp5_fcw_sensitive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Gw_mp5_0x530_530::Mp5_fcw_sensitiveType ret =  static_cast<Gw_mp5_0x530_530::Mp5_fcw_sensitiveType>(x);
  return ret;
}

// config detail: {'name': 'mp5_icmenushift_button', 'enum': {0: 'MP5_ICMENUSHIFT_BUTTON_NO_EVENT', 1: 'MP5_ICMENUSHIFT_BUTTON_PAGE_UP', 2: 'MP5_ICMENUSHIFT_BUTTON_PAGE_DOWN', 3: 'MP5_ICMENUSHIFT_BUTTON_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_icmenushift_buttonType Gwmp50x530530::mp5_icmenushift_button(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  Gw_mp5_0x530_530::Mp5_icmenushift_buttonType ret =  static_cast<Gw_mp5_0x530_530::Mp5_icmenushift_buttonType>(x);
  return ret;
}

// config detail: {'name': 'mp5_aeb_on_off', 'enum': {0: 'MP5_AEB_ON_OFF_SWITCH_ON', 1: 'MP5_AEB_ON_OFF_SWITCH_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_aeb_on_offType Gwmp50x530530::mp5_aeb_on_off(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  Gw_mp5_0x530_530::Mp5_aeb_on_offType ret =  static_cast<Gw_mp5_0x530_530::Mp5_aeb_on_offType>(x);
  return ret;
}

// config detail: {'name': 'mp5_fcw_on_off', 'enum': {0: 'MP5_FCW_ON_OFF_SWITCH_ON', 1: 'MP5_FCW_ON_OFF_SWITCH_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_fcw_on_offType Gwmp50x530530::mp5_fcw_on_off(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  Gw_mp5_0x530_530::Mp5_fcw_on_offType ret =  static_cast<Gw_mp5_0x530_530::Mp5_fcw_on_offType>(x);
  return ret;
}

// config detail: {'name': 'mp5_bsdswitchsts', 'enum': {0: 'MP5_BSDSWITCHSTS_ON', 1: 'MP5_BSDSWITCHSTS_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_bsdswitchstsType Gwmp50x530530::mp5_bsdswitchsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Gw_mp5_0x530_530::Mp5_bsdswitchstsType ret =  static_cast<Gw_mp5_0x530_530::Mp5_bsdswitchstsType>(x);
  return ret;
}

// config detail: {'name': 'mp5_afs_swtichsts', 'enum': {0: 'MP5_AFS_SWTICHSTS_NO_PRESS', 1: 'MP5_AFS_SWTICHSTS_PRESS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_afs_swtichstsType Gwmp50x530530::mp5_afs_swtichsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Gw_mp5_0x530_530::Mp5_afs_swtichstsType ret =  static_cast<Gw_mp5_0x530_530::Mp5_afs_swtichstsType>(x);
  return ret;
}

// config detail: {'name': 'mp5_laneassittypereq', 'enum': {0: 'MP5_LANEASSITTYPEREQ_NO_SELECTION', 1: 'MP5_LANEASSITTYPEREQ_LDW', 2: 'MP5_LANEASSITTYPEREQ_LKA', 3: 'MP5_LANEASSITTYPEREQ_LDW_LKA'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_laneassittypereqType Gwmp50x530530::mp5_laneassittypereq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 2);

  Gw_mp5_0x530_530::Mp5_laneassittypereqType ret =  static_cast<Gw_mp5_0x530_530::Mp5_laneassittypereqType>(x);
  return ret;
}

// config detail: {'name': 'mp5_icmenuactive_button', 'enum': {0: 'MP5_ICMENUACTIVE_BUTTON_OFF', 1: 'MP5_ICMENUACTIVE_BUTTON_MENU_SWITCH_ACTIVE', 2: 'MP5_ICMENUACTIVE_BUTTON_SILENCE', 3: 'MP5_ICMENUACTIVE_BUTTON_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_icmenuactive_buttonType Gwmp50x530530::mp5_icmenuactive_button(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 2);

  Gw_mp5_0x530_530::Mp5_icmenuactive_buttonType ret =  static_cast<Gw_mp5_0x530_530::Mp5_icmenuactive_buttonType>(x);
  return ret;
}

// config detail: {'name': 'mp5_alarmstatus', 'enum': {0: 'MP5_ALARMSTATUS_INACTIVE', 1: 'MP5_ALARMSTATUS_ACTIVE', 2: 'MP5_ALARMSTATUS_FAILED', 3: 'MP5_ALARMSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_alarmstatusType Gwmp50x530530::mp5_alarmstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 2);

  Gw_mp5_0x530_530::Mp5_alarmstatusType ret =  static_cast<Gw_mp5_0x530_530::Mp5_alarmstatusType>(x);
  return ret;
}

// config detail: {'name': 'mp5_alarmornot', 'enum': {0: 'MP5_ALARMORNOT_NO_ALARM', 1: 'MP5_ALARMORNOT_ALARM'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_alarmornotType Gwmp50x530530::mp5_alarmornot(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  Gw_mp5_0x530_530::Mp5_alarmornotType ret =  static_cast<Gw_mp5_0x530_530::Mp5_alarmornotType>(x);
  return ret;
}

// config detail: {'name': 'mp5_paakreset_req', 'enum': {0: 'MP5_PAAKRESET_REQ_NO_REQUEST', 1: 'MP5_PAAKRESET_REQ_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_paakreset_reqType Gwmp50x530530::mp5_paakreset_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(7, 1);

  Gw_mp5_0x530_530::Mp5_paakreset_reqType ret =  static_cast<Gw_mp5_0x530_530::Mp5_paakreset_reqType>(x);
  return ret;
}

// config detail: {'name': 'mp5_cta_active', 'enum': {0: 'MP5_CTA_ACTIVE_ON', 1: 'MP5_CTA_ACTIVE_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_cta_activeType Gwmp50x530530::mp5_cta_active(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  Gw_mp5_0x530_530::Mp5_cta_activeType ret =  static_cast<Gw_mp5_0x530_530::Mp5_cta_activeType>(x);
  return ret;
}

// config detail: {'name': 'mp5_bsdlca_active', 'enum': {0: 'MP5_BSDLCA_ACTIVE_ON', 1: 'MP5_BSDLCA_ACTIVE_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_bsdlca_activeType Gwmp50x530530::mp5_bsdlca_active(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 1);

  Gw_mp5_0x530_530::Mp5_bsdlca_activeType ret =  static_cast<Gw_mp5_0x530_530::Mp5_bsdlca_activeType>(x);
  return ret;
}

// config detail: {'name': 'mp5_tsrmenureq', 'enum': {0: 'MP5_TSRMENUREQ_ON', 1: 'MP5_TSRMENUREQ_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 43, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_tsrmenureqType Gwmp50x530530::mp5_tsrmenureq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(3, 1);

  Gw_mp5_0x530_530::Mp5_tsrmenureqType ret =  static_cast<Gw_mp5_0x530_530::Mp5_tsrmenureqType>(x);
  return ret;
}

// config detail: {'name': 'mp5_ldwwarning', 'enum': {0: 'MP5_LDWWARNING_AUDIBLE_WARNING_AND_WHEELSTEER_SHAKE', 1: 'MP5_LDWWARNING_AUDIBLE_WARNING', 2: 'MP5_LDWWARNING_WHEELSTEER_SHAKE', 3: 'MP5_LDWWARNING_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_ldwwarningType Gwmp50x530530::mp5_ldwwarning(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 2);

  Gw_mp5_0x530_530::Mp5_ldwwarningType ret =  static_cast<Gw_mp5_0x530_530::Mp5_ldwwarningType>(x);
  return ret;
}

// config detail: {'name': 'mp5_ldwsensitvity', 'enum': {0: 'MP5_LDWSENSITVITY_HIGH', 1: 'MP5_LDWSENSITVITY_LOW', 2: 'MP5_LDWSENSITVITY_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_ldwsensitvityType Gwmp50x530530::mp5_ldwsensitvity(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 2);

  Gw_mp5_0x530_530::Mp5_ldwsensitvityType ret =  static_cast<Gw_mp5_0x530_530::Mp5_ldwsensitvityType>(x);
  return ret;
}

// config detail: {'name': 'mp5_avm_angle', 'enum': {0: 'MP5_AVM_ANGLE_CENTER_KEY', 1: 'MP5_AVM_ANGLE_FRONT_3D_FRONT_AVM_2D', 2: 'MP5_AVM_ANGLE_FRONT_RIGHT_3D', 3: 'MP5_AVM_ANGLE_RIGHT_3D_RIGHT_AVM_2D', 4: 'MP5_AVM_ANGLE_REAR_RIGHT_3D', 5: 'MP5_AVM_ANGLE_REAR_3D_REAR_AVM_2D', 6: 'MP5_AVM_ANGLE_LEFT_REAR_3D', 7: 'MP5_AVM_ANGLE_LEFT_3D_LEFT_AVM_2D', 8: 'MP5_AVM_ANGLE_LEFT_FRONT_3D', 9: 'MP5_AVM_ANGLE_3D_KEY', 10: 'MP5_AVM_ANGLE_2D_KEY', 11: 'MP5_AVM_ANGLE_NO_VIDEO_SIGNAL_RESERVED', 12: 'MP5_AVM_ANGLE_RESERVED', 13: 'MP5_AVM_ANGLE_RESERVED', 14: 'MP5_AVM_ANGLE_RESERVED', 15: 'MP5_AVM_ANGLE_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_avm_angleType Gwmp50x530530::mp5_avm_angle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 4);

  Gw_mp5_0x530_530::Mp5_avm_angleType ret =  static_cast<Gw_mp5_0x530_530::Mp5_avm_angleType>(x);
  return ret;
}

// config detail: {'name': 'mp5_avm_rotationchange', 'offset': 0.0, 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'physical_range': '[0|63]', 'bit': 52, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg'}
int Gwmp50x530530::mp5_avm_rotationchange(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 5);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'mp5_ihcmenureq', 'enum': {0: 'MP5_IHCMENUREQ_ON', 1: 'MP5_IHCMENUREQ_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_ihcmenureqType Gwmp50x530530::mp5_ihcmenureq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Gw_mp5_0x530_530::Mp5_ihcmenureqType ret =  static_cast<Gw_mp5_0x530_530::Mp5_ihcmenureqType>(x);
  return ret;
}

// config detail: {'name': 'mp5_doa_active', 'enum': {0: 'MP5_DOA_ACTIVE_ON', 1: 'MP5_DOA_ACTIVE_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_doa_activeType Gwmp50x530530::mp5_doa_active(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  Gw_mp5_0x530_530::Mp5_doa_activeType ret =  static_cast<Gw_mp5_0x530_530::Mp5_doa_activeType>(x);
  return ret;
}

// config detail: {'name': 'mp5_apa_available_sts', 'enum': {0: 'MP5_APA_AVAILABLE_STS_INITIAL', 1: 'MP5_APA_AVAILABLE_STS_AVALIBLE', 2: 'MP5_APA_AVAILABLE_STS_NOT_AVALIBLE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 57, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_apa_available_stsType Gwmp50x530530::mp5_apa_available_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 2);

  Gw_mp5_0x530_530::Mp5_apa_available_stsType ret =  static_cast<Gw_mp5_0x530_530::Mp5_apa_available_stsType>(x);
  return ret;
}

// config detail: {'name': 'mp5_tpmsautolocationsetting', 'enum': {0: 'MP5_TPMSAUTOLOCATIONSETTING_AUTOLOCATIONSETTINGOFF', 1: 'MP5_TPMSAUTOLOCATIONSETTING_AUTOLOCATIONSETTINGON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 58, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_tpmsautolocationsettingType Gwmp50x530530::mp5_tpmsautolocationsetting(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(2, 1);

  Gw_mp5_0x530_530::Mp5_tpmsautolocationsettingType ret =  static_cast<Gw_mp5_0x530_530::Mp5_tpmsautolocationsettingType>(x);
  return ret;
}

// config detail: {'name': 'mp5_avm_rotationdirection', 'enum': {0: 'MP5_AVM_ROTATIONDIRECTION_INITIAL', 1: 'MP5_AVM_ROTATIONDIRECTION_CLOCKWISE', 2: 'MP5_AVM_ROTATIONDIRECTION_ANTICLOCKWISE', 3: 'MP5_AVM_ROTATIONDIRECTION_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 62, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_avm_rotationdirectionType Gwmp50x530530::mp5_avm_rotationdirection(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(5, 2);

  Gw_mp5_0x530_530::Mp5_avm_rotationdirectionType ret =  static_cast<Gw_mp5_0x530_530::Mp5_avm_rotationdirectionType>(x);
  return ret;
}

// config detail: {'name': 'mp5_avm_button', 'enum': {0: 'MP5_AVM_BUTTON_INIT', 1: 'MP5_AVM_BUTTON_START_UP', 2: 'MP5_AVM_BUTTON_SHUT_DOWN', 3: 'MP5_AVM_BUTTON_FAULTURE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_avm_buttonType Gwmp50x530530::mp5_avm_button(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Gw_mp5_0x530_530::Mp5_avm_buttonType ret =  static_cast<Gw_mp5_0x530_530::Mp5_avm_buttonType>(x);
  return ret;
}

// config detail: {'name': 'mp5_view_choice', 'enum': {0: 'MP5_VIEW_CHOICE_AVM', 1: 'MP5_VIEW_CHOICE_ADAS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_0x530_530::Mp5_view_choiceType Gwmp50x530530::mp5_view_choice(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  Gw_mp5_0x530_530::Mp5_view_choiceType ret =  static_cast<Gw_mp5_0x530_530::Mp5_view_choiceType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
