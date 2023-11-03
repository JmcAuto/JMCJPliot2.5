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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_MP5_0X530_530_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_MP5_0X530_530_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwmp50x530530 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwmp50x530530();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'MP5_FCW_Sensitive', 'enum': {0: 'MP5_FCW_SENSITIVE_UNAVAILABLE', 1: 'MP5_FCW_SENSITIVE_LEVEL1_LOW_SENSITIVE', 2: 'MP5_FCW_SENSITIVE_LEVEL2_NORMAL', 3: 'MP5_FCW_SENSITIVE_LEVEL3_HIGH_SENSITIVE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_fcw_sensitiveType mp5_fcw_sensitive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_ICMenuShift_Button', 'enum': {0: 'MP5_ICMENUSHIFT_BUTTON_NO_EVENT', 1: 'MP5_ICMENUSHIFT_BUTTON_PAGE_UP', 2: 'MP5_ICMENUSHIFT_BUTTON_PAGE_DOWN', 3: 'MP5_ICMENUSHIFT_BUTTON_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_icmenushift_buttonType mp5_icmenushift_button(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_AEB_ON_OFF', 'enum': {0: 'MP5_AEB_ON_OFF_SWITCH_ON', 1: 'MP5_AEB_ON_OFF_SWITCH_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_aeb_on_offType mp5_aeb_on_off(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_FCW_ON_OFF', 'enum': {0: 'MP5_FCW_ON_OFF_SWITCH_ON', 1: 'MP5_FCW_ON_OFF_SWITCH_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_fcw_on_offType mp5_fcw_on_off(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_BSDSwitchSts', 'enum': {0: 'MP5_BSDSWITCHSTS_ON', 1: 'MP5_BSDSWITCHSTS_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_bsdswitchstsType mp5_bsdswitchsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_AFS_SwtichSts', 'enum': {0: 'MP5_AFS_SWTICHSTS_NO_PRESS', 1: 'MP5_AFS_SWTICHSTS_PRESS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_afs_swtichstsType mp5_afs_swtichsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_LaneAssitTypeReq', 'enum': {0: 'MP5_LANEASSITTYPEREQ_NO_SELECTION', 1: 'MP5_LANEASSITTYPEREQ_LDW', 2: 'MP5_LANEASSITTYPEREQ_LKA', 3: 'MP5_LANEASSITTYPEREQ_LDW_LKA'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_laneassittypereqType mp5_laneassittypereq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_ICMenuActive_Button', 'enum': {0: 'MP5_ICMENUACTIVE_BUTTON_OFF', 1: 'MP5_ICMENUACTIVE_BUTTON_MENU_SWITCH_ACTIVE', 2: 'MP5_ICMENUACTIVE_BUTTON_SILENCE', 3: 'MP5_ICMENUACTIVE_BUTTON_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_icmenuactive_buttonType mp5_icmenuactive_button(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_AlarmStatus', 'enum': {0: 'MP5_ALARMSTATUS_INACTIVE', 1: 'MP5_ALARMSTATUS_ACTIVE', 2: 'MP5_ALARMSTATUS_FAILED', 3: 'MP5_ALARMSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_alarmstatusType mp5_alarmstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_AlarmOrNot', 'enum': {0: 'MP5_ALARMORNOT_NO_ALARM', 1: 'MP5_ALARMORNOT_ALARM'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_alarmornotType mp5_alarmornot(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_PaakReset_Req', 'enum': {0: 'MP5_PAAKRESET_REQ_NO_REQUEST', 1: 'MP5_PAAKRESET_REQ_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_paakreset_reqType mp5_paakreset_req(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_CTA_Active', 'enum': {0: 'MP5_CTA_ACTIVE_ON', 1: 'MP5_CTA_ACTIVE_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 40, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_cta_activeType mp5_cta_active(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_BSDLCA_Active', 'enum': {0: 'MP5_BSDLCA_ACTIVE_ON', 1: 'MP5_BSDLCA_ACTIVE_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_bsdlca_activeType mp5_bsdlca_active(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_TSRMenuReq', 'enum': {0: 'MP5_TSRMENUREQ_ON', 1: 'MP5_TSRMENUREQ_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 43, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_tsrmenureqType mp5_tsrmenureq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_LDWWarning', 'enum': {0: 'MP5_LDWWARNING_AUDIBLE_WARNING_AND_WHEELSTEER_SHAKE', 1: 'MP5_LDWWARNING_AUDIBLE_WARNING', 2: 'MP5_LDWWARNING_WHEELSTEER_SHAKE', 3: 'MP5_LDWWARNING_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_ldwwarningType mp5_ldwwarning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_LDWSensitvity', 'enum': {0: 'MP5_LDWSENSITVITY_HIGH', 1: 'MP5_LDWSENSITVITY_LOW', 2: 'MP5_LDWSENSITVITY_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_ldwsensitvityType mp5_ldwsensitvity(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_AVM_Angle', 'enum': {0: 'MP5_AVM_ANGLE_CENTER_KEY', 1: 'MP5_AVM_ANGLE_FRONT_3D_FRONT_AVM_2D', 2: 'MP5_AVM_ANGLE_FRONT_RIGHT_3D', 3: 'MP5_AVM_ANGLE_RIGHT_3D_RIGHT_AVM_2D', 4: 'MP5_AVM_ANGLE_REAR_RIGHT_3D', 5: 'MP5_AVM_ANGLE_REAR_3D_REAR_AVM_2D', 6: 'MP5_AVM_ANGLE_LEFT_REAR_3D', 7: 'MP5_AVM_ANGLE_LEFT_3D_LEFT_AVM_2D', 8: 'MP5_AVM_ANGLE_LEFT_FRONT_3D', 9: 'MP5_AVM_ANGLE_3D_KEY', 10: 'MP5_AVM_ANGLE_2D_KEY', 11: 'MP5_AVM_ANGLE_NO_VIDEO_SIGNAL_RESERVED', 12: 'MP5_AVM_ANGLE_RESERVED', 13: 'MP5_AVM_ANGLE_RESERVED', 14: 'MP5_AVM_ANGLE_RESERVED', 15: 'MP5_AVM_ANGLE_INVALID'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_avm_angleType mp5_avm_angle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_AVM_RotationChange', 'offset': 0.0, 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'physical_range': '[0|63]', 'bit': 52, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg'}
  int mp5_avm_rotationchange(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_IHCMenuReq', 'enum': {0: 'MP5_IHCMENUREQ_ON', 1: 'MP5_IHCMENUREQ_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_ihcmenureqType mp5_ihcmenureq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_DOA_Active', 'enum': {0: 'MP5_DOA_ACTIVE_ON', 1: 'MP5_DOA_ACTIVE_OFF'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_doa_activeType mp5_doa_active(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_APA_Available_Sts', 'enum': {0: 'MP5_APA_AVAILABLE_STS_INITIAL', 1: 'MP5_APA_AVAILABLE_STS_AVALIBLE', 2: 'MP5_APA_AVAILABLE_STS_NOT_AVALIBLE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 57, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_apa_available_stsType mp5_apa_available_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_TPMSAutoLocationSetting', 'enum': {0: 'MP5_TPMSAUTOLOCATIONSETTING_AUTOLOCATIONSETTINGOFF', 1: 'MP5_TPMSAUTOLOCATIONSETTING_AUTOLOCATIONSETTINGON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 58, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_tpmsautolocationsettingType mp5_tpmsautolocationsetting(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_AVM_RotationDirection', 'enum': {0: 'MP5_AVM_ROTATIONDIRECTION_INITIAL', 1: 'MP5_AVM_ROTATIONDIRECTION_CLOCKWISE', 2: 'MP5_AVM_ROTATIONDIRECTION_ANTICLOCKWISE', 3: 'MP5_AVM_ROTATIONDIRECTION_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 62, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_avm_rotationdirectionType mp5_avm_rotationdirection(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_AVM_Button', 'enum': {0: 'MP5_AVM_BUTTON_INIT', 1: 'MP5_AVM_BUTTON_START_UP', 2: 'MP5_AVM_BUTTON_SHUT_DOWN', 3: 'MP5_AVM_BUTTON_FAULTURE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_avm_buttonType mp5_avm_button(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_View_Choice', 'enum': {0: 'MP5_VIEW_CHOICE_AVM', 1: 'MP5_VIEW_CHOICE_ADAS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_0x530_530::Mp5_view_choiceType mp5_view_choice(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_MP5_0X530_530_H_
