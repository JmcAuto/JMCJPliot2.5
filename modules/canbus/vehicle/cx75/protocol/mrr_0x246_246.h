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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_MRR_0X246_246_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_MRR_0X246_246_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Mrr0x246246 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Mrr0x246246();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'TauGapSet', 'enum': {0: 'TAUGAPSET_TAUGAP_0', 1: 'TAUGAPSET_TAUGAP_1', 2: 'TAUGAPSET_TAUGAP_2', 3: 'TAUGAPSET_TAUGAP_3', 4: 'TAUGAPSET_TAUGAP_4', 5: 'TAUGAPSET_TAUGAP_5', 6: 'TAUGAPSET_TAUGAP_6', 7: 'TAUGAPSET_TAUGAP_7'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_taugapset(Mrr_0x246_246::TaugapsetType taugapset);

  // config detail: {'name': 'dxTarObj', 'enum': {0: 'DXTAROBJ_DISTANCE_0', 1: 'DXTAROBJ_DISTANCE_1', 2: 'DXTAROBJ_DISTANCE_2', 3: 'DXTAROBJ_DISTANCE_3', 4: 'DXTAROBJ_DISTANCE_4', 5: 'DXTAROBJ_DISTANCE_5', 6: 'DXTAROBJ_DISTANCE_6', 7: 'DXTAROBJ_DISTANCE_7'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_dxtarobj(Mrr_0x246_246::DxtarobjType dxtarobj);

  // config detail: {'name': 'ACCHMI_Mode', 'enum': {0: 'ACCHMI_MODE_OFF_MODE', 1: 'ACCHMI_MODE_PASSIVE_MODE', 2: 'ACCHMI_MODE_STAND_BY_MODE', 3: 'ACCHMI_MODE_ACTIVE_CONTROL_MODE', 4: 'ACCHMI_MODE_BRAKE_ONLY_MODE', 5: 'ACCHMI_MODE_OVERRIDE', 6: 'ACCHMI_MODE_STANDSTILL', 7: 'ACCHMI_MODE_FAILURE_MODE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_acchmi_mode(Mrr_0x246_246::Acchmi_modeType acchmi_mode);

  // config detail: {'name': 'ACCFailInfo', 'enum': {0: 'ACCFAILINFO_NO_ERROR', 1: 'ACCFAILINFO_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_accfailinfo(Mrr_0x246_246::AccfailinfoType accfailinfo);

  // config detail: {'name': 'TakeOverReq', 'enum': {0: 'TAKEOVERREQ_NO_TAKEOVER_REQUEST', 1: 'TAKEOVERREQ_VALID_TAKEOVER_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_takeoverreq(Mrr_0x246_246::TakeoverreqType takeoverreq);

  // config detail: {'name': 'MRR_FCW_Sensitive', 'enum': {0: 'MRR_FCW_SENSITIVE_UNAVAILABLE', 1: 'MRR_FCW_SENSITIVE_LEVEL1_LOW_SENSITIVE', 2: 'MRR_FCW_SENSITIVE_LEVEL2_NORMAL', 3: 'MRR_FCW_SENSITIVE_LEVEL3_HIGH_SENSITIVE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_mrr_fcw_sensitive(Mrr_0x246_246::Mrr_fcw_sensitiveType mrr_fcw_sensitive);

  // config detail: {'name': 'AEB_STATE', 'enum': {0: 'AEB_STATE_UNAVAILABLE', 1: 'AEB_STATE_OFF', 2: 'AEB_STATE_STANDBY', 3: 'AEB_STATE_ACTIVE_NO_INTERVENTION', 4: 'AEB_STATE_ACTIVE', 5: 'AEB_STATE_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_aeb_state(Mrr_0x246_246::Aeb_stateType aeb_state);

  // config detail: {'name': 'ACC_Startstop_infor', 'enum': {0: 'ACC_STARTSTOP_INFOR_ACC_STOPALLOWED', 1: 'ACC_STARTSTOP_INFOR_ACC_STOPFORBIDDEN', 2: 'ACC_STARTSTOP_INFOR_ACC_STARTREQUEST', 3: 'ACC_STARTSTOP_INFOR_ACC_SYSTEMFAILURE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_acc_startstop_infor(Mrr_0x246_246::Acc_startstop_inforType acc_startstop_infor);

  // config detail: {'name': 'FCW_preWarning', 'enum': {0: 'FCW_PREWARNING_NO_WARNING', 1: 'FCW_PREWARNING_WARNING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 30, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_fcw_prewarning(Mrr_0x246_246::Fcw_prewarningType fcw_prewarning);

  // config detail: {'name': 'FCW_latentWarning', 'enum': {0: 'FCW_LATENTWARNING_NO_WARNING', 1: 'FCW_LATENTWARNING_WARNING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_fcw_latentwarning(Mrr_0x246_246::Fcw_latentwarningType fcw_latentwarning);

  // config detail: {'name': 'FCW_STATE', 'enum': {0: 'FCW_STATE_UNAVAILABLE', 1: 'FCW_STATE_OFF', 2: 'FCW_STATE_STANDBY', 3: 'FCW_STATE_ACTIVE_NO_INTERVENTION', 4: 'FCW_STATE_ACTIVE', 5: 'FCW_STATE_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_fcw_state(Mrr_0x246_246::Fcw_stateType fcw_state);

  // config detail: {'name': 'Obj_Speed', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_obj_speed(int obj_speed);

  // config detail: {'name': 'Rolling_counter_0x246', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_rolling_counter_0x246(int rolling_counter_0x246);

  // config detail: {'name': 'Textinfo', 'enum': {0: 'TEXTINFO_NO_DISPLAY', 1: 'TEXTINFO_ACC_IS_SWITCHED_ON', 2: 'TEXTINFO_ACC_IS_SWITCHED_OFF', 3: 'TEXTINFO_ACC_IS_CANCELLED', 4: 'TEXTINFO_ACC_ACTIVE', 5: 'TEXTINFO_MRR_BLINDNESS', 6: 'TEXTINFO_ACC_AND_PEBS_ERROR', 7: 'TEXTINFO_EPB_ACTIVATE', 8: 'TEXTINFO_NO_FORWARD_GEAR', 9: 'TEXTINFO_SEATBELT_UNBUCKLED', 10: 'TEXTINFO_ESP_OFF', 11: 'TEXTINFO_SPEED_OVER_150KPH', 12: 'TEXTINFO_DOOR_OPEN', 13: 'TEXTINFO_OVERRIDE', 14: 'TEXTINFO_ESP_ERROR', 15: 'TEXTINFO_UNCALIBRATED'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_textinfo(Mrr_0x246_246::TextinfoType textinfo);

  // config detail: {'name': 'Checksum_0x246', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_checksum_0x246(int checksum_0x246);

  // config detail: {'name': 'vSetDis', 'enum': {511: 'VSETDIS_INVALID'}, 'precision': 0.5, 'len': 9, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255.5]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'kph'}
  Mrr0x246246* set_vsetdis(Mrr_0x246_246::VsetdisType vsetdis);

  // config detail: {'name': 'ObjValid', 'enum': {0: 'OBJVALID_NO_OBJECT', 1: 'OBJVALID_TARGET_OBJECT_DETECTED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Mrr0x246246* set_objvalid(Mrr_0x246_246::ObjvalidType objvalid);

 private:

  // config detail: {'name': 'TauGapSet', 'enum': {0: 'TAUGAPSET_TAUGAP_0', 1: 'TAUGAPSET_TAUGAP_1', 2: 'TAUGAPSET_TAUGAP_2', 3: 'TAUGAPSET_TAUGAP_3', 4: 'TAUGAPSET_TAUGAP_4', 5: 'TAUGAPSET_TAUGAP_5', 6: 'TAUGAPSET_TAUGAP_6', 7: 'TAUGAPSET_TAUGAP_7'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_taugapset(uint8_t* data, Mrr_0x246_246::TaugapsetType taugapset);

  // config detail: {'name': 'dxTarObj', 'enum': {0: 'DXTAROBJ_DISTANCE_0', 1: 'DXTAROBJ_DISTANCE_1', 2: 'DXTAROBJ_DISTANCE_2', 3: 'DXTAROBJ_DISTANCE_3', 4: 'DXTAROBJ_DISTANCE_4', 5: 'DXTAROBJ_DISTANCE_5', 6: 'DXTAROBJ_DISTANCE_6', 7: 'DXTAROBJ_DISTANCE_7'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_dxtarobj(uint8_t* data, Mrr_0x246_246::DxtarobjType dxtarobj);

  // config detail: {'name': 'ACCHMI_Mode', 'enum': {0: 'ACCHMI_MODE_OFF_MODE', 1: 'ACCHMI_MODE_PASSIVE_MODE', 2: 'ACCHMI_MODE_STAND_BY_MODE', 3: 'ACCHMI_MODE_ACTIVE_CONTROL_MODE', 4: 'ACCHMI_MODE_BRAKE_ONLY_MODE', 5: 'ACCHMI_MODE_OVERRIDE', 6: 'ACCHMI_MODE_STANDSTILL', 7: 'ACCHMI_MODE_FAILURE_MODE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_acchmi_mode(uint8_t* data, Mrr_0x246_246::Acchmi_modeType acchmi_mode);

  // config detail: {'name': 'ACCFailInfo', 'enum': {0: 'ACCFAILINFO_NO_ERROR', 1: 'ACCFAILINFO_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_accfailinfo(uint8_t* data, Mrr_0x246_246::AccfailinfoType accfailinfo);

  // config detail: {'name': 'TakeOverReq', 'enum': {0: 'TAKEOVERREQ_NO_TAKEOVER_REQUEST', 1: 'TAKEOVERREQ_VALID_TAKEOVER_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_takeoverreq(uint8_t* data, Mrr_0x246_246::TakeoverreqType takeoverreq);

  // config detail: {'name': 'MRR_FCW_Sensitive', 'enum': {0: 'MRR_FCW_SENSITIVE_UNAVAILABLE', 1: 'MRR_FCW_SENSITIVE_LEVEL1_LOW_SENSITIVE', 2: 'MRR_FCW_SENSITIVE_LEVEL2_NORMAL', 3: 'MRR_FCW_SENSITIVE_LEVEL3_HIGH_SENSITIVE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_mrr_fcw_sensitive(uint8_t* data, Mrr_0x246_246::Mrr_fcw_sensitiveType mrr_fcw_sensitive);

  // config detail: {'name': 'AEB_STATE', 'enum': {0: 'AEB_STATE_UNAVAILABLE', 1: 'AEB_STATE_OFF', 2: 'AEB_STATE_STANDBY', 3: 'AEB_STATE_ACTIVE_NO_INTERVENTION', 4: 'AEB_STATE_ACTIVE', 5: 'AEB_STATE_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_aeb_state(uint8_t* data, Mrr_0x246_246::Aeb_stateType aeb_state);

  // config detail: {'name': 'ACC_Startstop_infor', 'enum': {0: 'ACC_STARTSTOP_INFOR_ACC_STOPALLOWED', 1: 'ACC_STARTSTOP_INFOR_ACC_STOPFORBIDDEN', 2: 'ACC_STARTSTOP_INFOR_ACC_STARTREQUEST', 3: 'ACC_STARTSTOP_INFOR_ACC_SYSTEMFAILURE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_acc_startstop_infor(uint8_t* data, Mrr_0x246_246::Acc_startstop_inforType acc_startstop_infor);

  // config detail: {'name': 'FCW_preWarning', 'enum': {0: 'FCW_PREWARNING_NO_WARNING', 1: 'FCW_PREWARNING_WARNING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 30, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_fcw_prewarning(uint8_t* data, Mrr_0x246_246::Fcw_prewarningType fcw_prewarning);

  // config detail: {'name': 'FCW_latentWarning', 'enum': {0: 'FCW_LATENTWARNING_NO_WARNING', 1: 'FCW_LATENTWARNING_WARNING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_fcw_latentwarning(uint8_t* data, Mrr_0x246_246::Fcw_latentwarningType fcw_latentwarning);

  // config detail: {'name': 'FCW_STATE', 'enum': {0: 'FCW_STATE_UNAVAILABLE', 1: 'FCW_STATE_OFF', 2: 'FCW_STATE_STANDBY', 3: 'FCW_STATE_ACTIVE_NO_INTERVENTION', 4: 'FCW_STATE_ACTIVE', 5: 'FCW_STATE_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_fcw_state(uint8_t* data, Mrr_0x246_246::Fcw_stateType fcw_state);

  // config detail: {'name': 'Obj_Speed', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_obj_speed(uint8_t* data, int obj_speed);

  // config detail: {'name': 'Rolling_counter_0x246', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_rolling_counter_0x246(uint8_t* data, int rolling_counter_0x246);

  // config detail: {'name': 'Textinfo', 'enum': {0: 'TEXTINFO_NO_DISPLAY', 1: 'TEXTINFO_ACC_IS_SWITCHED_ON', 2: 'TEXTINFO_ACC_IS_SWITCHED_OFF', 3: 'TEXTINFO_ACC_IS_CANCELLED', 4: 'TEXTINFO_ACC_ACTIVE', 5: 'TEXTINFO_MRR_BLINDNESS', 6: 'TEXTINFO_ACC_AND_PEBS_ERROR', 7: 'TEXTINFO_EPB_ACTIVATE', 8: 'TEXTINFO_NO_FORWARD_GEAR', 9: 'TEXTINFO_SEATBELT_UNBUCKLED', 10: 'TEXTINFO_ESP_OFF', 11: 'TEXTINFO_SPEED_OVER_150KPH', 12: 'TEXTINFO_DOOR_OPEN', 13: 'TEXTINFO_OVERRIDE', 14: 'TEXTINFO_ESP_ERROR', 15: 'TEXTINFO_UNCALIBRATED'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_textinfo(uint8_t* data, Mrr_0x246_246::TextinfoType textinfo);

  // config detail: {'name': 'Checksum_0x246', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_0x246(uint8_t* data, int checksum_0x246);

  // config detail: {'name': 'vSetDis', 'enum': {511: 'VSETDIS_INVALID'}, 'precision': 0.5, 'len': 9, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255.5]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'kph'}
  void set_p_vsetdis(uint8_t* data, Mrr_0x246_246::VsetdisType vsetdis);

  // config detail: {'name': 'ObjValid', 'enum': {0: 'OBJVALID_NO_OBJECT', 1: 'OBJVALID_TARGET_OBJECT_DETECTED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_objvalid(uint8_t* data, Mrr_0x246_246::ObjvalidType objvalid);

 private:
  Mrr_0x246_246::TaugapsetType taugapset_;
  Mrr_0x246_246::DxtarobjType dxtarobj_;
  Mrr_0x246_246::Acchmi_modeType acchmi_mode_;
  Mrr_0x246_246::AccfailinfoType accfailinfo_;
  Mrr_0x246_246::TakeoverreqType takeoverreq_;
  Mrr_0x246_246::Mrr_fcw_sensitiveType mrr_fcw_sensitive_;
  Mrr_0x246_246::Aeb_stateType aeb_state_;
  Mrr_0x246_246::Acc_startstop_inforType acc_startstop_infor_;
  Mrr_0x246_246::Fcw_prewarningType fcw_prewarning_;
  Mrr_0x246_246::Fcw_latentwarningType fcw_latentwarning_;
  Mrr_0x246_246::Fcw_stateType fcw_state_;
  int obj_speed_;
  int rolling_counter_0x246_;
  Mrr_0x246_246::TextinfoType textinfo_;
  int checksum_0x246_;
  Mrr_0x246_246::VsetdisType vsetdis_;
  Mrr_0x246_246::ObjvalidType objvalid_;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_MRR_0X246_246_H_
