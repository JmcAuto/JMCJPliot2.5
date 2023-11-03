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

#include "modules/canbus/vehicle/cx75/protocol/mrr_0x246_246.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Mrr0x246246::ID = 0x246;

// public
Mrr0x246246::Mrr0x246246() { Reset(); }

uint32_t Mrr0x246246::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Mrr0x246246::UpdateData(uint8_t* data) {
  set_p_taugapset(data, taugapset_);
  set_p_dxtarobj(data, dxtarobj_);
  set_p_acchmi_mode(data, acchmi_mode_);
  set_p_accfailinfo(data, accfailinfo_);
  set_p_takeoverreq(data, takeoverreq_);
  set_p_mrr_fcw_sensitive(data, mrr_fcw_sensitive_);
  set_p_aeb_state(data, aeb_state_);
  set_p_acc_startstop_infor(data, acc_startstop_infor_);
  set_p_fcw_prewarning(data, fcw_prewarning_);
  set_p_fcw_latentwarning(data, fcw_latentwarning_);
  set_p_fcw_state(data, fcw_state_);
  set_p_obj_speed(data, obj_speed_);
  set_p_rolling_counter_0x246(data, rolling_counter_0x246_);
  set_p_textinfo(data, textinfo_);
  set_p_checksum_0x246(data, checksum_0x246_);
  set_p_vsetdis(data, vsetdis_);
  set_p_objvalid(data, objvalid_);
}

void Mrr0x246246::Reset() {
  // TODO you should check this manually
  taugapset_ = Mrr_0x246_246::TAUGAPSET_TAUGAP_0;
  dxtarobj_ = Mrr_0x246_246::DXTAROBJ_DISTANCE_0;
  acchmi_mode_ = Mrr_0x246_246::ACCHMI_MODE_OFF_MODE;
  accfailinfo_ = Mrr_0x246_246::ACCFAILINFO_NO_ERROR;
  takeoverreq_ = Mrr_0x246_246::TAKEOVERREQ_NO_TAKEOVER_REQUEST;
  mrr_fcw_sensitive_ = Mrr_0x246_246::MRR_FCW_SENSITIVE_UNAVAILABLE;
  aeb_state_ = Mrr_0x246_246::AEB_STATE_UNAVAILABLE;
  acc_startstop_infor_ = Mrr_0x246_246::ACC_STARTSTOP_INFOR_ACC_STOPALLOWED;
  fcw_prewarning_ = Mrr_0x246_246::FCW_PREWARNING_NO_WARNING;
  fcw_latentwarning_ = Mrr_0x246_246::FCW_LATENTWARNING_NO_WARNING;
  fcw_state_ = Mrr_0x246_246::FCW_STATE_UNAVAILABLE;
  obj_speed_ = 0;
  rolling_counter_0x246_ = 0;
  textinfo_ = Mrr_0x246_246::TEXTINFO_NO_DISPLAY;
  checksum_0x246_ = 0;
  vsetdis_ = Mrr_0x246_246::VSETDIS_INVALID;
  objvalid_ = Mrr_0x246_246::OBJVALID_NO_OBJECT;
}

Mrr0x246246* Mrr0x246246::set_taugapset(
    Mrr_0x246_246::TaugapsetType taugapset) {
  taugapset_ = taugapset;
  return this;
 }

// config detail: {'name': 'TauGapSet', 'enum': {0: 'TAUGAPSET_TAUGAP_0', 1: 'TAUGAPSET_TAUGAP_1', 2: 'TAUGAPSET_TAUGAP_2', 3: 'TAUGAPSET_TAUGAP_3', 4: 'TAUGAPSET_TAUGAP_4', 5: 'TAUGAPSET_TAUGAP_5', 6: 'TAUGAPSET_TAUGAP_6', 7: 'TAUGAPSET_TAUGAP_7'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_taugapset(uint8_t* data,
    Mrr_0x246_246::TaugapsetType taugapset) {
  int x = taugapset;

  Byte to_set(data + 1);
  to_set.set_value(x, 1, 3);
}


Mrr0x246246* Mrr0x246246::set_dxtarobj(
    Mrr_0x246_246::DxtarobjType dxtarobj) {
  dxtarobj_ = dxtarobj;
  return this;
 }

// config detail: {'name': 'dxTarObj', 'enum': {0: 'DXTAROBJ_DISTANCE_0', 1: 'DXTAROBJ_DISTANCE_1', 2: 'DXTAROBJ_DISTANCE_2', 3: 'DXTAROBJ_DISTANCE_3', 4: 'DXTAROBJ_DISTANCE_4', 5: 'DXTAROBJ_DISTANCE_5', 6: 'DXTAROBJ_DISTANCE_6', 7: 'DXTAROBJ_DISTANCE_7'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_dxtarobj(uint8_t* data,
    Mrr_0x246_246::DxtarobjType dxtarobj) {
  int x = dxtarobj;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 3);
}


Mrr0x246246* Mrr0x246246::set_acchmi_mode(
    Mrr_0x246_246::Acchmi_modeType acchmi_mode) {
  acchmi_mode_ = acchmi_mode;
  return this;
 }

// config detail: {'name': 'ACCHMI_Mode', 'enum': {0: 'ACCHMI_MODE_OFF_MODE', 1: 'ACCHMI_MODE_PASSIVE_MODE', 2: 'ACCHMI_MODE_STAND_BY_MODE', 3: 'ACCHMI_MODE_ACTIVE_CONTROL_MODE', 4: 'ACCHMI_MODE_BRAKE_ONLY_MODE', 5: 'ACCHMI_MODE_OVERRIDE', 6: 'ACCHMI_MODE_STANDSTILL', 7: 'ACCHMI_MODE_FAILURE_MODE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_acchmi_mode(uint8_t* data,
    Mrr_0x246_246::Acchmi_modeType acchmi_mode) {
  int x = acchmi_mode;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 3);
}


Mrr0x246246* Mrr0x246246::set_accfailinfo(
    Mrr_0x246_246::AccfailinfoType accfailinfo) {
  accfailinfo_ = accfailinfo;
  return this;
 }

// config detail: {'name': 'ACCFailInfo', 'enum': {0: 'ACCFAILINFO_NO_ERROR', 1: 'ACCFAILINFO_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_accfailinfo(uint8_t* data,
    Mrr_0x246_246::AccfailinfoType accfailinfo) {
  int x = accfailinfo;

  Byte to_set(data + 2);
  to_set.set_value(x, 3, 1);
}


Mrr0x246246* Mrr0x246246::set_takeoverreq(
    Mrr_0x246_246::TakeoverreqType takeoverreq) {
  takeoverreq_ = takeoverreq;
  return this;
 }

// config detail: {'name': 'TakeOverReq', 'enum': {0: 'TAKEOVERREQ_NO_TAKEOVER_REQUEST', 1: 'TAKEOVERREQ_VALID_TAKEOVER_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_takeoverreq(uint8_t* data,
    Mrr_0x246_246::TakeoverreqType takeoverreq) {
  int x = takeoverreq;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 1);
}


Mrr0x246246* Mrr0x246246::set_mrr_fcw_sensitive(
    Mrr_0x246_246::Mrr_fcw_sensitiveType mrr_fcw_sensitive) {
  mrr_fcw_sensitive_ = mrr_fcw_sensitive;
  return this;
 }

// config detail: {'name': 'MRR_FCW_Sensitive', 'enum': {0: 'MRR_FCW_SENSITIVE_UNAVAILABLE', 1: 'MRR_FCW_SENSITIVE_LEVEL1_LOW_SENSITIVE', 2: 'MRR_FCW_SENSITIVE_LEVEL2_NORMAL', 3: 'MRR_FCW_SENSITIVE_LEVEL3_HIGH_SENSITIVE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_mrr_fcw_sensitive(uint8_t* data,
    Mrr_0x246_246::Mrr_fcw_sensitiveType mrr_fcw_sensitive) {
  int x = mrr_fcw_sensitive;

  Byte to_set(data + 2);
  to_set.set_value(x, 6, 2);
}


Mrr0x246246* Mrr0x246246::set_aeb_state(
    Mrr_0x246_246::Aeb_stateType aeb_state) {
  aeb_state_ = aeb_state;
  return this;
 }

// config detail: {'name': 'AEB_STATE', 'enum': {0: 'AEB_STATE_UNAVAILABLE', 1: 'AEB_STATE_OFF', 2: 'AEB_STATE_STANDBY', 3: 'AEB_STATE_ACTIVE_NO_INTERVENTION', 4: 'AEB_STATE_ACTIVE', 5: 'AEB_STATE_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_aeb_state(uint8_t* data,
    Mrr_0x246_246::Aeb_stateType aeb_state) {
  int x = aeb_state;

  Byte to_set(data + 3);
  to_set.set_value(x, 1, 3);
}


Mrr0x246246* Mrr0x246246::set_acc_startstop_infor(
    Mrr_0x246_246::Acc_startstop_inforType acc_startstop_infor) {
  acc_startstop_infor_ = acc_startstop_infor;
  return this;
 }

// config detail: {'name': 'ACC_Startstop_infor', 'enum': {0: 'ACC_STARTSTOP_INFOR_ACC_STOPALLOWED', 1: 'ACC_STARTSTOP_INFOR_ACC_STOPFORBIDDEN', 2: 'ACC_STARTSTOP_INFOR_ACC_STARTREQUEST', 3: 'ACC_STARTSTOP_INFOR_ACC_SYSTEMFAILURE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_acc_startstop_infor(uint8_t* data,
    Mrr_0x246_246::Acc_startstop_inforType acc_startstop_infor) {
  int x = acc_startstop_infor;

  Byte to_set(data + 3);
  to_set.set_value(x, 4, 2);
}


Mrr0x246246* Mrr0x246246::set_fcw_prewarning(
    Mrr_0x246_246::Fcw_prewarningType fcw_prewarning) {
  fcw_prewarning_ = fcw_prewarning;
  return this;
 }

// config detail: {'name': 'FCW_preWarning', 'enum': {0: 'FCW_PREWARNING_NO_WARNING', 1: 'FCW_PREWARNING_WARNING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 30, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_fcw_prewarning(uint8_t* data,
    Mrr_0x246_246::Fcw_prewarningType fcw_prewarning) {
  int x = fcw_prewarning;

  Byte to_set(data + 3);
  to_set.set_value(x, 6, 1);
}


Mrr0x246246* Mrr0x246246::set_fcw_latentwarning(
    Mrr_0x246_246::Fcw_latentwarningType fcw_latentwarning) {
  fcw_latentwarning_ = fcw_latentwarning;
  return this;
 }

// config detail: {'name': 'FCW_latentWarning', 'enum': {0: 'FCW_LATENTWARNING_NO_WARNING', 1: 'FCW_LATENTWARNING_WARNING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_fcw_latentwarning(uint8_t* data,
    Mrr_0x246_246::Fcw_latentwarningType fcw_latentwarning) {
  int x = fcw_latentwarning;

  Byte to_set(data + 3);
  to_set.set_value(x, 7, 1);
}


Mrr0x246246* Mrr0x246246::set_fcw_state(
    Mrr_0x246_246::Fcw_stateType fcw_state) {
  fcw_state_ = fcw_state;
  return this;
 }

// config detail: {'name': 'FCW_STATE', 'enum': {0: 'FCW_STATE_UNAVAILABLE', 1: 'FCW_STATE_OFF', 2: 'FCW_STATE_STANDBY', 3: 'FCW_STATE_ACTIVE_NO_INTERVENTION', 4: 'FCW_STATE_ACTIVE', 5: 'FCW_STATE_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_fcw_state(uint8_t* data,
    Mrr_0x246_246::Fcw_stateType fcw_state) {
  int x = fcw_state;

  Byte to_set(data + 4);
  to_set.set_value(x, 5, 3);
}


Mrr0x246246* Mrr0x246246::set_obj_speed(
    int obj_speed) {
  obj_speed_ = obj_speed;
  return this;
 }

// config detail: {'name': 'Obj_Speed', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_obj_speed(uint8_t* data,
    int obj_speed) {
  obj_speed = ProtocolData::BoundedValue(0, 255, obj_speed);
  int x = obj_speed;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}


Mrr0x246246* Mrr0x246246::set_rolling_counter_0x246(
    int rolling_counter_0x246) {
  rolling_counter_0x246_ = rolling_counter_0x246;
  return this;
 }

// config detail: {'name': 'Rolling_counter_0x246', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_rolling_counter_0x246(uint8_t* data,
    int rolling_counter_0x246) {
  rolling_counter_0x246_++;
  rolling_counter_0x246_=rolling_counter_0x246_<=15?rolling_counter_0x246_:0;
  rolling_counter_0x246_ = ProtocolData::BoundedValue(0, 15, rolling_counter_0x246_);
  int x = rolling_counter_0x246_;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}


Mrr0x246246* Mrr0x246246::set_textinfo(
    Mrr_0x246_246::TextinfoType textinfo) {
  textinfo_ = textinfo;
  return this;
 }

// config detail: {'name': 'Textinfo', 'enum': {0: 'TEXTINFO_NO_DISPLAY', 1: 'TEXTINFO_ACC_IS_SWITCHED_ON', 2: 'TEXTINFO_ACC_IS_SWITCHED_OFF', 3: 'TEXTINFO_ACC_IS_CANCELLED', 4: 'TEXTINFO_ACC_ACTIVE', 5: 'TEXTINFO_MRR_BLINDNESS', 6: 'TEXTINFO_ACC_AND_PEBS_ERROR', 7: 'TEXTINFO_EPB_ACTIVATE', 8: 'TEXTINFO_NO_FORWARD_GEAR', 9: 'TEXTINFO_SEATBELT_UNBUCKLED', 10: 'TEXTINFO_ESP_OFF', 11: 'TEXTINFO_SPEED_OVER_150KPH', 12: 'TEXTINFO_DOOR_OPEN', 13: 'TEXTINFO_OVERRIDE', 14: 'TEXTINFO_ESP_ERROR', 15: 'TEXTINFO_UNCALIBRATED'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_textinfo(uint8_t* data,
    Mrr_0x246_246::TextinfoType textinfo) {
  int x = textinfo;

  Byte to_set(data + 6);
  to_set.set_value(x, 4, 4);
}


Mrr0x246246* Mrr0x246246::set_checksum_0x246(
    int checksum_0x246) {
  checksum_0x246_ = checksum_0x246;
  return this;
 }

// config detail: {'name': 'Checksum_0x246', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_checksum_0x246(uint8_t* data,
    int checksum_0x246) {
  checksum_0x246 = (data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]);
  checksum_0x246 = ProtocolData::BoundedValue(0, 255, checksum_0x246);
  int x = checksum_0x246;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Mrr0x246246* Mrr0x246246::set_vsetdis(
    Mrr_0x246_246::VsetdisType vsetdis) {
  vsetdis_ = vsetdis;
  return this;
 }

// config detail: {'name': 'vSetDis', 'enum': {511: 'VSETDIS_INVALID'}, 'precision': 0.5, 'len': 9, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255.5]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'kph'}
void Mrr0x246246::set_p_vsetdis(uint8_t* data,
    Mrr_0x246_246::VsetdisType vsetdis) {
  int x = vsetdis / 0.500000;
  uint8_t t = 0;

  t = x & 0x1;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 7, 1);
  x >>= 1;

  t = x & 0xFF;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 8);
}


Mrr0x246246* Mrr0x246246::set_objvalid(
    Mrr_0x246_246::ObjvalidType objvalid) {
  objvalid_ = objvalid;
  return this;
 }

// config detail: {'name': 'ObjValid', 'enum': {0: 'OBJVALID_NO_OBJECT', 1: 'OBJVALID_TARGET_OBJECT_DETECTED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Mrr0x246246::set_p_objvalid(uint8_t* data,
    Mrr_0x246_246::ObjvalidType objvalid) {
  int x = objvalid;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 1);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
