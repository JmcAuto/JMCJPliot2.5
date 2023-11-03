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

#include "modules/canbus/vehicle/cx75/protocol/pam_0x271_271.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Pam0x271271::ID = 0x271;

// public
Pam0x271271::Pam0x271271() { Reset(); }

uint32_t Pam0x271271::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Pam0x271271::UpdateData(uint8_t* data) {
  set_p_pam_infodisplayreq(data, pam_infodisplayreq_);
  set_p_pam_objectstsflm(data, pam_objectstsflm_);
  set_p_pam_objectstsflc(data, pam_objectstsflc_);
  set_p_pam_objectstsfrc(data, pam_objectstsfrc_);
  set_p_pam_objectstsfrm(data, pam_objectstsfrm_);
  set_p_pam_objectstsrlm(data, pam_objectstsrlm_);
  set_p_pam_objectstsrlc(data, pam_objectstsrlc_);
  set_p_pam_objectstsrrc(data, pam_objectstsrrc_);
  set_p_pam_objectstsrrm(data, pam_objectstsrrm_);
  set_p_pam_chimecmdsoundstatus(data, pam_chimecmdsoundstatus_);
  set_p_pam_rearsystemworksts(data, pam_rearsystemworksts_);
  set_p_pam_frontsystemworksts(data, pam_frontsystemworksts_);
  set_p_pam_parkingbarsts(data, pam_parkingbarsts_);
  set_p_pam_chimecmdsoundcadence(data, pam_chimecmdsoundcadence_);
  set_p_pam_parkingbarpercent(data, pam_parkingbarpercent_);
}

void Pam0x271271::Reset() {
  // TODO you should check this manually
  pam_infodisplayreq_ = Pam_0x271_271::PAM_INFODISPLAYREQ_NO_REQUEST;
  pam_objectstsflm_ = Pam_0x271_271::PAM_OBJECTSTSFLM_NO_OBJECT;
  pam_objectstsflc_ = Pam_0x271_271::PAM_OBJECTSTSFLC_NO_OBJECT;
  pam_objectstsfrc_ = Pam_0x271_271::PAM_OBJECTSTSFRC_NO_OBJECT;
  pam_objectstsfrm_ = Pam_0x271_271::PAM_OBJECTSTSFRM_NO_OBJECT;
  pam_objectstsrlm_ = Pam_0x271_271::PAM_OBJECTSTSRLM_NO_OBJECT;
  pam_objectstsrlc_ = Pam_0x271_271::PAM_OBJECTSTSRLC_NO_OBJECT;
  pam_objectstsrrc_ = Pam_0x271_271::PAM_OBJECTSTSRRC_NO_OBJECT;
  pam_objectstsrrm_ = Pam_0x271_271::PAM_OBJECTSTSRRM_NO_OBJECT;
  pam_chimecmdsoundstatus_ = Pam_0x271_271::PAM_CHIMECMDSOUNDSTATUS_NOWARNING;
  pam_rearsystemworksts_ = Pam_0x271_271::PAM_REARSYSTEMWORKSTS_DISABLE;
  pam_frontsystemworksts_ = Pam_0x271_271::PAM_FRONTSYSTEMWORKSTS_DISABLE;
  pam_parkingbarsts_ = Pam_0x271_271::PAM_PARKINGBARSTS_DISPLAY_OFF;
  pam_chimecmdsoundcadence_ = Pam_0x271_271::PAM_CHIMECMDSOUNDCADENCE_TONE_0_NO_WARNING;
  pam_parkingbarpercent_ = Pam_0x271_271::PAM_PARKINGBARPERCENT_0PERCENT;
}

Pam0x271271* Pam0x271271::set_pam_infodisplayreq(
    Pam_0x271_271::Pam_infodisplayreqType pam_infodisplayreq) {
  pam_infodisplayreq_ = pam_infodisplayreq;
  return this;
 }

// config detail: {'name': 'PAM_InfoDisplayReq', 'enum': {0: 'PAM_INFODISPLAYREQ_NO_REQUEST', 1: 'PAM_INFODISPLAYREQ_REQ_FORWARD_AGAIN', 2: 'PAM_INFODISPLAYREQ_REQ_TURN_LEVER', 3: 'PAM_INFODISPLAYREQ_REQ_GEAR_P', 4: 'PAM_INFODISPLAYREQ_REQ_CONFIRM_SWITCH', 5: 'PAM_INFODISPLAYREQ_REQ_SEARCHINGPROCESS', 6: 'PAM_INFODISPLAYREQ_REQ_DRIVE_FW', 7: 'PAM_INFODISPLAYREQ_REQ_STOP', 8: 'PAM_INFODISPLAYREQ_REQ_FUNCTION_SELECT', 9: 'PAM_INFODISPLAYREQ_REQ_FUNCTION_OFF', 10: 'PAM_INFODISPLAYREQ_REQ_PROCESS_BAR', 11: 'PAM_INFODISPLAYREQ_PPSC_REQUEST_ONLY', 12: 'PAM_INFODISPLAYREQ_CPSC_REQUEST_ONLY', 13: 'PAM_INFODISPLAYREQ_PPSC_CPSC_REQUEST', 14: 'PAM_INFODISPLAYREQ_REQ_GEAR_R', 15: 'PAM_INFODISPLAYREQ_REQ_GEAR_D', 16: 'PAM_INFODISPLAYREQ_REQ_DRIVE_BW', 17: 'PAM_INFODISPLAYREQ_REQ_SLOW_DOWN', 18: 'PAM_INFODISPLAYREQ_REQ_RELEASE_BRAKE_ANDSTEERING_WHEEL', 19: 'PAM_INFODISPLAYREQ_REQ_CLOSE_DOOR', 20: 'PAM_INFODISPLAYREQ_REQ_BUCKLE_SEAT_BELT', 21: 'PAM_INFODISPLAYREQ_REQ_REPRESS_DEAD_MAN_SWITCH', 22: 'PAM_INFODISPLAYREQ_REQ_PRESS_BRAKE_PEDAL', 23: 'PAM_INFODISPLAYREQ_REQ_GEAR_R_AND_BRAKE', 24: 'PAM_INFODISPLAYREQ_REQ_FUNCTION_COMPLETE', 25: 'PAM_INFODISPLAYREQ_REQ_FUNCTION_NOT_AVAILABLE', 26: 'PAM_INFODISPLAYREQ_RESERVED', 27: 'PAM_INFODISPLAYREQ_RESERVED', 28: 'PAM_INFODISPLAYREQ_RESERVED', 29: 'PAM_INFODISPLAYREQ_RESERVED', 30: 'PAM_INFODISPLAYREQ_RESERVED', 31: 'PAM_INFODISPLAYREQ_RESERVED'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_infodisplayreq(uint8_t* data,
    Pam_0x271_271::Pam_infodisplayreqType pam_infodisplayreq) {
  int x = pam_infodisplayreq;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}


Pam0x271271* Pam0x271271::set_pam_objectstsflm(
    Pam_0x271_271::Pam_objectstsflmType pam_objectstsflm) {
  pam_objectstsflm_ = pam_objectstsflm;
  return this;
 }

// config detail: {'name': 'PAM_ObjectStsFLM', 'enum': {0: 'PAM_OBJECTSTSFLM_NO_OBJECT', 1: 'PAM_OBJECTSTSFLM_OBJECT_IN_ZONE_1', 2: 'PAM_OBJECTSTSFLM_OBJECT_ZONE_2', 3: 'PAM_OBJECTSTSFLM_OBJECT_ZONE_3', 4: 'PAM_OBJECTSTSFLM_OBJECT_ZONE_4', 5: 'PAM_OBJECTSTSFLM_OBJECT_ZONE_5'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_objectstsflm(uint8_t* data,
    Pam_0x271_271::Pam_objectstsflmType pam_objectstsflm) {
  int x = pam_objectstsflm;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 4);
}


Pam0x271271* Pam0x271271::set_pam_objectstsflc(
    Pam_0x271_271::Pam_objectstsflcType pam_objectstsflc) {
  pam_objectstsflc_ = pam_objectstsflc;
  return this;
 }

// config detail: {'name': 'PAM_ObjectStsFLC', 'enum': {0: 'PAM_OBJECTSTSFLC_NO_OBJECT', 1: 'PAM_OBJECTSTSFLC_OBJECT_IN_ZONE_1', 2: 'PAM_OBJECTSTSFLC_OBJECT_ZONE_2', 3: 'PAM_OBJECTSTSFLC_OBJECT_ZONE_3'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_objectstsflc(uint8_t* data,
    Pam_0x271_271::Pam_objectstsflcType pam_objectstsflc) {
  int x = pam_objectstsflc;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 4);
}


Pam0x271271* Pam0x271271::set_pam_objectstsfrc(
    Pam_0x271_271::Pam_objectstsfrcType pam_objectstsfrc) {
  pam_objectstsfrc_ = pam_objectstsfrc;
  return this;
 }

// config detail: {'name': 'PAM_objectStsFRC', 'enum': {0: 'PAM_OBJECTSTSFRC_NO_OBJECT', 1: 'PAM_OBJECTSTSFRC_OBJECT_IN_ZONE_1', 2: 'PAM_OBJECTSTSFRC_OBJECT_ZONE_2', 3: 'PAM_OBJECTSTSFRC_OBJECT_ZONE_3'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_objectstsfrc(uint8_t* data,
    Pam_0x271_271::Pam_objectstsfrcType pam_objectstsfrc) {
  int x = pam_objectstsfrc;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 4);
}


Pam0x271271* Pam0x271271::set_pam_objectstsfrm(
    Pam_0x271_271::Pam_objectstsfrmType pam_objectstsfrm) {
  pam_objectstsfrm_ = pam_objectstsfrm;
  return this;
 }

// config detail: {'name': 'PAM_ObjectStsFRM', 'enum': {0: 'PAM_OBJECTSTSFRM_NO_OBJECT', 1: 'PAM_OBJECTSTSFRM_OBJECT_IN_ZONE_1', 2: 'PAM_OBJECTSTSFRM_OBJECT_ZONE_2', 3: 'PAM_OBJECTSTSFRM_OBJECT_ZONE_3', 4: 'PAM_OBJECTSTSFRM_OBJECT_ZONE_4', 5: 'PAM_OBJECTSTSFRM_OBJECT_ZONE_5'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_objectstsfrm(uint8_t* data,
    Pam_0x271_271::Pam_objectstsfrmType pam_objectstsfrm) {
  int x = pam_objectstsfrm;

  Byte to_set(data + 3);
  to_set.set_value(x, 4, 4);
}


Pam0x271271* Pam0x271271::set_pam_objectstsrlm(
    Pam_0x271_271::Pam_objectstsrlmType pam_objectstsrlm) {
  pam_objectstsrlm_ = pam_objectstsrlm;
  return this;
 }

// config detail: {'name': 'PAM_objectStsRLM', 'enum': {0: 'PAM_OBJECTSTSRLM_NO_OBJECT', 1: 'PAM_OBJECTSTSRLM_OBJECT_IN_ZONE1', 2: 'PAM_OBJECTSTSRLM_OBJECT_ZONE2', 3: 'PAM_OBJECTSTSRLM_OBJECT_ZONE3', 4: 'PAM_OBJECTSTSRLM_OBJECT_ZONE4', 5: 'PAM_OBJECTSTSRLM_OBJECT_ZONE5', 6: 'PAM_OBJECTSTSRLM_OBJECT_ZONE6', 7: 'PAM_OBJECTSTSRLM_OBJECT_ZONE7', 8: 'PAM_OBJECTSTSRLM_RESERVED', 9: 'PAM_OBJECTSTSRLM_RESERVED', 10: 'PAM_OBJECTSTSRLM_RESERVED', 11: 'PAM_OBJECTSTSRLM_RESERVED', 12: 'PAM_OBJECTSTSRLM_RESERVED', 13: 'PAM_OBJECTSTSRLM_RESERVED', 14: 'PAM_OBJECTSTSRLM_RESERVED', 15: 'PAM_OBJECTSTSRLM_RESERVED'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_objectstsrlm(uint8_t* data,
    Pam_0x271_271::Pam_objectstsrlmType pam_objectstsrlm) {
  int x = pam_objectstsrlm;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 4);
}


Pam0x271271* Pam0x271271::set_pam_objectstsrlc(
    Pam_0x271_271::Pam_objectstsrlcType pam_objectstsrlc) {
  pam_objectstsrlc_ = pam_objectstsrlc;
  return this;
 }

// config detail: {'name': 'PAM_objectStsRLC', 'enum': {0: 'PAM_OBJECTSTSRLC_NO_OBJECT', 1: 'PAM_OBJECTSTSRLC_OBJECT_IN_ZONE_1', 2: 'PAM_OBJECTSTSRLC_OBJECT_ZONE_2', 3: 'PAM_OBJECTSTSRLC_OBJECT_ZONE_3'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_objectstsrlc(uint8_t* data,
    Pam_0x271_271::Pam_objectstsrlcType pam_objectstsrlc) {
  int x = pam_objectstsrlc;

  Byte to_set(data + 4);
  to_set.set_value(x, 4, 4);
}


Pam0x271271* Pam0x271271::set_pam_objectstsrrc(
    Pam_0x271_271::Pam_objectstsrrcType pam_objectstsrrc) {
  pam_objectstsrrc_ = pam_objectstsrrc;
  return this;
 }

// config detail: {'name': 'PAM_objectStsRRC', 'enum': {0: 'PAM_OBJECTSTSRRC_NO_OBJECT', 1: 'PAM_OBJECTSTSRRC_OBJECT_IN_ZONE_1', 2: 'PAM_OBJECTSTSRRC_OBJECT_ZONE_2', 3: 'PAM_OBJECTSTSRRC_OBJECT_ZONE_3'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 43, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_objectstsrrc(uint8_t* data,
    Pam_0x271_271::Pam_objectstsrrcType pam_objectstsrrc) {
  int x = pam_objectstsrrc;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 4);
}


Pam0x271271* Pam0x271271::set_pam_objectstsrrm(
    Pam_0x271_271::Pam_objectstsrrmType pam_objectstsrrm) {
  pam_objectstsrrm_ = pam_objectstsrrm;
  return this;
 }

// config detail: {'name': 'PAM_objectStsRRM', 'enum': {0: 'PAM_OBJECTSTSRRM_NO_OBJECT', 1: 'PAM_OBJECTSTSRRM_OBJECT_IN_ZONE_1', 2: 'PAM_OBJECTSTSRRM_OBJECT_ZONE_2', 3: 'PAM_OBJECTSTSRRM_OBJECT_ZONE_3', 4: 'PAM_OBJECTSTSRRM_OBJECT_ZONE_4', 5: 'PAM_OBJECTSTSRRM_OBJECT_ZONE_5', 6: 'PAM_OBJECTSTSRRM_OBJECT_ZONE_6', 7: 'PAM_OBJECTSTSRRM_OBJECT_ZONE_7'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_objectstsrrm(uint8_t* data,
    Pam_0x271_271::Pam_objectstsrrmType pam_objectstsrrm) {
  int x = pam_objectstsrrm;

  Byte to_set(data + 5);
  to_set.set_value(x, 4, 4);
}


Pam0x271271* Pam0x271271::set_pam_chimecmdsoundstatus(
    Pam_0x271_271::Pam_chimecmdsoundstatusType pam_chimecmdsoundstatus) {
  pam_chimecmdsoundstatus_ = pam_chimecmdsoundstatus;
  return this;
 }

// config detail: {'name': 'PAM_ChimeCmdSoundstatus', 'enum': {0: 'PAM_CHIMECMDSOUNDSTATUS_NOWARNING', 1: 'PAM_CHIMECMDSOUNDSTATUS_FRONTSYSTEMWARNING', 2: 'PAM_CHIMECMDSOUNDSTATUS_REARSYSTEMWARNING', 3: 'PAM_CHIMECMDSOUNDSTATUS_APA'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 49, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_chimecmdsoundstatus(uint8_t* data,
    Pam_0x271_271::Pam_chimecmdsoundstatusType pam_chimecmdsoundstatus) {
  int x = pam_chimecmdsoundstatus;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 2);
}


Pam0x271271* Pam0x271271::set_pam_rearsystemworksts(
    Pam_0x271_271::Pam_rearsystemworkstsType pam_rearsystemworksts) {
  pam_rearsystemworksts_ = pam_rearsystemworksts;
  return this;
 }

// config detail: {'name': 'PAM_RearSysTemWorkSts', 'enum': {0: 'PAM_REARSYSTEMWORKSTS_DISABLE', 1: 'PAM_REARSYSTEMWORKSTS_ENABLE', 2: 'PAM_REARSYSTEMWORKSTS_ACTIVE', 3: 'PAM_REARSYSTEMWORKSTS_FAILED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 51, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_rearsystemworksts(uint8_t* data,
    Pam_0x271_271::Pam_rearsystemworkstsType pam_rearsystemworksts) {
  int x = pam_rearsystemworksts;

  Byte to_set(data + 6);
  to_set.set_value(x, 2, 2);
}


Pam0x271271* Pam0x271271::set_pam_frontsystemworksts(
    Pam_0x271_271::Pam_frontsystemworkstsType pam_frontsystemworksts) {
  pam_frontsystemworksts_ = pam_frontsystemworksts;
  return this;
 }

// config detail: {'name': 'PAM_FrontSysTemWorkSts', 'enum': {0: 'PAM_FRONTSYSTEMWORKSTS_DISABLE', 1: 'PAM_FRONTSYSTEMWORKSTS_ENABLE', 2: 'PAM_FRONTSYSTEMWORKSTS_ACTIVE', 3: 'PAM_FRONTSYSTEMWORKSTS_FAILED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_frontsystemworksts(uint8_t* data,
    Pam_0x271_271::Pam_frontsystemworkstsType pam_frontsystemworksts) {
  int x = pam_frontsystemworksts;

  Byte to_set(data + 6);
  to_set.set_value(x, 4, 2);
}


Pam0x271271* Pam0x271271::set_pam_parkingbarsts(
    Pam_0x271_271::Pam_parkingbarstsType pam_parkingbarsts) {
  pam_parkingbarsts_ = pam_parkingbarsts;
  return this;
 }

// config detail: {'name': 'PAM_ParkingBarSts', 'enum': {0: 'PAM_PARKINGBARSTS_DISPLAY_OFF', 1: 'PAM_PARKINGBARSTS_DISPLAY_ON_FROM_TOP_TO_BOTTOM', 2: 'PAM_PARKINGBARSTS_DISPLAY_ON_FROM_BOTTOM_TO_TOP'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_parkingbarsts(uint8_t* data,
    Pam_0x271_271::Pam_parkingbarstsType pam_parkingbarsts) {
  int x = pam_parkingbarsts;

  Byte to_set(data + 6);
  to_set.set_value(x, 6, 2);
}


Pam0x271271* Pam0x271271::set_pam_chimecmdsoundcadence(
    Pam_0x271_271::Pam_chimecmdsoundcadenceType pam_chimecmdsoundcadence) {
  pam_chimecmdsoundcadence_ = pam_chimecmdsoundcadence;
  return this;
 }

// config detail: {'name': 'PAM_ChimeCmdSoundCadence', 'enum': {0: 'PAM_CHIMECMDSOUNDCADENCE_TONE_0_NO_WARNING', 1: 'PAM_CHIMECMDSOUNDCADENCE_TONE_1', 2: 'PAM_CHIMECMDSOUNDCADENCE_TONE_2', 3: 'PAM_CHIMECMDSOUNDCADENCE_TONE_3', 4: 'PAM_CHIMECMDSOUNDCADENCE_TONE_4', 5: 'PAM_CHIMECMDSOUNDCADENCE_TONE_5', 6: 'PAM_CHIMECMDSOUNDCADENCE_TONE_6', 7: 'PAM_CHIMECMDSOUNDCADENCE_TONE_7', 8: 'PAM_CHIMECMDSOUNDCADENCE_RPA_NOK', 9: 'PAM_CHIMECMDSOUNDCADENCE_RPA_OK', 10: 'PAM_CHIMECMDSOUNDCADENCE_TONE_10_PAM_FAILED_TONE', 11: 'PAM_CHIMECMDSOUNDCADENCE_TONE_11_PAM_WARNING_TONE', 12: 'PAM_CHIMECMDSOUNDCADENCE_TONE_12_PAM_SUCCESSFUL_TONE'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|14]', 'bit': 59, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Pam0x271271::set_p_pam_chimecmdsoundcadence(uint8_t* data,
    Pam_0x271_271::Pam_chimecmdsoundcadenceType pam_chimecmdsoundcadence) {
  int x = pam_chimecmdsoundcadence;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 4);
}


Pam0x271271* Pam0x271271::set_pam_parkingbarpercent(
    Pam_0x271_271::Pam_parkingbarpercentType pam_parkingbarpercent) {
  pam_parkingbarpercent_ = pam_parkingbarpercent;
  return this;
 }

// config detail: {'name': 'PAM_ParkingBarpercent', 'enum': {0: 'PAM_PARKINGBARPERCENT_0PERCENT', 1: 'PAM_PARKINGBARPERCENT_1PERCENT', 2: 'PAM_PARKINGBARPERCENT_2PERCENT', 3: 'PAM_PARKINGBARPERCENT_3PERCENT', 4: 'PAM_PARKINGBARPERCENT_4PERCENT', 5: 'PAM_PARKINGBARPERCENT_5PERCENT', 6: 'PAM_PARKINGBARPERCENT_6PERCENT', 7: 'PAM_PARKINGBARPERCENT_7PERCENT', 8: 'PAM_PARKINGBARPERCENT_8PERCENT', 9: 'PAM_PARKINGBARPERCENT_9PERCENT', 10: 'PAM_PARKINGBARPERCENT_10PERCENT', 11: 'PAM_PARKINGBARPERCENT_11PERCENT', 12: 'PAM_PARKINGBARPERCENT_12PERCENT', 13: 'PAM_PARKINGBARPERCENT_13PERCENT', 14: 'PAM_PARKINGBARPERCENT_14PERCENT', 15: 'PAM_PARKINGBARPERCENT_15PERCENT', 16: 'PAM_PARKINGBARPERCENT_16PERCENT', 17: 'PAM_PARKINGBARPERCENT_17PERCENT', 18: 'PAM_PARKINGBARPERCENT_18PERCENT', 19: 'PAM_PARKINGBARPERCENT_19PERCENT', 20: 'PAM_PARKINGBARPERCENT_20PERCENT', 21: 'PAM_PARKINGBARPERCENT_21PERCENT', 22: 'PAM_PARKINGBARPERCENT_22PERCENT', 23: 'PAM_PARKINGBARPERCENT_23PERCENT', 24: 'PAM_PARKINGBARPERCENT_24PERCENT', 25: 'PAM_PARKINGBARPERCENT_25PERCENT', 26: 'PAM_PARKINGBARPERCENT_26PERCENT', 27: 'PAM_PARKINGBARPERCENT_27PERCENT', 28: 'PAM_PARKINGBARPERCENT_28PERCENT', 29: 'PAM_PARKINGBARPERCENT_29PERCENT', 30: 'PAM_PARKINGBARPERCENT_30PERCENT', 31: 'PAM_PARKINGBARPERCENT_31PERCENT', 32: 'PAM_PARKINGBARPERCENT_32PERCENT', 33: 'PAM_PARKINGBARPERCENT_33PERCENT', 34: 'PAM_PARKINGBARPERCENT_34PERCENT', 35: 'PAM_PARKINGBARPERCENT_35PERCENT', 36: 'PAM_PARKINGBARPERCENT_36PERCENT', 37: 'PAM_PARKINGBARPERCENT_37PERCENT', 38: 'PAM_PARKINGBARPERCENT_38PERCENT', 39: 'PAM_PARKINGBARPERCENT_39PERCENT', 40: 'PAM_PARKINGBARPERCENT_40PERCENT', 41: 'PAM_PARKINGBARPERCENT_41PERCENT', 42: 'PAM_PARKINGBARPERCENT_42PERCENT', 43: 'PAM_PARKINGBARPERCENT_43PERCENT', 44: 'PAM_PARKINGBARPERCENT_44PERCENT', 45: 'PAM_PARKINGBARPERCENT_45PERCENT', 46: 'PAM_PARKINGBARPERCENT_46PERCENT', 47: 'PAM_PARKINGBARPERCENT_47PERCENT', 48: 'PAM_PARKINGBARPERCENT_48PERCENT', 49: 'PAM_PARKINGBARPERCENT_49PERCENT', 50: 'PAM_PARKINGBARPERCENT_50PERCENT', 51: 'PAM_PARKINGBARPERCENT_51PERCENT', 52: 'PAM_PARKINGBARPERCENT_52PERCENT', 53: 'PAM_PARKINGBARPERCENT_53PERCENT', 54: 'PAM_PARKINGBARPERCENT_54PERCENT', 55: 'PAM_PARKINGBARPERCENT_55PERCENT', 56: 'PAM_PARKINGBARPERCENT_56PERCENT', 57: 'PAM_PARKINGBARPERCENT_57PERCENT', 58: 'PAM_PARKINGBARPERCENT_58PERCENT', 59: 'PAM_PARKINGBARPERCENT_59PERCENT', 60: 'PAM_PARKINGBARPERCENT_60PERCENT', 61: 'PAM_PARKINGBARPERCENT_61PERCENT', 62: 'PAM_PARKINGBARPERCENT_62PERCENT', 63: 'PAM_PARKINGBARPERCENT_63PERCENT', 64: 'PAM_PARKINGBARPERCENT_64PERCENT', 65: 'PAM_PARKINGBARPERCENT_65PERCENT', 66: 'PAM_PARKINGBARPERCENT_66PERCENT', 67: 'PAM_PARKINGBARPERCENT_67PERCENT', 68: 'PAM_PARKINGBARPERCENT_68PERCENT', 69: 'PAM_PARKINGBARPERCENT_69PERCENT', 70: 'PAM_PARKINGBARPERCENT_70PERCENT', 71: 'PAM_PARKINGBARPERCENT_71PERCENT', 72: 'PAM_PARKINGBARPERCENT_72PERCENT', 73: 'PAM_PARKINGBARPERCENT_73PERCENT', 74: 'PAM_PARKINGBARPERCENT_74PERCENT', 75: 'PAM_PARKINGBARPERCENT_75PERCENT', 76: 'PAM_PARKINGBARPERCENT_76PERCENT', 77: 'PAM_PARKINGBARPERCENT_77PERCENT', 78: 'PAM_PARKINGBARPERCENT_78PERCENT', 79: 'PAM_PARKINGBARPERCENT_79PERCENT', 80: 'PAM_PARKINGBARPERCENT_80PERCENT', 81: 'PAM_PARKINGBARPERCENT_81PERCENT', 82: 'PAM_PARKINGBARPERCENT_82PERCENT', 83: 'PAM_PARKINGBARPERCENT_83PERCENT', 84: 'PAM_PARKINGBARPERCENT_84PERCENT', 85: 'PAM_PARKINGBARPERCENT_85PERCENT', 86: 'PAM_PARKINGBARPERCENT_86PERCENT', 87: 'PAM_PARKINGBARPERCENT_87PERCENT', 88: 'PAM_PARKINGBARPERCENT_88PERCENT', 89: 'PAM_PARKINGBARPERCENT_89PERCENT', 90: 'PAM_PARKINGBARPERCENT_90PERCENT', 91: 'PAM_PARKINGBARPERCENT_91PERCENT', 92: 'PAM_PARKINGBARPERCENT_92PERCENT', 93: 'PAM_PARKINGBARPERCENT_93PERCENT', 94: 'PAM_PARKINGBARPERCENT_94PERCENT', 95: 'PAM_PARKINGBARPERCENT_95PERCENT', 96: 'PAM_PARKINGBARPERCENT_96PERCENT', 97: 'PAM_PARKINGBARPERCENT_97PERCENT', 98: 'PAM_PARKINGBARPERCENT_98PERCENT', 99: 'PAM_PARKINGBARPERCENT_99PERCENT', 100: 'PAM_PARKINGBARPERCENT_100PERCENT', 101: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 102: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 103: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 104: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 105: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 106: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 107: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 108: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 109: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 110: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 111: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 112: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 113: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 114: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 115: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 116: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 117: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 118: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 119: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 120: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 121: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 122: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 123: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 124: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 125: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 126: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA', 127: 'PAM_PARKINGBARPERCENT_NO_DISPLAY_APA'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|100]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': '%'}
void Pam0x271271::set_p_pam_parkingbarpercent(uint8_t* data,
    Pam_0x271_271::Pam_parkingbarpercentType pam_parkingbarpercent) {
  int x = pam_parkingbarpercent;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
