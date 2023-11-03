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

#include "modules/canbus/vehicle/cx75/protocol/ipm_0x245_245.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Ipm0x245245::ID = 0x245;

// public
Ipm0x245245::Ipm0x245245() { Reset(); }

uint32_t Ipm0x245245::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Ipm0x245245::UpdateData(uint8_t* data) {
  set_p_ipm_laneassit_torquereqstatus(data, ipm_laneassit_torquereqstatus_);
  set_p_ipm_laneassit_torquevalidity(data, ipm_laneassit_torquevalidity_);
  set_p_ipm_laneassit_sensitivity(data, ipm_laneassit_sensitivity_);
  set_p_ipm_laneassit_warningaudio(data, ipm_laneassit_warningaudio_);
  set_p_ipm_laneassit_leftvisualization(data, ipm_laneassit_leftvisualization_);
  set_p_ipm_laneassit_rightvisualization(data, ipm_laneassit_rightvisualization_);
  set_p_ipm_slaspdlimitunits(data, ipm_slaspdlimitunits_);
  set_p_ipm_slastate(data, ipm_slastate_);
  set_p_ipm_laneassit_status(data, ipm_laneassit_status_);
  set_p_ipm_slaspdlimit(data, ipm_slaspdlimit_);
  set_p_ipm_hma_status(data, ipm_hma_status_);
  set_p_ipm_tjaica_status(data, ipm_tjaica_status_);
  set_p_rolling_counter_0x245(data, rolling_counter_0x245_);
  set_p_ipm_ldwwarningtype(data, ipm_ldwwarningtype_);
  set_p_ipm_laneassit_handsonreq(data, ipm_laneassit_handsonreq_);
  set_p_ipm_hma_highbeamreq(data, ipm_hma_highbeamreq_);
  set_p_ipm_laneassit_torquereq(data, ipm_laneassit_torquereq_);
  set_p_ipm_laneassittype(data, ipm_laneassittype_);
  set_p_ipm_laneassit_warningvibration(data, ipm_laneassit_warningvibration_);
  set_p_checksum_0x245(data, checksum_0x245_);
}

void Ipm0x245245::Reset() {
  // TODO you should check this manually
  ipm_laneassit_torquereqstatus_ = Ipm_0x245_245::IPM_LANEASSIT_TORQUEREQSTATUS_NO_REQUEST;
  ipm_laneassit_torquevalidity_ = Ipm_0x245_245::IPM_LANEASSIT_TORQUEVALIDITY_VALID;
  ipm_laneassit_sensitivity_ = Ipm_0x245_245::IPM_LANEASSIT_SENSITIVITY_HIGHSENSITIVITY;
  ipm_laneassit_warningaudio_ = Ipm_0x245_245::IPM_LANEASSIT_WARNINGAUDIO_INACTIVE;
  ipm_laneassit_leftvisualization_ = Ipm_0x245_245::IPM_LANEASSIT_LEFTVISUALIZATION_NO_DISPLAY;
  ipm_laneassit_rightvisualization_ = Ipm_0x245_245::IPM_LANEASSIT_RIGHTVISUALIZATION_NO_DISPLAY;
  ipm_slaspdlimitunits_ = Ipm_0x245_245::IPM_SLASPDLIMITUNITS_UNKNOWN;
  ipm_slastate_ = Ipm_0x245_245::IPM_SLASTATE_OFF;
  ipm_laneassit_status_ = Ipm_0x245_245::IPM_LANEASSIT_STATUS_OFF;
  ipm_slaspdlimit_ = Ipm_0x245_245::IPM_SLASPDLIMIT_NO_DISPLAY;
  ipm_hma_status_ = Ipm_0x245_245::IPM_HMA_STATUS_OFF;
  ipm_tjaica_status_ = Ipm_0x245_245::IPM_TJAICA_STATUS_INACTIVE;
  rolling_counter_0x245_ = 0;
  ipm_ldwwarningtype_ = Ipm_0x245_245::IPM_LDWWARNINGTYPE_AUDIBLE_WARNING_AND_WHEELSTEER_SHAKE;
  ipm_laneassit_handsonreq_ = Ipm_0x245_245::IPM_LANEASSIT_HANDSONREQ_INACTIVE;
  ipm_hma_highbeamreq_ = Ipm_0x245_245::IPM_HMA_HIGHBEAMREQ_NO_REQUEST;
  checksum_0x245_ = 0;
  ipm_laneassit_torquereq_ = 0;
  ipm_laneassittype_ = Ipm_0x245_245::IPM_LANEASSITTYPE_UNAVAILABLE;
  ipm_laneassit_warningvibration_ = Ipm_0x245_245::IPM_LANEASSIT_WARNINGVIBRATION_INACTIVE;
}

Ipm0x245245* Ipm0x245245::set_ipm_laneassit_torquereqstatus(
    Ipm_0x245_245::Ipm_laneassit_torquereqstatusType ipm_laneassit_torquereqstatus) {
  ipm_laneassit_torquereqstatus_ = ipm_laneassit_torquereqstatus;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssit_TorqueReqStatus', 'enum': {0: 'IPM_LANEASSIT_TORQUEREQSTATUS_NO_REQUEST', 1: 'IPM_LANEASSIT_TORQUEREQSTATUS_TORQUE_REQUEST', 2: 'IPM_LANEASSIT_TORQUEREQSTATUS_RESERVED', 3: 'IPM_LANEASSIT_TORQUEREQSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_laneassit_torquereqstatus(uint8_t* data,
    Ipm_0x245_245::Ipm_laneassit_torquereqstatusType ipm_laneassit_torquereqstatus) {
  int x = ipm_laneassit_torquereqstatus;

  Byte to_set(data + 1);
  to_set.set_value(x, 2, 2);
}


Ipm0x245245* Ipm0x245245::set_ipm_laneassit_torquevalidity(
    Ipm_0x245_245::Ipm_laneassit_torquevalidityType ipm_laneassit_torquevalidity) {
  ipm_laneassit_torquevalidity_ = ipm_laneassit_torquevalidity;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssit_TorqueValidity', 'enum': {0: 'IPM_LANEASSIT_TORQUEVALIDITY_VALID', 1: 'IPM_LANEASSIT_TORQUEVALIDITY_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_laneassit_torquevalidity(uint8_t* data,
    Ipm_0x245_245::Ipm_laneassit_torquevalidityType ipm_laneassit_torquevalidity) {
  int x = ipm_laneassit_torquevalidity;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 1);
}


Ipm0x245245* Ipm0x245245::set_ipm_laneassit_sensitivity(
    Ipm_0x245_245::Ipm_laneassit_sensitivityType ipm_laneassit_sensitivity) {
  ipm_laneassit_sensitivity_ = ipm_laneassit_sensitivity;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssit_Sensitivity', 'enum': {0: 'IPM_LANEASSIT_SENSITIVITY_HIGHSENSITIVITY', 1: 'IPM_LANEASSIT_SENSITIVITY_LOWSENSITIVITY'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_laneassit_sensitivity(uint8_t* data,
    Ipm_0x245_245::Ipm_laneassit_sensitivityType ipm_laneassit_sensitivity) {
  int x = ipm_laneassit_sensitivity;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 1);
}


Ipm0x245245* Ipm0x245245::set_ipm_laneassit_warningaudio(
    Ipm_0x245_245::Ipm_laneassit_warningaudioType ipm_laneassit_warningaudio) {
  ipm_laneassit_warningaudio_ = ipm_laneassit_warningaudio;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssit_WarningAudio', 'enum': {0: 'IPM_LANEASSIT_WARNINGAUDIO_INACTIVE', 1: 'IPM_LANEASSIT_WARNINGAUDIO_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_laneassit_warningaudio(uint8_t* data,
    Ipm_0x245_245::Ipm_laneassit_warningaudioType ipm_laneassit_warningaudio) {
  int x = ipm_laneassit_warningaudio;

  Byte to_set(data + 2);
  to_set.set_value(x, 1, 1);
}


Ipm0x245245* Ipm0x245245::set_ipm_laneassit_leftvisualization(
    Ipm_0x245_245::Ipm_laneassit_leftvisualizationType ipm_laneassit_leftvisualization) {
  ipm_laneassit_leftvisualization_ = ipm_laneassit_leftvisualization;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssit_LeftVisualization', 'enum': {0: 'IPM_LANEASSIT_LEFTVISUALIZATION_NO_DISPLAY', 1: 'IPM_LANEASSIT_LEFTVISUALIZATION_LINE_TRACKING', 2: 'IPM_LANEASSIT_LEFTVISUALIZATION_INTERVENTION', 3: 'IPM_LANEASSIT_LEFTVISUALIZATION_WARNING'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_laneassit_leftvisualization(uint8_t* data,
    Ipm_0x245_245::Ipm_laneassit_leftvisualizationType ipm_laneassit_leftvisualization) {
  int x = ipm_laneassit_leftvisualization;

  Byte to_set(data + 2);
  to_set.set_value(x, 2, 2);
}


Ipm0x245245* Ipm0x245245::set_ipm_laneassit_rightvisualization(
    Ipm_0x245_245::Ipm_laneassit_rightvisualizationType ipm_laneassit_rightvisualization) {
  ipm_laneassit_rightvisualization_ = ipm_laneassit_rightvisualization;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssit_RightVisualization', 'enum': {0: 'IPM_LANEASSIT_RIGHTVISUALIZATION_NO_DISPLAY', 1: 'IPM_LANEASSIT_RIGHTVISUALIZATION_LINE_TRACKING', 2: 'IPM_LANEASSIT_RIGHTVISUALIZATION_INTERVENTION', 3: 'IPM_LANEASSIT_RIGHTVISUALIZATION_WARNING'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_laneassit_rightvisualization(uint8_t* data,
    Ipm_0x245_245::Ipm_laneassit_rightvisualizationType ipm_laneassit_rightvisualization) {
  int x = ipm_laneassit_rightvisualization;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 2);
}


Ipm0x245245* Ipm0x245245::set_ipm_slaspdlimitunits(
    Ipm_0x245_245::Ipm_slaspdlimitunitsType ipm_slaspdlimitunits) {
  ipm_slaspdlimitunits_ = ipm_slaspdlimitunits;
  return this;
 }

// config detail: {'name': 'IPM_SLASpdlimitUnits', 'enum': {0: 'IPM_SLASPDLIMITUNITS_UNKNOWN', 1: 'IPM_SLASPDLIMITUNITS_KMH', 2: 'IPM_SLASPDLIMITUNITS_MPH', 3: 'IPM_SLASPDLIMITUNITS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_slaspdlimitunits(uint8_t* data,
    Ipm_0x245_245::Ipm_slaspdlimitunitsType ipm_slaspdlimitunits) {
  int x = ipm_slaspdlimitunits;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 2);
}


Ipm0x245245* Ipm0x245245::set_ipm_slastate(
    Ipm_0x245_245::Ipm_slastateType ipm_slastate) {
  ipm_slastate_ = ipm_slastate;
  return this;
 }

// config detail: {'name': 'IPM_SLAState', 'enum': {0: 'IPM_SLASTATE_OFF', 1: 'IPM_SLASTATE_FUSION_MODE', 2: 'IPM_SLASTATE_VISION_MODE', 3: 'IPM_SLASTATE_NAVIGATION_ONLY_MODE', 4: 'IPM_SLASTATE_DEFECT'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_slastate(uint8_t* data,
    Ipm_0x245_245::Ipm_slastateType ipm_slastate) {
  int x = ipm_slastate;

  Byte to_set(data + 3);
  to_set.set_value(x, 2, 3);
}


Ipm0x245245* Ipm0x245245::set_ipm_laneassit_status(
    Ipm_0x245_245::Ipm_laneassit_statusType ipm_laneassit_status) {
  ipm_laneassit_status_ = ipm_laneassit_status;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssit_Status', 'enum': {0: 'IPM_LANEASSIT_STATUS_OFF', 1: 'IPM_LANEASSIT_STATUS_STANDBY', 2: 'IPM_LANEASSIT_STATUS_LDW_ACTIVE_LKA_STANDBY', 3: 'IPM_LANEASSIT_STATUS_LDW_STANDBY_LKA_ACTIVE', 4: 'IPM_LANEASSIT_STATUS_ACTIVE', 5: 'IPM_LANEASSIT_STATUS_FAULT', 6: 'IPM_LANEASSIT_STATUS_CAMERA_BLOCKED', 7: 'IPM_LANEASSIT_STATUS_CAMERA_UNCALIBRATED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_laneassit_status(uint8_t* data,
    Ipm_0x245_245::Ipm_laneassit_statusType ipm_laneassit_status) {
  int x = ipm_laneassit_status;

  Byte to_set(data + 3);
  to_set.set_value(x, 5, 3);
}


Ipm0x245245* Ipm0x245245::set_ipm_slaspdlimit(
    Ipm_0x245_245::Ipm_slaspdlimitType ipm_slaspdlimit) {
  ipm_slaspdlimit_ = ipm_slaspdlimit;
  return this;
 }

// config detail: {'name': 'IPM_SLASpdlimit', 'enum': {0: 'IPM_SLASPDLIMIT_NO_DISPLAY', 1: 'IPM_SLASPDLIMIT_SPL_CANCELLED', 2: 'IPM_SLASPDLIMIT_SPL_5', 3: 'IPM_SLASPDLIMIT_SPL_10', 4: 'IPM_SLASPDLIMIT_SPL_15', 5: 'IPM_SLASPDLIMIT_SPL_20', 6: 'IPM_SLASPDLIMIT_SPL_25', 7: 'IPM_SLASPDLIMIT_SPL_30', 8: 'IPM_SLASPDLIMIT_SPL_35', 9: 'IPM_SLASPDLIMIT_SPL_40', 10: 'IPM_SLASPDLIMIT_SPL_45', 11: 'IPM_SLASPDLIMIT_SPL_50', 12: 'IPM_SLASPDLIMIT_SPL_55', 13: 'IPM_SLASPDLIMIT_SPL_60', 14: 'IPM_SLASPDLIMIT_SPL_65', 15: 'IPM_SLASPDLIMIT_SPL_70', 16: 'IPM_SLASPDLIMIT_SPL_75', 17: 'IPM_SLASPDLIMIT_SPL_80', 18: 'IPM_SLASPDLIMIT_SPL_85', 19: 'IPM_SLASPDLIMIT_SPL_90', 20: 'IPM_SLASPDLIMIT_SPL_95', 21: 'IPM_SLASPDLIMIT_SPL_100', 22: 'IPM_SLASPDLIMIT_SPL_105', 23: 'IPM_SLASPDLIMIT_SPL_110', 24: 'IPM_SLASPDLIMIT_SPL_115', 25: 'IPM_SLASPDLIMIT_SPL_120', 26: 'IPM_SLASPDLIMIT_SPL_125', 27: 'IPM_SLASPDLIMIT_SPL_130'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_slaspdlimit(uint8_t* data,
    Ipm_0x245_245::Ipm_slaspdlimitType ipm_slaspdlimit) {
  int x = ipm_slaspdlimit;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}


Ipm0x245245* Ipm0x245245::set_ipm_hma_status(
    Ipm_0x245_245::Ipm_hma_statusType ipm_hma_status) {
  ipm_hma_status_ = ipm_hma_status;
  return this;
 }

// config detail: {'name': 'IPM_HMA_Status', 'enum': {0: 'IPM_HMA_STATUS_OFF', 1: 'IPM_HMA_STATUS_PASSIVE', 2: 'IPM_HMA_STATUS_STANDBY', 3: 'IPM_HMA_STATUS_ACTIVE', 4: 'IPM_HMA_STATUS_FAULT', 5: 'IPM_HMA_STATUS_CAMERA_BLOCKED', 6: 'IPM_HMA_STATUS_RESERVED', 7: 'IPM_HMA_STATUS_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_hma_status(uint8_t* data,
    Ipm_0x245_245::Ipm_hma_statusType ipm_hma_status) {
  int x = ipm_hma_status;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 3);
}


Ipm0x245245* Ipm0x245245::set_ipm_tjaica_status(
    Ipm_0x245_245::Ipm_tjaica_statusType ipm_tjaica_status) {
  ipm_tjaica_status_ = ipm_tjaica_status;
  return this;
 }

// config detail: {'name': 'IPM_TJAICA_Status', 'enum': {0: 'IPM_TJAICA_STATUS_INACTIVE', 1: 'IPM_TJAICA_STATUS_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_tjaica_status(uint8_t* data,
    Ipm_0x245_245::Ipm_tjaica_statusType ipm_tjaica_status) {
  int x = ipm_tjaica_status;

  Byte to_set(data + 5);
  to_set.set_value(x, 7, 1);
}


Ipm0x245245* Ipm0x245245::set_rolling_counter_0x245(
    int rolling_counter_0x245) {
  rolling_counter_0x245_ = rolling_counter_0x245;
  return this;
 }

// config detail: {'name': 'Rolling_counter_0x245', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_rolling_counter_0x245(uint8_t* data,
    int rolling_counter_0x245) {
  rolling_counter_0x245_++;
  rolling_counter_0x245_=rolling_counter_0x245_<=15?rolling_counter_0x245_:0;
  rolling_counter_0x245_ = ProtocolData::BoundedValue(0, 15, rolling_counter_0x245_);
  int x = rolling_counter_0x245_;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}


Ipm0x245245* Ipm0x245245::set_ipm_ldwwarningtype(
    Ipm_0x245_245::Ipm_ldwwarningtypeType ipm_ldwwarningtype) {
  ipm_ldwwarningtype_ = ipm_ldwwarningtype;
  return this;
 }

// config detail: {'name': 'IPM_LDWWarningType', 'enum': {0: 'IPM_LDWWARNINGTYPE_AUDIBLE_WARNING_AND_WHEELSTEER_SHAKE', 1: 'IPM_LDWWARNINGTYPE_AUDIBLE_WARNING', 2: 'IPM_LDWWARNINGTYPE_WHEELSTEER_SHAKE', 3: 'IPM_LDWWARNINGTYPE_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_ldwwarningtype(uint8_t* data,
    Ipm_0x245_245::Ipm_ldwwarningtypeType ipm_ldwwarningtype) {
  int x = ipm_ldwwarningtype;

  Byte to_set(data + 6);
  to_set.set_value(x, 4, 2);
}


Ipm0x245245* Ipm0x245245::set_ipm_laneassit_handsonreq(
    Ipm_0x245_245::Ipm_laneassit_handsonreqType ipm_laneassit_handsonreq) {
  ipm_laneassit_handsonreq_ = ipm_laneassit_handsonreq;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssit_HandsonReq', 'enum': {0: 'IPM_LANEASSIT_HANDSONREQ_INACTIVE', 1: 'IPM_LANEASSIT_HANDSONREQ_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_laneassit_handsonreq(uint8_t* data,
    Ipm_0x245_245::Ipm_laneassit_handsonreqType ipm_laneassit_handsonreq) {
  int x = ipm_laneassit_handsonreq;

  Byte to_set(data + 6);
  to_set.set_value(x, 6, 1);
}


Ipm0x245245* Ipm0x245245::set_ipm_hma_highbeamreq(
    Ipm_0x245_245::Ipm_hma_highbeamreqType ipm_hma_highbeamreq) {
  ipm_hma_highbeamreq_ = ipm_hma_highbeamreq;
  return this;
 }

// config detail: {'name': 'IPM_HMA_HighbeamReq', 'enum': {0: 'IPM_HMA_HIGHBEAMREQ_NO_REQUEST', 1: 'IPM_HMA_HIGHBEAMREQ_HIGH_BEAM_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_hma_highbeamreq(uint8_t* data,
    Ipm_0x245_245::Ipm_hma_highbeamreqType ipm_hma_highbeamreq) {
  int x = ipm_hma_highbeamreq;

  Byte to_set(data + 6);
  to_set.set_value(x, 7, 1);
}


Ipm0x245245* Ipm0x245245::set_checksum_0x245(
    int checksum_0x245) {
  checksum_0x245_ = checksum_0x245;
  return this;
 }

// config detail: {'name': 'Checksum_0x245', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_checksum_0x245(uint8_t* data,
    int checksum_0x245) {
  checksum_0x245 = (data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]);
  checksum_0x245 = ProtocolData::BoundedValue(0, 255, checksum_0x245);
  int x = checksum_0x245;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Ipm0x245245* Ipm0x245245::set_ipm_laneassit_torquereq(
    double ipm_laneassit_torquereq) {
  ipm_laneassit_torquereq_ = ipm_laneassit_torquereq;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssit_TorqueReq', 'enum': {2047: 'IPM_LANEASSIT_TORQUEREQ_INVALID'}, 'precision': 0.01, 'len': 11, 'is_signed_var': False, 'offset': -10.24, 'physical_range': '[-10.24|10.22]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
void Ipm0x245245::set_p_ipm_laneassit_torquereq(uint8_t* data,
    double ipm_laneassit_torquereq) {
  int x = (ipm_laneassit_torquereq - -10.240000) / 0.010000;
  uint8_t t = 0;

  t = x & 0x7;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 5, 3);
  x >>= 3;

  t = x & 0xFF;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 8);
}


Ipm0x245245* Ipm0x245245::set_ipm_laneassittype(
    Ipm_0x245_245::Ipm_laneassittypeType ipm_laneassittype) {
  ipm_laneassittype_ = ipm_laneassittype;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssitType', 'enum': {0: 'IPM_LANEASSITTYPE_UNAVAILABLE', 1: 'IPM_LANEASSITTYPE_LDW', 2: 'IPM_LANEASSITTYPE_LKA', 3: 'IPM_LANEASSITTYPE_LDW_LKA', 4: 'IPM_LANEASSITTYPE_TJA_ICA', 5: 'IPM_LANEASSITTYPE_TJA_ICA_LDW', 6: 'IPM_LANEASSITTYPE_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_laneassittype(uint8_t* data,
    Ipm_0x245_245::Ipm_laneassittypeType ipm_laneassittype) {
  int x = ipm_laneassittype;
  uint8_t t = 0;

  t = x & 0x3;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = x & 0x1;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 1);
}


Ipm0x245245* Ipm0x245245::set_ipm_laneassit_warningvibration(
    Ipm_0x245_245::Ipm_laneassit_warningvibrationType ipm_laneassit_warningvibration) {
  ipm_laneassit_warningvibration_ = ipm_laneassit_warningvibration;
  return this;
 }

// config detail: {'name': 'IPM_LaneAssit_WarningVibration', 'enum': {0: 'IPM_LANEASSIT_WARNINGVIBRATION_INACTIVE', 1: 'IPM_LANEASSIT_WARNINGVIBRATION_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Ipm0x245245::set_p_ipm_laneassit_warningvibration(uint8_t* data,
    Ipm_0x245_245::Ipm_laneassit_warningvibrationType ipm_laneassit_warningvibration) {
  int x = ipm_laneassit_warningvibration;

  Byte to_set(data + 1);
  to_set.set_value(x, 1, 1);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
