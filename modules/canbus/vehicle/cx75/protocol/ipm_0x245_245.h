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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_IPM_0X245_245_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_IPM_0X245_245_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Ipm0x245245 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Ipm0x245245();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'IPM_LaneAssit_TorqueReqStatus', 'enum': {0: 'IPM_LANEASSIT_TORQUEREQSTATUS_NO_REQUEST', 1: 'IPM_LANEASSIT_TORQUEREQSTATUS_TORQUE_REQUEST', 2: 'IPM_LANEASSIT_TORQUEREQSTATUS_RESERVED', 3: 'IPM_LANEASSIT_TORQUEREQSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_laneassit_torquereqstatus(Ipm_0x245_245::Ipm_laneassit_torquereqstatusType ipm_laneassit_torquereqstatus);

  // config detail: {'name': 'IPM_LaneAssit_TorqueValidity', 'enum': {0: 'IPM_LANEASSIT_TORQUEVALIDITY_VALID', 1: 'IPM_LANEASSIT_TORQUEVALIDITY_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_laneassit_torquevalidity(Ipm_0x245_245::Ipm_laneassit_torquevalidityType ipm_laneassit_torquevalidity);

  // config detail: {'name': 'IPM_LaneAssit_Sensitivity', 'enum': {0: 'IPM_LANEASSIT_SENSITIVITY_HIGHSENSITIVITY', 1: 'IPM_LANEASSIT_SENSITIVITY_LOWSENSITIVITY'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_laneassit_sensitivity(Ipm_0x245_245::Ipm_laneassit_sensitivityType ipm_laneassit_sensitivity);

  // config detail: {'name': 'IPM_LaneAssit_WarningAudio', 'enum': {0: 'IPM_LANEASSIT_WARNINGAUDIO_INACTIVE', 1: 'IPM_LANEASSIT_WARNINGAUDIO_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_laneassit_warningaudio(Ipm_0x245_245::Ipm_laneassit_warningaudioType ipm_laneassit_warningaudio);

  // config detail: {'name': 'IPM_LaneAssit_LeftVisualization', 'enum': {0: 'IPM_LANEASSIT_LEFTVISUALIZATION_NO_DISPLAY', 1: 'IPM_LANEASSIT_LEFTVISUALIZATION_LINE_TRACKING', 2: 'IPM_LANEASSIT_LEFTVISUALIZATION_INTERVENTION', 3: 'IPM_LANEASSIT_LEFTVISUALIZATION_WARNING'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_laneassit_leftvisualization(Ipm_0x245_245::Ipm_laneassit_leftvisualizationType ipm_laneassit_leftvisualization);

  // config detail: {'name': 'IPM_LaneAssit_RightVisualization', 'enum': {0: 'IPM_LANEASSIT_RIGHTVISUALIZATION_NO_DISPLAY', 1: 'IPM_LANEASSIT_RIGHTVISUALIZATION_LINE_TRACKING', 2: 'IPM_LANEASSIT_RIGHTVISUALIZATION_INTERVENTION', 3: 'IPM_LANEASSIT_RIGHTVISUALIZATION_WARNING'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_laneassit_rightvisualization(Ipm_0x245_245::Ipm_laneassit_rightvisualizationType ipm_laneassit_rightvisualization);

  // config detail: {'name': 'IPM_SLASpdlimitUnits', 'enum': {0: 'IPM_SLASPDLIMITUNITS_UNKNOWN', 1: 'IPM_SLASPDLIMITUNITS_KMH', 2: 'IPM_SLASPDLIMITUNITS_MPH', 3: 'IPM_SLASPDLIMITUNITS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_slaspdlimitunits(Ipm_0x245_245::Ipm_slaspdlimitunitsType ipm_slaspdlimitunits);

  // config detail: {'name': 'IPM_SLAState', 'enum': {0: 'IPM_SLASTATE_OFF', 1: 'IPM_SLASTATE_FUSION_MODE', 2: 'IPM_SLASTATE_VISION_MODE', 3: 'IPM_SLASTATE_NAVIGATION_ONLY_MODE', 4: 'IPM_SLASTATE_DEFECT'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_slastate(Ipm_0x245_245::Ipm_slastateType ipm_slastate);

  // config detail: {'name': 'IPM_LaneAssit_Status', 'enum': {0: 'IPM_LANEASSIT_STATUS_OFF', 1: 'IPM_LANEASSIT_STATUS_STANDBY', 2: 'IPM_LANEASSIT_STATUS_LDW_ACTIVE_LKA_STANDBY', 3: 'IPM_LANEASSIT_STATUS_LDW_STANDBY_LKA_ACTIVE', 4: 'IPM_LANEASSIT_STATUS_ACTIVE', 5: 'IPM_LANEASSIT_STATUS_FAULT', 6: 'IPM_LANEASSIT_STATUS_CAMERA_BLOCKED', 7: 'IPM_LANEASSIT_STATUS_CAMERA_UNCALIBRATED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_laneassit_status(Ipm_0x245_245::Ipm_laneassit_statusType ipm_laneassit_status);

  // config detail: {'name': 'IPM_SLASpdlimit', 'enum': {0: 'IPM_SLASPDLIMIT_NO_DISPLAY', 1: 'IPM_SLASPDLIMIT_SPL_CANCELLED', 2: 'IPM_SLASPDLIMIT_SPL_5', 3: 'IPM_SLASPDLIMIT_SPL_10', 4: 'IPM_SLASPDLIMIT_SPL_15', 5: 'IPM_SLASPDLIMIT_SPL_20', 6: 'IPM_SLASPDLIMIT_SPL_25', 7: 'IPM_SLASPDLIMIT_SPL_30', 8: 'IPM_SLASPDLIMIT_SPL_35', 9: 'IPM_SLASPDLIMIT_SPL_40', 10: 'IPM_SLASPDLIMIT_SPL_45', 11: 'IPM_SLASPDLIMIT_SPL_50', 12: 'IPM_SLASPDLIMIT_SPL_55', 13: 'IPM_SLASPDLIMIT_SPL_60', 14: 'IPM_SLASPDLIMIT_SPL_65', 15: 'IPM_SLASPDLIMIT_SPL_70', 16: 'IPM_SLASPDLIMIT_SPL_75', 17: 'IPM_SLASPDLIMIT_SPL_80', 18: 'IPM_SLASPDLIMIT_SPL_85', 19: 'IPM_SLASPDLIMIT_SPL_90', 20: 'IPM_SLASPDLIMIT_SPL_95', 21: 'IPM_SLASPDLIMIT_SPL_100', 22: 'IPM_SLASPDLIMIT_SPL_105', 23: 'IPM_SLASPDLIMIT_SPL_110', 24: 'IPM_SLASPDLIMIT_SPL_115', 25: 'IPM_SLASPDLIMIT_SPL_120', 26: 'IPM_SLASPDLIMIT_SPL_125', 27: 'IPM_SLASPDLIMIT_SPL_130'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_slaspdlimit(Ipm_0x245_245::Ipm_slaspdlimitType ipm_slaspdlimit);

  // config detail: {'name': 'IPM_HMA_Status', 'enum': {0: 'IPM_HMA_STATUS_OFF', 1: 'IPM_HMA_STATUS_PASSIVE', 2: 'IPM_HMA_STATUS_STANDBY', 3: 'IPM_HMA_STATUS_ACTIVE', 4: 'IPM_HMA_STATUS_FAULT', 5: 'IPM_HMA_STATUS_CAMERA_BLOCKED', 6: 'IPM_HMA_STATUS_RESERVED', 7: 'IPM_HMA_STATUS_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_hma_status(Ipm_0x245_245::Ipm_hma_statusType ipm_hma_status);

  // config detail: {'name': 'IPM_TJAICA_Status', 'enum': {0: 'IPM_TJAICA_STATUS_INACTIVE', 1: 'IPM_TJAICA_STATUS_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_tjaica_status(Ipm_0x245_245::Ipm_tjaica_statusType ipm_tjaica_status);

  // config detail: {'name': 'Rolling_counter_0x245', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_rolling_counter_0x245(int rolling_counter_0x245);

  // config detail: {'name': 'IPM_LDWWarningType', 'enum': {0: 'IPM_LDWWARNINGTYPE_AUDIBLE_WARNING_AND_WHEELSTEER_SHAKE', 1: 'IPM_LDWWARNINGTYPE_AUDIBLE_WARNING', 2: 'IPM_LDWWARNINGTYPE_WHEELSTEER_SHAKE', 3: 'IPM_LDWWARNINGTYPE_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_ldwwarningtype(Ipm_0x245_245::Ipm_ldwwarningtypeType ipm_ldwwarningtype);

  // config detail: {'name': 'IPM_LaneAssit_HandsonReq', 'enum': {0: 'IPM_LANEASSIT_HANDSONREQ_INACTIVE', 1: 'IPM_LANEASSIT_HANDSONREQ_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_laneassit_handsonreq(Ipm_0x245_245::Ipm_laneassit_handsonreqType ipm_laneassit_handsonreq);

  // config detail: {'name': 'IPM_HMA_HighbeamReq', 'enum': {0: 'IPM_HMA_HIGHBEAMREQ_NO_REQUEST', 1: 'IPM_HMA_HIGHBEAMREQ_HIGH_BEAM_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_hma_highbeamreq(Ipm_0x245_245::Ipm_hma_highbeamreqType ipm_hma_highbeamreq);

  // config detail: {'name': 'Checksum_0x245', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_checksum_0x245(int checksum_0x245);

  // config detail: {'name': 'IPM_LaneAssit_TorqueReq', 'enum': {2047: 'IPM_LANEASSIT_TORQUEREQ_INVALID'}, 'precision': 0.01, 'len': 11, 'is_signed_var': False, 'offset': -10.24, 'physical_range': '[-10.24|10.22]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  Ipm0x245245* set_ipm_laneassit_torquereq(double ipm_laneassit_torquereq);

  // config detail: {'name': 'IPM_LaneAssitType', 'enum': {0: 'IPM_LANEASSITTYPE_UNAVAILABLE', 1: 'IPM_LANEASSITTYPE_LDW', 2: 'IPM_LANEASSITTYPE_LKA', 3: 'IPM_LANEASSITTYPE_LDW_LKA', 4: 'IPM_LANEASSITTYPE_TJA_ICA', 5: 'IPM_LANEASSITTYPE_TJA_ICA_LDW', 6: 'IPM_LANEASSITTYPE_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_laneassittype(Ipm_0x245_245::Ipm_laneassittypeType ipm_laneassittype);

  // config detail: {'name': 'IPM_LaneAssit_WarningVibration', 'enum': {0: 'IPM_LANEASSIT_WARNINGVIBRATION_INACTIVE', 1: 'IPM_LANEASSIT_WARNINGVIBRATION_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Ipm0x245245* set_ipm_laneassit_warningvibration(Ipm_0x245_245::Ipm_laneassit_warningvibrationType ipm_laneassit_warningvibration);

 private:

  // config detail: {'name': 'IPM_LaneAssit_TorqueReqStatus', 'enum': {0: 'IPM_LANEASSIT_TORQUEREQSTATUS_NO_REQUEST', 1: 'IPM_LANEASSIT_TORQUEREQSTATUS_TORQUE_REQUEST', 2: 'IPM_LANEASSIT_TORQUEREQSTATUS_RESERVED', 3: 'IPM_LANEASSIT_TORQUEREQSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_laneassit_torquereqstatus(uint8_t* data, Ipm_0x245_245::Ipm_laneassit_torquereqstatusType ipm_laneassit_torquereqstatus);

  // config detail: {'name': 'IPM_LaneAssit_TorqueValidity', 'enum': {0: 'IPM_LANEASSIT_TORQUEVALIDITY_VALID', 1: 'IPM_LANEASSIT_TORQUEVALIDITY_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_laneassit_torquevalidity(uint8_t* data, Ipm_0x245_245::Ipm_laneassit_torquevalidityType ipm_laneassit_torquevalidity);

  // config detail: {'name': 'IPM_LaneAssit_Sensitivity', 'enum': {0: 'IPM_LANEASSIT_SENSITIVITY_HIGHSENSITIVITY', 1: 'IPM_LANEASSIT_SENSITIVITY_LOWSENSITIVITY'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_laneassit_sensitivity(uint8_t* data, Ipm_0x245_245::Ipm_laneassit_sensitivityType ipm_laneassit_sensitivity);

  // config detail: {'name': 'IPM_LaneAssit_WarningAudio', 'enum': {0: 'IPM_LANEASSIT_WARNINGAUDIO_INACTIVE', 1: 'IPM_LANEASSIT_WARNINGAUDIO_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_laneassit_warningaudio(uint8_t* data, Ipm_0x245_245::Ipm_laneassit_warningaudioType ipm_laneassit_warningaudio);

  // config detail: {'name': 'IPM_LaneAssit_LeftVisualization', 'enum': {0: 'IPM_LANEASSIT_LEFTVISUALIZATION_NO_DISPLAY', 1: 'IPM_LANEASSIT_LEFTVISUALIZATION_LINE_TRACKING', 2: 'IPM_LANEASSIT_LEFTVISUALIZATION_INTERVENTION', 3: 'IPM_LANEASSIT_LEFTVISUALIZATION_WARNING'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_laneassit_leftvisualization(uint8_t* data, Ipm_0x245_245::Ipm_laneassit_leftvisualizationType ipm_laneassit_leftvisualization);

  // config detail: {'name': 'IPM_LaneAssit_RightVisualization', 'enum': {0: 'IPM_LANEASSIT_RIGHTVISUALIZATION_NO_DISPLAY', 1: 'IPM_LANEASSIT_RIGHTVISUALIZATION_LINE_TRACKING', 2: 'IPM_LANEASSIT_RIGHTVISUALIZATION_INTERVENTION', 3: 'IPM_LANEASSIT_RIGHTVISUALIZATION_WARNING'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_laneassit_rightvisualization(uint8_t* data, Ipm_0x245_245::Ipm_laneassit_rightvisualizationType ipm_laneassit_rightvisualization);

  // config detail: {'name': 'IPM_SLASpdlimitUnits', 'enum': {0: 'IPM_SLASPDLIMITUNITS_UNKNOWN', 1: 'IPM_SLASPDLIMITUNITS_KMH', 2: 'IPM_SLASPDLIMITUNITS_MPH', 3: 'IPM_SLASPDLIMITUNITS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_slaspdlimitunits(uint8_t* data, Ipm_0x245_245::Ipm_slaspdlimitunitsType ipm_slaspdlimitunits);

  // config detail: {'name': 'IPM_SLAState', 'enum': {0: 'IPM_SLASTATE_OFF', 1: 'IPM_SLASTATE_FUSION_MODE', 2: 'IPM_SLASTATE_VISION_MODE', 3: 'IPM_SLASTATE_NAVIGATION_ONLY_MODE', 4: 'IPM_SLASTATE_DEFECT'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_slastate(uint8_t* data, Ipm_0x245_245::Ipm_slastateType ipm_slastate);

  // config detail: {'name': 'IPM_LaneAssit_Status', 'enum': {0: 'IPM_LANEASSIT_STATUS_OFF', 1: 'IPM_LANEASSIT_STATUS_STANDBY', 2: 'IPM_LANEASSIT_STATUS_LDW_ACTIVE_LKA_STANDBY', 3: 'IPM_LANEASSIT_STATUS_LDW_STANDBY_LKA_ACTIVE', 4: 'IPM_LANEASSIT_STATUS_ACTIVE', 5: 'IPM_LANEASSIT_STATUS_FAULT', 6: 'IPM_LANEASSIT_STATUS_CAMERA_BLOCKED', 7: 'IPM_LANEASSIT_STATUS_CAMERA_UNCALIBRATED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_laneassit_status(uint8_t* data, Ipm_0x245_245::Ipm_laneassit_statusType ipm_laneassit_status);

  // config detail: {'name': 'IPM_SLASpdlimit', 'enum': {0: 'IPM_SLASPDLIMIT_NO_DISPLAY', 1: 'IPM_SLASPDLIMIT_SPL_CANCELLED', 2: 'IPM_SLASPDLIMIT_SPL_5', 3: 'IPM_SLASPDLIMIT_SPL_10', 4: 'IPM_SLASPDLIMIT_SPL_15', 5: 'IPM_SLASPDLIMIT_SPL_20', 6: 'IPM_SLASPDLIMIT_SPL_25', 7: 'IPM_SLASPDLIMIT_SPL_30', 8: 'IPM_SLASPDLIMIT_SPL_35', 9: 'IPM_SLASPDLIMIT_SPL_40', 10: 'IPM_SLASPDLIMIT_SPL_45', 11: 'IPM_SLASPDLIMIT_SPL_50', 12: 'IPM_SLASPDLIMIT_SPL_55', 13: 'IPM_SLASPDLIMIT_SPL_60', 14: 'IPM_SLASPDLIMIT_SPL_65', 15: 'IPM_SLASPDLIMIT_SPL_70', 16: 'IPM_SLASPDLIMIT_SPL_75', 17: 'IPM_SLASPDLIMIT_SPL_80', 18: 'IPM_SLASPDLIMIT_SPL_85', 19: 'IPM_SLASPDLIMIT_SPL_90', 20: 'IPM_SLASPDLIMIT_SPL_95', 21: 'IPM_SLASPDLIMIT_SPL_100', 22: 'IPM_SLASPDLIMIT_SPL_105', 23: 'IPM_SLASPDLIMIT_SPL_110', 24: 'IPM_SLASPDLIMIT_SPL_115', 25: 'IPM_SLASPDLIMIT_SPL_120', 26: 'IPM_SLASPDLIMIT_SPL_125', 27: 'IPM_SLASPDLIMIT_SPL_130'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|255]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_slaspdlimit(uint8_t* data, Ipm_0x245_245::Ipm_slaspdlimitType ipm_slaspdlimit);

  // config detail: {'name': 'IPM_HMA_Status', 'enum': {0: 'IPM_HMA_STATUS_OFF', 1: 'IPM_HMA_STATUS_PASSIVE', 2: 'IPM_HMA_STATUS_STANDBY', 3: 'IPM_HMA_STATUS_ACTIVE', 4: 'IPM_HMA_STATUS_FAULT', 5: 'IPM_HMA_STATUS_CAMERA_BLOCKED', 6: 'IPM_HMA_STATUS_RESERVED', 7: 'IPM_HMA_STATUS_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_hma_status(uint8_t* data, Ipm_0x245_245::Ipm_hma_statusType ipm_hma_status);

  // config detail: {'name': 'IPM_TJAICA_Status', 'enum': {0: 'IPM_TJAICA_STATUS_INACTIVE', 1: 'IPM_TJAICA_STATUS_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_tjaica_status(uint8_t* data, Ipm_0x245_245::Ipm_tjaica_statusType ipm_tjaica_status);

  // config detail: {'name': 'Rolling_counter_0x245', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_rolling_counter_0x245(uint8_t* data, int rolling_counter_0x245);

  // config detail: {'name': 'IPM_LDWWarningType', 'enum': {0: 'IPM_LDWWARNINGTYPE_AUDIBLE_WARNING_AND_WHEELSTEER_SHAKE', 1: 'IPM_LDWWARNINGTYPE_AUDIBLE_WARNING', 2: 'IPM_LDWWARNINGTYPE_WHEELSTEER_SHAKE', 3: 'IPM_LDWWARNINGTYPE_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_ldwwarningtype(uint8_t* data, Ipm_0x245_245::Ipm_ldwwarningtypeType ipm_ldwwarningtype);

  // config detail: {'name': 'IPM_LaneAssit_HandsonReq', 'enum': {0: 'IPM_LANEASSIT_HANDSONREQ_INACTIVE', 1: 'IPM_LANEASSIT_HANDSONREQ_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_laneassit_handsonreq(uint8_t* data, Ipm_0x245_245::Ipm_laneassit_handsonreqType ipm_laneassit_handsonreq);

  // config detail: {'name': 'IPM_HMA_HighbeamReq', 'enum': {0: 'IPM_HMA_HIGHBEAMREQ_NO_REQUEST', 1: 'IPM_HMA_HIGHBEAMREQ_HIGH_BEAM_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_hma_highbeamreq(uint8_t* data, Ipm_0x245_245::Ipm_hma_highbeamreqType ipm_hma_highbeamreq);

  // config detail: {'name': 'Checksum_0x245', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum_0x245(uint8_t* data, int checksum_0x245);

  // config detail: {'name': 'IPM_LaneAssit_TorqueReq', 'enum': {2047: 'IPM_LANEASSIT_TORQUEREQ_INVALID'}, 'precision': 0.01, 'len': 11, 'is_signed_var': False, 'offset': -10.24, 'physical_range': '[-10.24|10.22]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'Nm'}
  void set_p_ipm_laneassit_torquereq(uint8_t* data, double ipm_laneassit_torquereq);

  // config detail: {'name': 'IPM_LaneAssitType', 'enum': {0: 'IPM_LANEASSITTYPE_UNAVAILABLE', 1: 'IPM_LANEASSITTYPE_LDW', 2: 'IPM_LANEASSITTYPE_LKA', 3: 'IPM_LANEASSITTYPE_LDW_LKA', 4: 'IPM_LANEASSITTYPE_TJA_ICA', 5: 'IPM_LANEASSITTYPE_TJA_ICA_LDW', 6: 'IPM_LANEASSITTYPE_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_laneassittype(uint8_t* data, Ipm_0x245_245::Ipm_laneassittypeType ipm_laneassittype);

  // config detail: {'name': 'IPM_LaneAssit_WarningVibration', 'enum': {0: 'IPM_LANEASSIT_WARNINGVIBRATION_INACTIVE', 1: 'IPM_LANEASSIT_WARNINGVIBRATION_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_ipm_laneassit_warningvibration(uint8_t* data, Ipm_0x245_245::Ipm_laneassit_warningvibrationType ipm_laneassit_warningvibration);

 private:
  Ipm_0x245_245::Ipm_laneassit_torquereqstatusType ipm_laneassit_torquereqstatus_;
  Ipm_0x245_245::Ipm_laneassit_torquevalidityType ipm_laneassit_torquevalidity_;
  Ipm_0x245_245::Ipm_laneassit_sensitivityType ipm_laneassit_sensitivity_;
  Ipm_0x245_245::Ipm_laneassit_warningaudioType ipm_laneassit_warningaudio_;
  Ipm_0x245_245::Ipm_laneassit_leftvisualizationType ipm_laneassit_leftvisualization_;
  Ipm_0x245_245::Ipm_laneassit_rightvisualizationType ipm_laneassit_rightvisualization_;
  Ipm_0x245_245::Ipm_slaspdlimitunitsType ipm_slaspdlimitunits_;
  Ipm_0x245_245::Ipm_slastateType ipm_slastate_;
  Ipm_0x245_245::Ipm_laneassit_statusType ipm_laneassit_status_;
  Ipm_0x245_245::Ipm_slaspdlimitType ipm_slaspdlimit_;
  Ipm_0x245_245::Ipm_hma_statusType ipm_hma_status_;
  Ipm_0x245_245::Ipm_tjaica_statusType ipm_tjaica_status_;
  int rolling_counter_0x245_;
  Ipm_0x245_245::Ipm_ldwwarningtypeType ipm_ldwwarningtype_;
  Ipm_0x245_245::Ipm_laneassit_handsonreqType ipm_laneassit_handsonreq_;
  Ipm_0x245_245::Ipm_hma_highbeamreqType ipm_hma_highbeamreq_;
  int checksum_0x245_;
  double ipm_laneassit_torquereq_;
  Ipm_0x245_245::Ipm_laneassittypeType ipm_laneassittype_;
  Ipm_0x245_245::Ipm_laneassit_warningvibrationType ipm_laneassit_warningvibration_;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_IPM_0X245_245_H_
