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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_SWM_MRR_0X31B_31B_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_SWM_MRR_0X31B_31B_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwswmmrr0x31b31b : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwswmmrr0x31b31b();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'SWM_ACCtauGapSetPlus', 'enum': {0: 'SWM_ACCTAUGAPSETPLUS_NO_PRESS', 1: 'SWM_ACCTAUGAPSETPLUS_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_acctaugapsetplusType swm_acctaugapsetplus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ACCvSetMinus', 'enum': {0: 'SWM_ACCVSETMINUS_NO_PRESS', 1: 'SWM_ACCVSETMINUS_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_accvsetminusType swm_accvsetminus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ShiftPadReqUp', 'enum': {0: 'SWM_SHIFTPADREQUP_NO_PRESS', 1: 'SWM_SHIFTPADREQUP_PRESS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 10, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_shiftpadrequpType swm_shiftpadrequp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ACCLimpHomeSts', 'enum': {0: 'SWM_ACCLIMPHOMESTS_NORMAL', 1: 'SWM_ACCLIMPHOMESTS_LIMPHOME'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_acclimphomestsType swm_acclimphomests(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_LaneAssistSwitch', 'enum': {0: 'SWM_LANEASSISTSWITCH_PREVENT_LANEASSIST_CONTROL', 1: 'SWM_LANEASSISTSWITCH_ENABLE_LANEASSIST_CONTROL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_laneassistswitchType swm_laneassistswitch(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ACCtauGapSetMinus', 'enum': {0: 'SWM_ACCTAUGAPSETMINUS_NO_PRESS', 1: 'SWM_ACCTAUGAPSETMINUS_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_acctaugapsetminusType swm_acctaugapsetminus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ACCvSetPlus', 'enum': {0: 'SWM_ACCVSETPLUS_NO_PRESS', 1: 'SWM_ACCVSETPLUS_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_accvsetplusType swm_accvsetplus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ShiftPadFlt', 'enum': {0: 'SWM_SHIFTPADFLT_NO_FAULT', 1: 'SWM_SHIFTPADFLT_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_shiftpadfltType swm_shiftpadflt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ACCDeactivate', 'enum': {0: 'SWM_ACCDEACTIVATE_NO_PRESS', 1: 'SWM_ACCDEACTIVATE_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_accdeactivateType swm_accdeactivate(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ACCResume', 'enum': {0: 'SWM_ACCRESUME_NO_PRESS', 1: 'SWM_ACCRESUME_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 4, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_accresumeType swm_accresume(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ACCSet', 'enum': {0: 'SWM_ACCSET_NO_PRESS', 1: 'SWM_ACCSET_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_accsetType swm_accset(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x31B', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x31b(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ACCEnableSwitch', 'enum': {0: 'SWM_ACCENABLESWITCH_PREVENT_ACC_CONTROL', 1: 'SWM_ACCENABLESWITCH_ENABLE_ACC_CONTROL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_accenableswitchType swm_accenableswitch(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x31B', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x31b(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ACCResume_Qt', 'enum': {0: 'SWM_ACCRESUME_QT_VALID', 1: 'SWM_ACCRESUME_QT_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_accresume_qtType swm_accresume_qt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_TJASwitch', 'enum': {0: 'SWM_TJASWITCH_PREVENT_TJA_CONTROL', 1: 'SWM_TJASWITCH_ENABLE_TJA_CONTROL'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_tjaswitchType swm_tjaswitch(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_ShiftPadReqDown', 'enum': {0: 'SWM_SHIFTPADREQDOWN_NO_PRESS', 1: 'SWM_SHIFTPADREQDOWN_PRESS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_mrr_0x31b_31b::Swm_shiftpadreqdownType swm_shiftpadreqdown(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_SWM_MRR_0X31B_31B_H_
