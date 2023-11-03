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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_SOD_0X275_275_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_SOD_0X275_275_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Sod0x275275 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Sod0x275275();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'SOD_LCA_warningReqRight', 'enum': {0: 'SOD_LCA_WARNINGREQRIGHT_NO_WARNING', 1: 'SOD_LCA_WARNINGREQRIGHT_WARNING_LEVEL_1', 2: 'SOD_LCA_WARNINGREQRIGHT_WARNING_LEVEL_2', 3: 'SOD_LCA_WARNINGREQRIGHT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_lca_warningreqrightType sod_lca_warningreqright(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_BLIS_Active', 'enum': {0: 'SOD_BLIS_ACTIVE_NO_ACTIVE', 1: 'SOD_BLIS_ACTIVE_STANBY', 2: 'SOD_BLIS_ACTIVE_ACTIVE', 3: 'SOD_BLIS_ACTIVE_FAIL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_blis_activeType sod_blis_active(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_BLIS_display', 'enum': {0: 'SOD_BLIS_DISPLAY_NO_DISPLAY', 1: 'SOD_BLIS_DISPLAY_SOD_LCA_CTA_DOA_ERROR', 2: 'SOD_BLIS_DISPLAY_SOD_LCA_CTA_DOA_BLINDNESS', 3: 'SOD_BLIS_DISPLAY_SOD_LCA_CTA_DOA_UNCALIBRATION', 4: 'SOD_BLIS_DISPLAY_TEMPORARY_ERROR', 5: 'SOD_BLIS_DISPLAY_RESERVED', 6: 'SOD_BLIS_DISPLAY_RESERVED', 7: 'SOD_BLIS_DISPLAY_RESERVED', 8: 'SOD_BLIS_DISPLAY_RESERVED', 9: 'SOD_BLIS_DISPLAY_RESERVED', 10: 'SOD_BLIS_DISPLAY_RESERVED', 11: 'SOD_BLIS_DISPLAY_RESERVED', 12: 'SOD_BLIS_DISPLAY_RESERVED', 13: 'SOD_BLIS_DISPLAY_RESERVED', 14: 'SOD_BLIS_DISPLAY_RESERVED', 15: 'SOD_BLIS_DISPLAY_RESERVED'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_blis_displayType sod_blis_display(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_LCA_warningReqLeft', 'enum': {0: 'SOD_LCA_WARNINGREQLEFT_NO_WARNING', 1: 'SOD_LCA_WARNINGREQLEFT_WARNING_LEVEL_1', 2: 'SOD_LCA_WARNINGREQLEFT_WARNING_LEVEL_2', 3: 'SOD_LCA_WARNINGREQLEFT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_lca_warningreqleftType sod_lca_warningreqleft(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_DOA_warningReqLeftRear', 'enum': {0: 'SOD_DOA_WARNINGREQLEFTREAR_NO_WARNING', 1: 'SOD_DOA_WARNINGREQLEFTREAR_WARNING_LEVEL_1', 2: 'SOD_DOA_WARNINGREQLEFTREAR_WARNING_LEVEL_2', 3: 'SOD_DOA_WARNINGREQLEFTREAR_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_doa_warningreqleftrearType sod_doa_warningreqleftrear(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_DOA_warningReqLeftFront', 'enum': {0: 'SOD_DOA_WARNINGREQLEFTFRONT_NO_WARNING', 1: 'SOD_DOA_WARNINGREQLEFTFRONT_WARNING_LEVEL_1', 2: 'SOD_DOA_WARNINGREQLEFTFRONT_WARNING_LEVEL_2', 3: 'SOD_DOA_WARNINGREQLEFTFRONT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_doa_warningreqleftfrontType sod_doa_warningreqleftfront(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_CTA_warningReqLeft', 'enum': {0: 'SOD_CTA_WARNINGREQLEFT_NO_WARNING', 1: 'SOD_CTA_WARNINGREQLEFT_RESERVED', 2: 'SOD_CTA_WARNINGREQLEFT_WARNING_LEVEL_2', 3: 'SOD_CTA_WARNINGREQLEFT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_cta_warningreqleftType sod_cta_warningreqleft(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_DOA_State', 'enum': {0: 'SOD_DOA_STATE_INACTIVE', 1: 'SOD_DOA_STATE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_doa_stateType sod_doa_state(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_CTA_State', 'enum': {0: 'SOD_CTA_STATE_INACTIVE', 1: 'SOD_CTA_STATE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_cta_stateType sod_cta_state(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_SODLCA_State', 'enum': {0: 'SOD_SODLCA_STATE_INACTIVE', 1: 'SOD_SODLCA_STATE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_sodlca_stateType sod_sodlca_state(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'DOA_OFFTelltale', 'enum': {0: 'DOA_OFFTELLTALE_OFFTELLTLAE_OFF', 1: 'DOA_OFFTELLTALE_OFFTELLTALE_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Doa_offtelltaleType doa_offtelltale(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_DOA_warningReqRightRear', 'enum': {0: 'SOD_DOA_WARNINGREQRIGHTREAR_NO_WARNING', 1: 'SOD_DOA_WARNINGREQRIGHTREAR_WARNING_LEVEL_1', 2: 'SOD_DOA_WARNINGREQRIGHTREAR_WARNING_LEVEL_2', 3: 'SOD_DOA_WARNINGREQRIGHTREAR_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_doa_warningreqrightrearType sod_doa_warningreqrightrear(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'CTA_OFFTelltale', 'enum': {0: 'CTA_OFFTELLTALE_OFFTELLTLAE_OFF', 1: 'CTA_OFFTELLTALE_OFFTELLTALE_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 30, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Cta_offtelltaleType cta_offtelltale(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SODLCA_OFFTelltale', 'enum': {0: 'SODLCA_OFFTELLTALE_OFFTELLTLAE_OFF', 1: 'SODLCA_OFFTELLTALE_OFFTELLTALE_ON'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sodlca_offtelltaleType sodlca_offtelltale(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_DOA_warningReqRightFront', 'enum': {0: 'SOD_DOA_WARNINGREQRIGHTFRONT_NO_WARNING', 1: 'SOD_DOA_WARNINGREQRIGHTFRONT_WARNING_LEVEL_1', 2: 'SOD_DOA_WARNINGREQRIGHTFRONT_WARNING_LEVEL_2', 3: 'SOD_DOA_WARNINGREQRIGHTFRONT_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_doa_warningreqrightfrontType sod_doa_warningreqrightfront(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x275', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x275(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x275', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x275(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_CTA_warningReqRight', 'enum': {0: 'SOD_CTA_WARNINGREQRIGHT_NO_WARNING', 1: 'SOD_CTA_WARNINGREQRIGHT_RESERVED', 2: 'SOD_CTA_WARNINGREQRIGHT_WARNING_LEVEL_2'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_cta_warningreqrightType sod_cta_warningreqright(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SOD_AutoAlignmentFlag', 'enum': {0: 'SOD_AUTOALIGNMENTFLAG_NO_SUCCESS', 1: 'SOD_AUTOALIGNMENTFLAG_SUCCESSFUL', 3: 'SOD_AUTOALIGNMENTFLAG_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Sod_0x275_275::Sod_autoalignmentflagType sod_autoalignmentflag(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_SOD_0X275_275_H_
