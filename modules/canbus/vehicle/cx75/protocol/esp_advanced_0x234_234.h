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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_ADVANCED_0X234_234_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_ADVANCED_0X234_234_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Espadvanced0x234234 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Espadvanced0x234234();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ESP_VDCActive', 'enum': {0: 'ESP_VDCACTIVE_NOT_ACTIVE', 1: 'ESP_VDCACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_vdcactiveType esp_vdcactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_EBDActive', 'enum': {0: 'ESP_EBDACTIVE_NOT_ACTIVE', 1: 'ESP_EBDACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_ebdactiveType esp_ebdactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_ECDTempOff', 'enum': {0: 'ESP_ECDTEMPOFF_NOT_HIGH', 1: 'ESP_ECDTEMPOFF_TEMP_TOO_HIGH'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_ecdtempoffType esp_ecdtempoff(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_NoBrakeForce', 'enum': {0: 'ESP_NOBRAKEFORCE_EXIST_BRK_FORCE', 1: 'ESP_NOBRAKEFORCE_NO_BRK_FORCE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_nobrakeforceType esp_nobrakeforce(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_BrkFricTqTotAtWhlvaild', 'enum': {0: 'ESP_BRKFRICTQTOTATWHLVAILD_VALID', 1: 'ESP_BRKFRICTQTOTATWHLVAILD_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_brkfrictqtotatwhlvaildType esp_brkfrictqtotatwhlvaild(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_AEBdecAvailable', 'enum': {0: 'ESP_AEBDECAVAILABLE_NOT_AVAILABLE', 1: 'ESP_AEBDECAVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_aebdecavailableType esp_aebdecavailable(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_AEBdecActive', 'enum': {0: 'ESP_AEBDECACTIVE_NOT_ACTIVATED', 1: 'ESP_AEBDECACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_aebdecactiveType esp_aebdecactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_PrefillAvailable', 'enum': {0: 'ESP_PREFILLAVAILABLE_NOT_AVAILABLE', 1: 'ESP_PREFILLAVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_prefillavailableType esp_prefillavailable(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_PrefillActive', 'enum': {0: 'ESP_PREFILLACTIVE_NOT_ACTIVATED', 1: 'ESP_PREFILLACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_prefillactiveType esp_prefillactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_ABAavailable', 'enum': {0: 'ESP_ABAAVAILABLE_NOT_AVAILABLE', 1: 'ESP_ABAAVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_abaavailableType esp_abaavailable(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_ABAactive', 'enum': {0: 'ESP_ABAACTIVE_NOT_ACTIVATED', 1: 'ESP_ABAACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_abaactiveType esp_abaactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_CDDAvailable', 'enum': {0: 'ESP_CDDAVAILABLE_NOT_AVAILABLE', 1: 'ESP_CDDAVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_cddavailableType esp_cddavailable(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_DTCactive', 'enum': {0: 'ESP_DTCACTIVE_NOT_ACTIVATED', 1: 'ESP_DTCACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_dtcactiveType esp_dtcactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_AWBavailable', 'enum': {0: 'ESP_AWBAVAILABLE_NOT_AVAILABLE', 1: 'ESP_AWBAVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_awbavailableType esp_awbavailable(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_AWBactive', 'enum': {0: 'ESP_AWBACTIVE_NOT_ACTIVATED', 1: 'ESP_AWBACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_awbactiveType esp_awbactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_BrkFricTqTotAtWhl', 'offset': 0.0, 'precision': 1.0, 'len': 14, 'is_signed_var': False, 'physical_range': '[0|10230]', 'bit': 37, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int esp_brkfrictqtotatwhl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_VLCerror', 'enum': {0: 'ESP_VLCERROR_NOT_ERROR', 1: 'ESP_VLCERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 38, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_vlcerrorType esp_vlcerror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_CDDerror', 'enum': {0: 'ESP_CDDERROR_NOT_ERROR', 1: 'ESP_CDDERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_cdderrorType esp_cdderror(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x234', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x234(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x234', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x234(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_CDDActive', 'enum': {0: 'ESP_CDDACTIVE_NOT_ACTIVATED', 1: 'ESP_CDDACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_cddactiveType esp_cddactive(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_CDD_APactive', 'enum': {0: 'ESP_CDD_APACTIVE_NOT_ACTIVATED', 1: 'ESP_CDD_APACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_advanced_0x234_234::Esp_cdd_apactiveType esp_cdd_apactive(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_ESP_ADVANCED_0X234_234_H_
