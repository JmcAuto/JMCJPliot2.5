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

#include "modules/canbus/vehicle/cx75/protocol/esp_advanced_0x234_234.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Espadvanced0x234234::Espadvanced0x234234() {}
const int32_t Espadvanced0x234234::ID = 0x234;

void Espadvanced0x234234::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_vdcactive(esp_vdcactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_ebdactive(esp_ebdactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_ecdtempoff(esp_ecdtempoff(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_nobrakeforce(esp_nobrakeforce(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_brkfrictqtotatwhlvaild(esp_brkfrictqtotatwhlvaild(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_aebdecavailable(esp_aebdecavailable(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_aebdecactive(esp_aebdecactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_prefillavailable(esp_prefillavailable(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_prefillactive(esp_prefillactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_abaavailable(esp_abaavailable(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_abaactive(esp_abaactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_cddavailable(esp_cddavailable(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_dtcactive(esp_dtcactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_awbavailable(esp_awbavailable(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_awbactive(esp_awbactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_brkfrictqtotatwhl(esp_brkfrictqtotatwhl(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_vlcerror(esp_vlcerror(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_cdderror(esp_cdderror(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_rolling_counter_0x234(rolling_counter_0x234(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_checksum_0x234(checksum_0x234(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_cddactive(esp_cddactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_advanced_0x234_234()->set_esp_cdd_apactive(esp_cdd_apactive(bytes, length));
}

// config detail: {'name': 'esp_vdcactive', 'enum': {0: 'ESP_VDCACTIVE_NOT_ACTIVE', 1: 'ESP_VDCACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_vdcactiveType Espadvanced0x234234::esp_vdcactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  Esp_advanced_0x234_234::Esp_vdcactiveType ret =  static_cast<Esp_advanced_0x234_234::Esp_vdcactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_ebdactive', 'enum': {0: 'ESP_EBDACTIVE_NOT_ACTIVE', 1: 'ESP_EBDACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_ebdactiveType Espadvanced0x234234::esp_ebdactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  Esp_advanced_0x234_234::Esp_ebdactiveType ret =  static_cast<Esp_advanced_0x234_234::Esp_ebdactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_ecdtempoff', 'enum': {0: 'ESP_ECDTEMPOFF_NOT_HIGH', 1: 'ESP_ECDTEMPOFF_TEMP_TOO_HIGH'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 14, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_ecdtempoffType Espadvanced0x234234::esp_ecdtempoff(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  Esp_advanced_0x234_234::Esp_ecdtempoffType ret =  static_cast<Esp_advanced_0x234_234::Esp_ecdtempoffType>(x);
  return ret;
}

// config detail: {'name': 'esp_nobrakeforce', 'enum': {0: 'ESP_NOBRAKEFORCE_EXIST_BRK_FORCE', 1: 'ESP_NOBRAKEFORCE_NO_BRK_FORCE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_nobrakeforceType Espadvanced0x234234::esp_nobrakeforce(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  Esp_advanced_0x234_234::Esp_nobrakeforceType ret =  static_cast<Esp_advanced_0x234_234::Esp_nobrakeforceType>(x);
  return ret;
}

// config detail: {'name': 'esp_brkfrictqtotatwhlvaild', 'enum': {0: 'ESP_BRKFRICTQTOTATWHLVAILD_VALID', 1: 'ESP_BRKFRICTQTOTATWHLVAILD_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 16, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_brkfrictqtotatwhlvaildType Espadvanced0x234234::esp_brkfrictqtotatwhlvaild(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 1);

  Esp_advanced_0x234_234::Esp_brkfrictqtotatwhlvaildType ret =  static_cast<Esp_advanced_0x234_234::Esp_brkfrictqtotatwhlvaildType>(x);
  return ret;
}

// config detail: {'name': 'esp_aebdecavailable', 'enum': {0: 'ESP_AEBDECAVAILABLE_NOT_AVAILABLE', 1: 'ESP_AEBDECAVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_aebdecavailableType Espadvanced0x234234::esp_aebdecavailable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(1, 1);

  Esp_advanced_0x234_234::Esp_aebdecavailableType ret =  static_cast<Esp_advanced_0x234_234::Esp_aebdecavailableType>(x);
  return ret;
}

// config detail: {'name': 'esp_aebdecactive', 'enum': {0: 'ESP_AEBDECACTIVE_NOT_ACTIVATED', 1: 'ESP_AEBDECACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 18, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_aebdecactiveType Espadvanced0x234234::esp_aebdecactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 1);

  Esp_advanced_0x234_234::Esp_aebdecactiveType ret =  static_cast<Esp_advanced_0x234_234::Esp_aebdecactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_prefillavailable', 'enum': {0: 'ESP_PREFILLAVAILABLE_NOT_AVAILABLE', 1: 'ESP_PREFILLAVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_prefillavailableType Espadvanced0x234234::esp_prefillavailable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 1);

  Esp_advanced_0x234_234::Esp_prefillavailableType ret =  static_cast<Esp_advanced_0x234_234::Esp_prefillavailableType>(x);
  return ret;
}

// config detail: {'name': 'esp_prefillactive', 'enum': {0: 'ESP_PREFILLACTIVE_NOT_ACTIVATED', 1: 'ESP_PREFILLACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 20, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_prefillactiveType Espadvanced0x234234::esp_prefillactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 1);

  Esp_advanced_0x234_234::Esp_prefillactiveType ret =  static_cast<Esp_advanced_0x234_234::Esp_prefillactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_abaavailable', 'enum': {0: 'ESP_ABAAVAILABLE_NOT_AVAILABLE', 1: 'ESP_ABAAVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_abaavailableType Espadvanced0x234234::esp_abaavailable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(5, 1);

  Esp_advanced_0x234_234::Esp_abaavailableType ret =  static_cast<Esp_advanced_0x234_234::Esp_abaavailableType>(x);
  return ret;
}

// config detail: {'name': 'esp_abaactive', 'enum': {0: 'ESP_ABAACTIVE_NOT_ACTIVATED', 1: 'ESP_ABAACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 22, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_abaactiveType Espadvanced0x234234::esp_abaactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 1);

  Esp_advanced_0x234_234::Esp_abaactiveType ret =  static_cast<Esp_advanced_0x234_234::Esp_abaactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_cddavailable', 'enum': {0: 'ESP_CDDAVAILABLE_NOT_AVAILABLE', 1: 'ESP_CDDAVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_cddavailableType Espadvanced0x234234::esp_cddavailable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(7, 1);

  Esp_advanced_0x234_234::Esp_cddavailableType ret =  static_cast<Esp_advanced_0x234_234::Esp_cddavailableType>(x);
  return ret;
}

// config detail: {'name': 'esp_dtcactive', 'enum': {0: 'ESP_DTCACTIVE_NOT_ACTIVATED', 1: 'ESP_DTCACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_dtcactiveType Espadvanced0x234234::esp_dtcactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  Esp_advanced_0x234_234::Esp_dtcactiveType ret =  static_cast<Esp_advanced_0x234_234::Esp_dtcactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_awbavailable', 'enum': {0: 'ESP_AWBAVAILABLE_NOT_AVAILABLE', 1: 'ESP_AWBAVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_awbavailableType Espadvanced0x234234::esp_awbavailable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(1, 1);

  Esp_advanced_0x234_234::Esp_awbavailableType ret =  static_cast<Esp_advanced_0x234_234::Esp_awbavailableType>(x);
  return ret;
}

// config detail: {'name': 'esp_awbactive', 'enum': {0: 'ESP_AWBACTIVE_NOT_ACTIVATED', 1: 'ESP_AWBACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_awbactiveType Espadvanced0x234234::esp_awbactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  Esp_advanced_0x234_234::Esp_awbactiveType ret =  static_cast<Esp_advanced_0x234_234::Esp_awbactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_brkfrictqtotatwhl', 'offset': 0.0, 'precision': 1.0, 'len': 14, 'is_signed_var': False, 'physical_range': '[0|10230]', 'bit': 37, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espadvanced0x234234::esp_brkfrictqtotatwhl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_vlcerror', 'enum': {0: 'ESP_VLCERROR_NOT_ERROR', 1: 'ESP_VLCERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 38, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_vlcerrorType Espadvanced0x234234::esp_vlcerror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 1);

  Esp_advanced_0x234_234::Esp_vlcerrorType ret =  static_cast<Esp_advanced_0x234_234::Esp_vlcerrorType>(x);
  return ret;
}

// config detail: {'name': 'esp_cdderror', 'enum': {0: 'ESP_CDDERROR_NOT_ERROR', 1: 'ESP_CDDERROR_ERROR'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_cdderrorType Espadvanced0x234234::esp_cdderror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  Esp_advanced_0x234_234::Esp_cdderrorType ret =  static_cast<Esp_advanced_0x234_234::Esp_cdderrorType>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x234', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espadvanced0x234234::rolling_counter_0x234(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'checksum_0x234', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espadvanced0x234234::checksum_0x234(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_cddactive', 'enum': {0: 'ESP_CDDACTIVE_NOT_ACTIVATED', 1: 'ESP_CDDACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_cddactiveType Espadvanced0x234234::esp_cddactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Esp_advanced_0x234_234::Esp_cddactiveType ret =  static_cast<Esp_advanced_0x234_234::Esp_cddactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_cdd_apactive', 'enum': {0: 'ESP_CDD_APACTIVE_NOT_ACTIVATED', 1: 'ESP_CDD_APACTIVE_ACTIVATED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_advanced_0x234_234::Esp_cdd_apactiveType Espadvanced0x234234::esp_cdd_apactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  Esp_advanced_0x234_234::Esp_cdd_apactiveType ret =  static_cast<Esp_advanced_0x234_234::Esp_cdd_apactiveType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
