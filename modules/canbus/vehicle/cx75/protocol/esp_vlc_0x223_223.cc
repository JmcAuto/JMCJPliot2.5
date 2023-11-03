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

#include "modules/canbus/vehicle/cx75/protocol/esp_vlc_0x223_223.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Espvlc0x223223::Espvlc0x223223() {}
const int32_t Espvlc0x223223::ID = 0x223;

void Espvlc0x223223::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_vlc_internaltargetacce(esp_vlc_internaltargetacce(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_apa_gearboxenable(esp_apa_gearboxenable(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_target_gear_request(esp_target_gear_request(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_vlc_apactive(esp_vlc_apactive(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_pam_lc_failurests(esp_pam_lc_failurests(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_pam_lc_status(esp_pam_lc_status(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_rolling_counter_0x223(rolling_counter_0x223(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_vlc_active(esp_vlc_active(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_vlc_available(esp_vlc_available(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_vlcapa_available(esp_vlcapa_available(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_vlcengtorqreqact(esp_vlcengtorqreqact(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_checksum_0x223(checksum_0x223(bytes, length));
  chassis->mutable_cx75()->mutable_esp_vlc_0x223_223()->set_esp_vlcengtorqreq(esp_vlcengtorqreq(bytes, length));
  chassis->mutable_check_response()->set_is_esp_online(
      esp_vlc_active(bytes, length) == 1);
    chassis->mutable_check_response()->set_is_apa_online(
      esp_vlc_apactive(bytes, length) == 1&&esp_pam_lc_failurests(bytes, length)==0);
}

// config detail: {'name': 'esp_vlc_internaltargetacce', 'offset': -7.0, 'precision': 0.05, 'len': 8, 'is_signed_var': False, 'physical_range': '[-7|5.75]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Espvlc0x223223::esp_vlc_internaltargetacce(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.050000 + -7.000000;
  return ret;
}

// config detail: {'name': 'esp_apa_gearboxenable', 'enum': {0: 'ESP_APA_GEARBOXENABLE_NO_REQUEST', 1: 'ESP_APA_GEARBOXENABLE_GEAR_SHIFT_REQUEST'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_vlc_0x223_223::Esp_apa_gearboxenableType Espvlc0x223223::esp_apa_gearboxenable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 2);

  Esp_vlc_0x223_223::Esp_apa_gearboxenableType ret =  static_cast<Esp_vlc_0x223_223::Esp_apa_gearboxenableType>(x);
  return ret;
}

// config detail: {'name': 'esp_target_gear_request', 'enum': {0: 'ESP_TARGET_GEAR_REQUEST_NO_REQUEST', 1: 'ESP_TARGET_GEAR_REQUEST_PARK', 2: 'ESP_TARGET_GEAR_REQUEST_REVERSE', 3: 'ESP_TARGET_GEAR_REQUEST_NEUTRAL', 4: 'ESP_TARGET_GEAR_REQUEST_DRIVE'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_vlc_0x223_223::Esp_target_gear_requestType Espvlc0x223223::esp_target_gear_request(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 3);

  Esp_vlc_0x223_223::Esp_target_gear_requestType ret =  static_cast<Esp_vlc_0x223_223::Esp_target_gear_requestType>(x);
  return ret;
}

// config detail: {'name': 'esp_vlc_apactive', 'enum': {0: 'ESP_VLC_APACTIVE_NOT_ACTIVE', 1: 'ESP_VLC_APACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_vlc_0x223_223::Esp_vlc_apactiveType Espvlc0x223223::esp_vlc_apactive(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Esp_vlc_0x223_223::Esp_vlc_apactiveType ret =  static_cast<Esp_vlc_0x223_223::Esp_vlc_apactiveType>(x);
  return ret;
}

// config detail: {'name': 'esp_pam_lc_failurests', 'enum': {0: 'ESP_PAM_LC_FAILURESTS_NO_ERROR', 1: 'ESP_PAM_LC_FAILURESTS_VEHICLE_BLOCKED', 2: 'ESP_PAM_LC_FAILURESTS_UNEXPECTED_GEARPOSITION', 3: 'ESP_PAM_LC_FAILURESTS_UNEXPECTED_EPB_ACTION', 4: 'ESP_PAM_LC_FAILURESTS_UNEXPECTED_ACCPEDALINTERVENTION', 5: 'ESP_PAM_LC_FAILURESTS_UNEXPECTED_GEARINTERVENTION', 7: 'ESP_PAM_LC_FAILURESTS_ERROR'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_vlc_0x223_223::Esp_pam_lc_failurestsType Espvlc0x223223::esp_pam_lc_failurests(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 3);

  Esp_vlc_0x223_223::Esp_pam_lc_failurestsType ret =  static_cast<Esp_vlc_0x223_223::Esp_pam_lc_failurestsType>(x);
  return ret;
}

// config detail: {'name': 'esp_pam_lc_status', 'enum': {0: 'ESP_PAM_LC_STATUS_OFF', 1: 'ESP_PAM_LC_STATUS_STANDBY', 10: 'ESP_PAM_LC_STATUS_ERROR', 4: 'ESP_PAM_LC_STATUS_ACTIVE_AUTOMATICPARK'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_vlc_0x223_223::Esp_pam_lc_statusType Espvlc0x223223::esp_pam_lc_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 4);

  Esp_vlc_0x223_223::Esp_pam_lc_statusType ret =  static_cast<Esp_vlc_0x223_223::Esp_pam_lc_statusType>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x223', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espvlc0x223223::rolling_counter_0x223(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_vlc_active', 'enum': {0: 'ESP_VLC_ACTIVE_NOT_ACTIVE', 1: 'ESP_VLC_ACTIVE_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_vlc_0x223_223::Esp_vlc_activeType Espvlc0x223223::esp_vlc_active(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Esp_vlc_0x223_223::Esp_vlc_activeType ret =  static_cast<Esp_vlc_0x223_223::Esp_vlc_activeType>(x);
  return ret;
}

// config detail: {'name': 'esp_vlc_available', 'enum': {0: 'ESP_VLC_AVAILABLE_NOT_AVAILABLE', 1: 'ESP_VLC_AVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_vlc_0x223_223::Esp_vlc_availableType Espvlc0x223223::esp_vlc_available(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Esp_vlc_0x223_223::Esp_vlc_availableType ret =  static_cast<Esp_vlc_0x223_223::Esp_vlc_availableType>(x);
  return ret;
}

// config detail: {'name': 'esp_vlcapa_available', 'enum': {0: 'ESP_VLCAPA_AVAILABLE_NOT_AVAILABLE', 1: 'ESP_VLCAPA_AVAILABLE_AVAILABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_vlc_0x223_223::Esp_vlcapa_availableType Espvlc0x223223::esp_vlcapa_available(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Esp_vlc_0x223_223::Esp_vlcapa_availableType ret =  static_cast<Esp_vlc_0x223_223::Esp_vlcapa_availableType>(x);
  return ret;
}

// config detail: {'name': 'esp_vlcengtorqreqact', 'enum': {0: 'ESP_VLCENGTORQREQACT_VALID', 1: 'ESP_VLCENGTORQREQACT_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_vlc_0x223_223::Esp_vlcengtorqreqactType Espvlc0x223223::esp_vlcengtorqreqact(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  Esp_vlc_0x223_223::Esp_vlcengtorqreqactType ret =  static_cast<Esp_vlc_0x223_223::Esp_vlcengtorqreqactType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x223', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espvlc0x223223::checksum_0x223(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_vlcengtorqreq', 'offset': -30000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-30000|30000]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
int Espvlc0x223223::esp_vlcengtorqreq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -30000.000000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
