/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/cx75/protocol/esp_whlpulse_0x236_236.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Espwhlpulse0x236236::Espwhlpulse0x236236() {}
const int32_t Espwhlpulse0x236236::ID = 0x236;

void Espwhlpulse0x236236::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_esp_whlpulse_0x236_236()->set_esp_wheelpulse_fr(esp_wheelpulse_fr(bytes, length));
  chassis->mutable_cx75()->mutable_esp_whlpulse_0x236_236()->set_esp_wheelpulse_rl(esp_wheelpulse_rl(bytes, length));
  chassis->mutable_cx75()->mutable_esp_whlpulse_0x236_236()->set_esp_wheelpulse_rr(esp_wheelpulse_rr(bytes, length));
  chassis->mutable_cx75()->mutable_esp_whlpulse_0x236_236()->set_rollingcounter_esp_0x236(rollingcounter_esp_0x236(bytes, length));
  chassis->mutable_cx75()->mutable_esp_whlpulse_0x236_236()->set_esp_wheelpulse_rr_valid(esp_wheelpulse_rr_valid(bytes, length));
  chassis->mutable_cx75()->mutable_esp_whlpulse_0x236_236()->set_esp_wheelpulse_rl_valid(esp_wheelpulse_rl_valid(bytes, length));
  chassis->mutable_cx75()->mutable_esp_whlpulse_0x236_236()->set_esp_wheelpulse_fr_valid(esp_wheelpulse_fr_valid(bytes, length));
  chassis->mutable_cx75()->mutable_esp_whlpulse_0x236_236()->set_esp_wheelpulse_fl_valid(esp_wheelpulse_fl_valid(bytes, length));
  chassis->mutable_cx75()->mutable_esp_whlpulse_0x236_236()->set_checksum_esp_0x236(checksum_esp_0x236(bytes, length));
  chassis->mutable_cx75()->mutable_esp_whlpulse_0x236_236()->set_esp_wheelpulse_fl(esp_wheelpulse_fl(bytes, length));
}

// config detail: {'name': 'esp_wheelpulse_fr', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espwhlpulse0x236236::esp_wheelpulse_fr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_wheelpulse_rl', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espwhlpulse0x236236::esp_wheelpulse_rl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_wheelpulse_rr', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espwhlpulse0x236236::esp_wheelpulse_rr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rollingcounter_esp_0x236', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espwhlpulse0x236236::rollingcounter_esp_0x236(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_wheelpulse_rr_valid', 'enum': {0: 'ESP_WHEELPULSE_RR_VALID_VALID', 1: 'ESP_WHEELPULSE_RR_VALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_whlpulse_0x236_236::Esp_wheelpulse_rr_validType Espwhlpulse0x236236::esp_wheelpulse_rr_valid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Esp_whlpulse_0x236_236::Esp_wheelpulse_rr_validType ret =  static_cast<Esp_whlpulse_0x236_236::Esp_wheelpulse_rr_validType>(x);
  return ret;
}

// config detail: {'name': 'esp_wheelpulse_rl_valid', 'enum': {0: 'ESP_WHEELPULSE_RL_VALID_VALID', 1: 'ESP_WHEELPULSE_RL_VALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_whlpulse_0x236_236::Esp_wheelpulse_rl_validType Espwhlpulse0x236236::esp_wheelpulse_rl_valid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Esp_whlpulse_0x236_236::Esp_wheelpulse_rl_validType ret =  static_cast<Esp_whlpulse_0x236_236::Esp_wheelpulse_rl_validType>(x);
  return ret;
}

// config detail: {'name': 'esp_wheelpulse_fr_valid', 'enum': {0: 'ESP_WHEELPULSE_FR_VALID_VALID', 1: 'ESP_WHEELPULSE_FR_VALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_whlpulse_0x236_236::Esp_wheelpulse_fr_validType Espwhlpulse0x236236::esp_wheelpulse_fr_valid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Esp_whlpulse_0x236_236::Esp_wheelpulse_fr_validType ret =  static_cast<Esp_whlpulse_0x236_236::Esp_wheelpulse_fr_validType>(x);
  return ret;
}

// config detail: {'name': 'esp_wheelpulse_fl_valid', 'enum': {0: 'ESP_WHEELPULSE_FL_VALID_VALID', 1: 'ESP_WHEELPULSE_FL_VALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_whlpulse_0x236_236::Esp_wheelpulse_fl_validType Espwhlpulse0x236236::esp_wheelpulse_fl_valid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  Esp_whlpulse_0x236_236::Esp_wheelpulse_fl_validType ret =  static_cast<Esp_whlpulse_0x236_236::Esp_wheelpulse_fl_validType>(x);
  return ret;
}

// config detail: {'name': 'checksum_esp_0x236', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espwhlpulse0x236236::checksum_esp_0x236(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_wheelpulse_fl', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|254]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espwhlpulse0x236236::esp_wheelpulse_fl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
