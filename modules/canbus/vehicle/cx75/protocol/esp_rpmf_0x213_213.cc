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

#include "modules/canbus/vehicle/cx75/protocol/esp_rpmf_0x213_213.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Esprpmf0x213213::Esprpmf0x213213() {}
const int32_t Esprpmf0x213213::ID = 0x213;

void Esprpmf0x213213::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_esp_rpmf_0x213_213()->set_esp_wheelrpmfr(esp_wheelrpmfr(bytes, length));
  chassis->mutable_cx75()->mutable_esp_rpmf_0x213_213()->set_esp_wheelrpmfrvalid(esp_wheelrpmfrvalid(bytes, length));
  chassis->mutable_cx75()->mutable_esp_rpmf_0x213_213()->set_rolling_counter_0x213(rolling_counter_0x213(bytes, length));
  chassis->mutable_cx75()->mutable_esp_rpmf_0x213_213()->set_checksum_0x213(checksum_0x213(bytes, length));
  chassis->mutable_cx75()->mutable_esp_rpmf_0x213_213()->set_esp_wheelrpmfl(esp_wheelrpmfl(bytes, length));
  chassis->mutable_cx75()->mutable_esp_rpmf_0x213_213()->set_esp_wheelrpmflvalid(esp_wheelrpmflvalid(bytes, length));
}

// config detail: {'name': 'esp_wheelrpmfr', 'offset': 0.0, 'precision': 0.0625, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|2047.875]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rpm'}
double Esprpmf0x213213::esp_wheelrpmfr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'esp_wheelrpmfrvalid', 'enum': {0: 'ESP_WHEELRPMFRVALID_VALID', 1: 'ESP_WHEELRPMFRVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 24, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_rpmf_0x213_213::Esp_wheelrpmfrvalidType Esprpmf0x213213::esp_wheelrpmfrvalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  Esp_rpmf_0x213_213::Esp_wheelrpmfrvalidType ret =  static_cast<Esp_rpmf_0x213_213::Esp_wheelrpmfrvalidType>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x213', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esprpmf0x213213::rolling_counter_0x213(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'checksum_0x213', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esprpmf0x213213::checksum_0x213(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_wheelrpmfl', 'offset': 0.0, 'precision': 0.0625, 'len': 15, 'is_signed_var': False, 'physical_range': '[0|2047.875]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rpm'}
double Esprpmf0x213213::esp_wheelrpmfl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  double ret = x * 0.062500;
  return ret;
}

// config detail: {'name': 'esp_wheelrpmflvalid', 'enum': {0: 'ESP_WHEELRPMFLVALID_VALID', 1: 'ESP_WHEELRPMFLVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 8, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_rpmf_0x213_213::Esp_wheelrpmflvalidType Esprpmf0x213213::esp_wheelrpmflvalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  Esp_rpmf_0x213_213::Esp_wheelrpmflvalidType ret =  static_cast<Esp_rpmf_0x213_213::Esp_wheelrpmflvalidType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
