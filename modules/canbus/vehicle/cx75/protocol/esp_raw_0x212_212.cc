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

#include "modules/canbus/vehicle/cx75/protocol/esp_raw_0x212_212.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Espraw0x212212::Espraw0x212212() {}
const int32_t Espraw0x212212::ID = 0x212;

void Espraw0x212212::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_esp_raw_0x212_212()->set_esp_vxwwheelrawfr(esp_vxwwheelrawfr(bytes, length));
  chassis->mutable_cx75()->mutable_esp_raw_0x212_212()->set_esp_vxwwheelrawrl(esp_vxwwheelrawrl(bytes, length));
  chassis->mutable_cx75()->mutable_esp_raw_0x212_212()->set_esp_vxwwheelrawrr(esp_vxwwheelrawrr(bytes, length));
  chassis->mutable_cx75()->mutable_esp_raw_0x212_212()->set_rolling_counter_0x212(rolling_counter_0x212(bytes, length));
  chassis->mutable_cx75()->mutable_esp_raw_0x212_212()->set_esp_vxwwheelrrvalid(esp_vxwwheelrrvalid(bytes, length));
  chassis->mutable_cx75()->mutable_esp_raw_0x212_212()->set_esp_vxwwheelrlvalid(esp_vxwwheelrlvalid(bytes, length));
  chassis->mutable_cx75()->mutable_esp_raw_0x212_212()->set_esp_vxwwheelfrvalid(esp_vxwwheelfrvalid(bytes, length));
  chassis->mutable_cx75()->mutable_esp_raw_0x212_212()->set_esp_vxwwheelflvalid(esp_vxwwheelflvalid(bytes, length));
  chassis->mutable_cx75()->mutable_esp_raw_0x212_212()->set_checksum_0x212(checksum_0x212(bytes, length));
  chassis->mutable_cx75()->mutable_esp_raw_0x212_212()->set_esp_vxwwheelrawfl(esp_vxwwheelrawfl(bytes, length));
}

// config detail: {'name': 'esp_vxwwheelrawfr', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 11, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
double Espraw0x212212::esp_vxwwheelrawfr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'name': 'esp_vxwwheelrawrl', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
double Espraw0x212212::esp_vxwwheelrawrl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'name': 'esp_vxwwheelrawrr', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 35, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
double Espraw0x212212::esp_vxwwheelrawrr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x212', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espraw0x212212::rolling_counter_0x212(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_vxwwheelrrvalid', 'enum': {0: 'ESP_VXWWHEELRRVALID_VALID', 1: 'ESP_VXWWHEELRRVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_raw_0x212_212::Esp_vxwwheelrrvalidType Espraw0x212212::esp_vxwwheelrrvalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Esp_raw_0x212_212::Esp_vxwwheelrrvalidType ret =  static_cast<Esp_raw_0x212_212::Esp_vxwwheelrrvalidType>(x);
  return ret;
}

// config detail: {'name': 'esp_vxwwheelrlvalid', 'enum': {0: 'ESP_VXWWHEELRLVALID_VALID', 1: 'ESP_VXWWHEELRLVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_raw_0x212_212::Esp_vxwwheelrlvalidType Espraw0x212212::esp_vxwwheelrlvalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Esp_raw_0x212_212::Esp_vxwwheelrlvalidType ret =  static_cast<Esp_raw_0x212_212::Esp_vxwwheelrlvalidType>(x);
  return ret;
}

// config detail: {'name': 'esp_vxwwheelfrvalid', 'enum': {0: 'ESP_VXWWHEELFRVALID_VALID', 1: 'ESP_VXWWHEELFRVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_raw_0x212_212::Esp_vxwwheelfrvalidType Espraw0x212212::esp_vxwwheelfrvalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Esp_raw_0x212_212::Esp_vxwwheelfrvalidType ret =  static_cast<Esp_raw_0x212_212::Esp_vxwwheelfrvalidType>(x);
  return ret;
}

// config detail: {'name': 'esp_vxwwheelflvalid', 'enum': {0: 'ESP_VXWWHEELFLVALID_VALID', 1: 'ESP_VXWWHEELFLVALID_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_raw_0x212_212::Esp_vxwwheelflvalidType Espraw0x212212::esp_vxwwheelflvalid(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  Esp_raw_0x212_212::Esp_vxwwheelflvalidType ret =  static_cast<Esp_raw_0x212_212::Esp_vxwwheelflvalidType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x212', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espraw0x212212::checksum_0x212(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_vxwwheelrawfl', 'offset': 0.0, 'precision': 0.05625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|230.2875]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kph'}
double Espraw0x212212::esp_vxwwheelrawfl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.056250;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
