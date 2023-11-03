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

#include "modules/canbus/vehicle/cx75/protocol/esp_direction_0x235_235.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Espdirection0x235235::Espdirection0x235235() {}
const int32_t Espdirection0x235235::ID = 0x235;

void Espdirection0x235235::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_whlmilgrearre(esp_whlmilgrearre(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_frwheeldrivedirstatus(esp_frwheeldrivedirstatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_flwheeldrivedirstatus(esp_flwheeldrivedirstatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_rrwheeldrivedirstatus(esp_rrwheeldrivedirstatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_rlwheeldrivedirstatus(esp_rlwheeldrivedirstatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_rrwheeldrivedirection(esp_rrwheeldrivedirection(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_rlwheeldrivedirection(esp_rlwheeldrivedirection(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_frwheeldrivedirection(esp_frwheeldrivedirection(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_flwheeldrivedirection(esp_flwheeldrivedirection(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_rolling_counter_0x235(rolling_counter_0x235(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_whlmilgrearlestatus(esp_whlmilgrearlestatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_whlmilgrearristatus(esp_whlmilgrearristatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_vehiclestandstill(esp_vehiclestandstill(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_checksum_0x235(checksum_0x235(bytes, length));
  chassis->mutable_cx75()->mutable_esp_direction_0x235_235()->set_esp_whlmilgrearle(esp_whlmilgrearle(bytes, length));
}

// config detail: {'name': 'esp_whlmilgrearre', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': 'edge'}
int Espdirection0x235235::esp_whlmilgrearre(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_frwheeldrivedirstatus', 'enum': {0: 'ESP_FRWHEELDRIVEDIRSTATUS_VALID', 1: 'ESP_FRWHEELDRIVEDIRSTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 32, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_frwheeldrivedirstatusType Espdirection0x235235::esp_frwheeldrivedirstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Esp_direction_0x235_235::Esp_frwheeldrivedirstatusType ret =  static_cast<Esp_direction_0x235_235::Esp_frwheeldrivedirstatusType>(x);
  return ret;
}

// config detail: {'name': 'esp_flwheeldrivedirstatus', 'enum': {0: 'ESP_FLWHEELDRIVEDIRSTATUS_VALID', 1: 'ESP_FLWHEELDRIVEDIRSTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_flwheeldrivedirstatusType Espdirection0x235235::esp_flwheeldrivedirstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  Esp_direction_0x235_235::Esp_flwheeldrivedirstatusType ret =  static_cast<Esp_direction_0x235_235::Esp_flwheeldrivedirstatusType>(x);
  return ret;
}

// config detail: {'name': 'esp_rrwheeldrivedirstatus', 'enum': {0: 'ESP_RRWHEELDRIVEDIRSTATUS_VALID', 1: 'ESP_RRWHEELDRIVEDIRSTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 34, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_rrwheeldrivedirstatusType Espdirection0x235235::esp_rrwheeldrivedirstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  Esp_direction_0x235_235::Esp_rrwheeldrivedirstatusType ret =  static_cast<Esp_direction_0x235_235::Esp_rrwheeldrivedirstatusType>(x);
  return ret;
}

// config detail: {'name': 'esp_rlwheeldrivedirstatus', 'enum': {0: 'ESP_RLWHEELDRIVEDIRSTATUS_VALID', 1: 'ESP_RLWHEELDRIVEDIRSTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_rlwheeldrivedirstatusType Espdirection0x235235::esp_rlwheeldrivedirstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 1);

  Esp_direction_0x235_235::Esp_rlwheeldrivedirstatusType ret =  static_cast<Esp_direction_0x235_235::Esp_rlwheeldrivedirstatusType>(x);
  return ret;
}

// config detail: {'name': 'esp_rrwheeldrivedirection', 'enum': {0: 'ESP_RRWHEELDRIVEDIRECTION_UNDEFINED', 1: 'ESP_RRWHEELDRIVEDIRECTION_STANDSTILL', 2: 'ESP_RRWHEELDRIVEDIRECTION_FORWARD', 3: 'ESP_RRWHEELDRIVEDIRECTION_BACKWARD'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_rrwheeldrivedirectionType Espdirection0x235235::esp_rrwheeldrivedirection(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 2);

  Esp_direction_0x235_235::Esp_rrwheeldrivedirectionType ret =  static_cast<Esp_direction_0x235_235::Esp_rrwheeldrivedirectionType>(x);
  return ret;
}

// config detail: {'name': 'esp_rlwheeldrivedirection', 'enum': {0: 'ESP_RLWHEELDRIVEDIRECTION_UNDEFINED', 1: 'ESP_RLWHEELDRIVEDIRECTION_STANDSTILL', 2: 'ESP_RLWHEELDRIVEDIRECTION_FORWARD', 3: 'ESP_RLWHEELDRIVEDIRECTION_BACKWARD'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 43, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_rlwheeldrivedirectionType Espdirection0x235235::esp_rlwheeldrivedirection(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 2);

  Esp_direction_0x235_235::Esp_rlwheeldrivedirectionType ret =  static_cast<Esp_direction_0x235_235::Esp_rlwheeldrivedirectionType>(x);
  return ret;
}

// config detail: {'name': 'esp_frwheeldrivedirection', 'enum': {0: 'ESP_FRWHEELDRIVEDIRECTION_UNDEFINED', 1: 'ESP_FRWHEELDRIVEDIRECTION_STANDSTILL', 2: 'ESP_FRWHEELDRIVEDIRECTION_FORWARD', 3: 'ESP_FRWHEELDRIVEDIRECTION_BACKWARD'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_frwheeldrivedirectionType Espdirection0x235235::esp_frwheeldrivedirection(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 2);

  Esp_direction_0x235_235::Esp_frwheeldrivedirectionType ret =  static_cast<Esp_direction_0x235_235::Esp_frwheeldrivedirectionType>(x);
  return ret;
}

// config detail: {'name': 'esp_flwheeldrivedirection', 'enum': {0: 'ESP_FLWHEELDRIVEDIRECTION_UNDEFINED', 1: 'ESP_FLWHEELDRIVEDIRECTION_STANDSTILL', 2: 'ESP_FLWHEELDRIVEDIRECTION_FORWARD', 3: 'ESP_FLWHEELDRIVEDIRECTION_BACKWARD'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_flwheeldrivedirectionType Espdirection0x235235::esp_flwheeldrivedirection(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 2);

  Esp_direction_0x235_235::Esp_flwheeldrivedirectionType ret =  static_cast<Esp_direction_0x235_235::Esp_flwheeldrivedirectionType>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x235', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espdirection0x235235::rolling_counter_0x235(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_whlmilgrearlestatus', 'enum': {0: 'ESP_WHLMILGREARLESTATUS_VALID', 1: 'ESP_WHLMILGREARLESTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_whlmilgrearlestatusType Espdirection0x235235::esp_whlmilgrearlestatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Esp_direction_0x235_235::Esp_whlmilgrearlestatusType ret =  static_cast<Esp_direction_0x235_235::Esp_whlmilgrearlestatusType>(x);
  return ret;
}

// config detail: {'name': 'esp_whlmilgrearristatus', 'enum': {0: 'ESP_WHLMILGREARRISTATUS_VALID', 1: 'ESP_WHLMILGREARRISTATUS_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_whlmilgrearristatusType Espdirection0x235235::esp_whlmilgrearristatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Esp_direction_0x235_235::Esp_whlmilgrearristatusType ret =  static_cast<Esp_direction_0x235_235::Esp_whlmilgrearristatusType>(x);
  return ret;
}

// config detail: {'name': 'esp_vehiclestandstill', 'enum': {0: 'ESP_VEHICLESTANDSTILL_NOT_STANDSTILL', 1: 'ESP_VEHICLESTANDSTILL_STANDSTILL', 2: 'ESP_VEHICLESTANDSTILL_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_direction_0x235_235::Esp_vehiclestandstillType Espdirection0x235235::esp_vehiclestandstill(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  Esp_direction_0x235_235::Esp_vehiclestandstillType ret =  static_cast<Esp_direction_0x235_235::Esp_vehiclestandstillType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x235', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espdirection0x235235::checksum_0x235(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_whlmilgrearle', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': 'edge'}
int Espdirection0x235235::esp_whlmilgrearle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
