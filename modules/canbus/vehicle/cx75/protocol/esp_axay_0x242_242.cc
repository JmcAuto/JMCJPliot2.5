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

#include "modules/canbus/vehicle/cx75/protocol/esp_axay_0x242_242.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Espaxay0x242242::Espaxay0x242242() {}
const int32_t Espaxay0x242242::ID = 0x242;

void Espaxay0x242242::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_esp_axay_0x242_242()->set_esp_alat(esp_alat(bytes, length));
  chassis->mutable_cx75()->mutable_esp_axay_0x242_242()->set_esp_yawrate(esp_yawrate(bytes, length));
  chassis->mutable_cx75()->mutable_esp_axay_0x242_242()->set_rolling_counter_0x242(rolling_counter_0x242(bytes, length));
  chassis->mutable_cx75()->mutable_esp_axay_0x242_242()->set_esp_yawratestatus(esp_yawratestatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_axay_0x242_242()->set_esp_alatstatus(esp_alatstatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_axay_0x242_242()->set_esp_algtstatus(esp_algtstatus(bytes, length));
  chassis->mutable_cx75()->mutable_esp_axay_0x242_242()->set_checksum_0x242(checksum_0x242(bytes, length));
  chassis->mutable_cx75()->mutable_esp_axay_0x242_242()->set_esp_algt(esp_algt(bytes, length));
}

// config detail: {'name': 'esp_alat', 'offset': -21.593092, 'precision': 0.027127, 'len': 12, 'is_signed_var': False, 'physical_range': '[-21.593092|21.593092]', 'bit': 11, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s^2'}
double Espaxay0x242242::esp_alat(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.027127 + -21.593092;
  return ret;
}

// config detail: {'name': 'esp_yawrate', 'offset': -2.0942132, 'precision': 0.0021326, 'len': 12, 'is_signed_var': False, 'physical_range': '[-2.0942132|2.0942132]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'rad s'}
double Espaxay0x242242::esp_yawrate(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.002133 + -2.094213;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x242', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espaxay0x242242::rolling_counter_0x242(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_yawratestatus', 'enum': {0: 'ESP_YAWRATESTATUS_OK', 1: 'ESP_YAWRATESTATUS_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_axay_0x242_242::Esp_yawratestatusType Espaxay0x242242::esp_yawratestatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Esp_axay_0x242_242::Esp_yawratestatusType ret =  static_cast<Esp_axay_0x242_242::Esp_yawratestatusType>(x);
  return ret;
}

// config detail: {'name': 'esp_alatstatus', 'enum': {0: 'ESP_ALATSTATUS_OK', 1: 'ESP_ALATSTATUS_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_axay_0x242_242::Esp_alatstatusType Espaxay0x242242::esp_alatstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Esp_axay_0x242_242::Esp_alatstatusType ret =  static_cast<Esp_axay_0x242_242::Esp_alatstatusType>(x);
  return ret;
}

// config detail: {'name': 'esp_algtstatus', 'enum': {0: 'ESP_ALGTSTATUS_OK', 1: 'ESP_ALGTSTATUS_FAULT'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_axay_0x242_242::Esp_algtstatusType Espaxay0x242242::esp_algtstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  Esp_axay_0x242_242::Esp_algtstatusType ret =  static_cast<Esp_axay_0x242_242::Esp_algtstatusType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x242', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Espaxay0x242242::checksum_0x242(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_algt', 'offset': -21.593092, 'precision': 0.027127, 'len': 12, 'is_signed_var': False, 'physical_range': '[-21.593092|21.593092]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm s^2'}
double Espaxay0x242242::esp_algt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.027127 + -21.593092;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
