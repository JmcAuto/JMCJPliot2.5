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

#include "modules/canbus/vehicle/cx75/protocol/esp_tq_0x217_217.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Esptq0x217217::Esptq0x217217() {}
const int32_t Esptq0x217217::ID = 0x217;

void Esptq0x217217::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_esp_tq_0x217_217()->set_esp_tqdecreqdbyespslow(esp_tqdecreqdbyespslow(bytes, length));
  chassis->mutable_cx75()->mutable_esp_tq_0x217_217()->set_esp_tqinccreqdbyesp(esp_tqinccreqdbyesp(bytes, length));
  chassis->mutable_cx75()->mutable_esp_tq_0x217_217()->set_rolling_counter_0x217(rolling_counter_0x217(bytes, length));
  chassis->mutable_cx75()->mutable_esp_tq_0x217_217()->set_esp_trqinc_req(esp_trqinc_req(bytes, length));
  chassis->mutable_cx75()->mutable_esp_tq_0x217_217()->set_esp_trqred_req(esp_trqred_req(bytes, length));
  chassis->mutable_cx75()->mutable_esp_tq_0x217_217()->set_checksum_0x217(checksum_0x217(bytes, length));
  chassis->mutable_cx75()->mutable_esp_tq_0x217_217()->set_esp_tqdecreqdbyespfast(esp_tqdecreqdbyespfast(bytes, length));
}

// config detail: {'name': 'esp_tqdecreqdbyespslow', 'offset': -1000.0, 'precision': 0.5, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|1000]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
double Esptq0x217217::esp_tqdecreqdbyespslow(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.500000 + -1000.000000;
  return ret;
}

// config detail: {'name': 'esp_tqinccreqdbyesp', 'offset': -1000.0, 'precision': 0.5, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|1000]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
double Esptq0x217217::esp_tqinccreqdbyesp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.500000 + -1000.000000;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x217', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esptq0x217217::rolling_counter_0x217(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_trqinc_req', 'enum': {0: 'ESP_TRQINC_REQ_INACTIVE', 1: 'ESP_TRQINC_REQ_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 52, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_tq_0x217_217::Esp_trqinc_reqType Esptq0x217217::esp_trqinc_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  Esp_tq_0x217_217::Esp_trqinc_reqType ret =  static_cast<Esp_tq_0x217_217::Esp_trqinc_reqType>(x);
  return ret;
}

// config detail: {'name': 'esp_trqred_req', 'enum': {0: 'ESP_TRQRED_REQ_INACTIVE', 1: 'ESP_TRQRED_REQ_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_tq_0x217_217::Esp_trqred_reqType Esptq0x217217::esp_trqred_req(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  Esp_tq_0x217_217::Esp_trqred_reqType ret =  static_cast<Esp_tq_0x217_217::Esp_trqred_reqType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x217', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esptq0x217217::checksum_0x217(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_tqdecreqdbyespfast', 'offset': -1000.0, 'precision': 0.5, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|1000]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Nm'}
double Esptq0x217217::esp_tqdecreqdbyespfast(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.500000 + -1000.000000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
