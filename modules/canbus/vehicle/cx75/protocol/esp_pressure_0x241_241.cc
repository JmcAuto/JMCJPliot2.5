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

#include "modules/canbus/vehicle/cx75/protocol/esp_pressure_0x241_241.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Esppressure0x241241::Esppressure0x241241() {}
const int32_t Esppressure0x241241::ID = 0x241;

void Esppressure0x241241::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_esp_pressure_0x241_241()->set_esp_trfcasetqlmtdbyesp(esp_trfcasetqlmtdbyesp(bytes, length));
  chassis->mutable_cx75()->mutable_esp_pressure_0x241_241()->set_esp_trfcasemodreqdbyesp(esp_trfcasemodreqdbyesp(bytes, length));
  chassis->mutable_cx75()->mutable_esp_pressure_0x241_241()->set_esp_presoffsetmastercylindervali(esp_presoffsetmastercylindervali(bytes, length));
  chassis->mutable_cx75()->mutable_esp_pressure_0x241_241()->set_esp_presoffsetmastercylinder(esp_presoffsetmastercylinder(bytes, length));
  chassis->mutable_cx75()->mutable_esp_pressure_0x241_241()->set_rolling_counter_0x241(rolling_counter_0x241(bytes, length));
  chassis->mutable_cx75()->mutable_esp_pressure_0x241_241()->set_esp_master_cylinder_pressure_sta(esp_master_cylinder_pressure_sta(bytes, length));
  chassis->mutable_cx75()->mutable_esp_pressure_0x241_241()->set_checksum_0x241(checksum_0x241(bytes, length));
  chassis->mutable_cx75()->mutable_esp_pressure_0x241_241()->set_esp_master_cylinder_pressure(esp_master_cylinder_pressure(bytes, length));
}

// config detail: {'name': 'esp_trfcasetqlmtdbyesp', 'offset': 0.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|4095]', 'bit': 11, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
int Esppressure0x241241::esp_trfcasetqlmtdbyesp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_trfcasemodreqdbyesp', 'enum': {0: 'ESP_TRFCASEMODREQDBYESP_NO_REQUEST', 1: 'ESP_TRFCASEMODREQDBYESP_FAST_OPEN_REQUEST', 2: 'ESP_TRFCASEMODREQDBYESP_TORQUE_UPPER_LIMIT_REQUEST', 3: 'ESP_TRFCASEMODREQDBYESP_FAILURE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_pressure_0x241_241::Esp_trfcasemodreqdbyespType Esppressure0x241241::esp_trfcasemodreqdbyesp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 2);

  Esp_pressure_0x241_241::Esp_trfcasemodreqdbyespType ret =  static_cast<Esp_pressure_0x241_241::Esp_trfcasemodreqdbyespType>(x);
  return ret;
}

// config detail: {'name': 'esp_presoffsetmastercylindervali', 'enum': {0: 'ESP_PRESOFFSETMASTERCYLINDERVALI_VALID', 1: 'ESP_PRESOFFSETMASTERCYLINDERVALI_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_pressure_0x241_241::Esp_presoffsetmastercylindervaliType Esppressure0x241241::esp_presoffsetmastercylindervali(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 1);

  Esp_pressure_0x241_241::Esp_presoffsetmastercylindervaliType ret =  static_cast<Esp_pressure_0x241_241::Esp_presoffsetmastercylindervaliType>(x);
  return ret;
}

// config detail: {'name': 'esp_presoffsetmastercylinder', 'enum': {31: 'ESP_PRESOFFSETMASTERCYLINDER_INVALID'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': -15.0, 'physical_range': '[-15|15]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'bar'}
int Esppressure0x241241::esp_presoffsetmastercylinder(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(3, 5);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x241', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esppressure0x241241::rolling_counter_0x241(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_master_cylinder_pressure_sta', 'enum': {0: 'ESP_MASTER_CYLINDER_PRESSURE_STA_VALID', 1: 'ESP_MASTER_CYLINDER_PRESSURE_STA_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Esp_pressure_0x241_241::Esp_master_cylinder_pressure_staType Esppressure0x241241::esp_master_cylinder_pressure_sta(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  Esp_pressure_0x241_241::Esp_master_cylinder_pressure_staType ret =  static_cast<Esp_pressure_0x241_241::Esp_master_cylinder_pressure_staType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x241', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Esppressure0x241241::checksum_0x241(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'esp_master_cylinder_pressure', 'offset': 0.0, 'precision': 0.0625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|255.9375]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'bar'}
double Esppressure0x241241::esp_master_cylinder_pressure(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.062500;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
