/******************************************************************************
 * Copyright 2017 The JmcAuto Authors. All Rights Reserved.
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

#include "modules/drivers/gnss_m2/protocol/odometer_input_99.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Odometerinput99::ID = 0x99;

// public
Odometerinput99::Odometerinput99() { Reset(); }

uint32_t Odometerinput99::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Odometerinput99::UpdateData(uint8_t* data) {

  set_p_vehspd_input(data, vehspd_input_);
  set_p_gearposition_input(data, gearposition_input_);
  set_p_checksum(data, checksum_);
}

void Odometerinput99::Reset() {
  // TODO you should check this manually
  checksum_ = 0;
  vehspd_input_ = 0.0;
  gearposition_input_ = 0;
}

Odometerinput99* Odometerinput99::set_checksum(
    int checksum) {
  checksum_ = checksum;
  return this;
 }

// config detail: {'description': 'checksum=Sum(Byte0:Byte6)^0xff', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name': 'CheckSum', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Odometerinput99::set_p_checksum(uint8_t* data,
    int checksum) {

  checksum = (data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6])^0xff;
  checksum = ProtocolData::BoundedValue(0, 255, checksum);
  int x = checksum;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Odometerinput99* Odometerinput99::set_vehspd_input(
    double vehspd_input) {
  vehspd_input_ = vehspd_input;
  return this;
 }

// config detail: {'name': 'VehSpd_input', 'offset': 0.0, 'precision': 0.00390625, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|256]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
void Odometerinput99::set_p_vehspd_input(uint8_t* data,
    double vehspd_input) {
  vehspd_input = ProtocolData::BoundedValue(0.0, 256.0, vehspd_input);
  int x = vehspd_input / 0.003906;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 3);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
}


Odometerinput99* Odometerinput99::set_gearposition_input(
    int gearposition_input) {
  gearposition_input_ = gearposition_input;
  return this;
 }

// config detail: {'name': 'GearPosition_input', 'offset': 0.0, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'physical_range': '[0|2]', 'bit': 2, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Odometerinput99::set_p_gearposition_input(uint8_t* data,
    int gearposition_input) {
  gearposition_input = ProtocolData::BoundedValue(0, 2, gearposition_input);
  int x = gearposition_input;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 3);

  
}

}  // namespace m2
}  // namespace canbus
}  // namespace jmc_auto
