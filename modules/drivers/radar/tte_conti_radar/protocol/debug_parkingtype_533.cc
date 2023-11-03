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

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_parkingtype_533.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Debugparkingtype533::ID = 0x533;

// public
Debugparkingtype533::Debugparkingtype533() { Reset(); }

uint32_t Debugparkingtype533::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Debugparkingtype533::UpdateData(uint8_t* data) {
  set_p_parkingtype533_copy_7(data, parkingtype533_copy_7_);
  set_p_parkingtype533_copy_6(data, parkingtype533_copy_6_);
  set_p_parkingtype533_copy_5(data, parkingtype533_copy_5_);
  set_p_parkingtype533_copy_4(data, parkingtype533_copy_4_);
  set_p_parkingtype533_copy_3(data, parkingtype533_copy_3_);
  set_p_parkingtype533_copy_2(data, parkingtype533_copy_2_);
  set_p_parkingtype533_copy_1(data, parkingtype533_copy_1_);
  set_p_parkingtype533(data, parkingtype533_);
}

void Debugparkingtype533::Reset() {
  // TODO you should check this manually
  parkingtype533_copy_7_ = 4;
  parkingtype533_copy_6_ = 0;
  parkingtype533_copy_5_ = 0;
  parkingtype533_copy_4_ = 128;
  parkingtype533_copy_3_ = 0;
  parkingtype533_copy_2_ = 0;
  parkingtype533_copy_1_ = 0;
  parkingtype533_ = 0;
}

Debugparkingtype533* Debugparkingtype533::set_parkingtype533_copy_7(
    int parkingtype533_copy_7) {
  parkingtype533_copy_7_ = parkingtype533_copy_7;
  return this;
 }

// config detail: {'name': 'ParkingType533_Copy_7', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugparkingtype533::set_p_parkingtype533_copy_7(uint8_t* data,
    int parkingtype533_copy_7) {
  parkingtype533_copy_7 = ProtocolData::BoundedValue(0, 255, parkingtype533_copy_7);
  int x = parkingtype533_copy_7;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Debugparkingtype533* Debugparkingtype533::set_parkingtype533_copy_6(
    int parkingtype533_copy_6) {
  parkingtype533_copy_6_ = parkingtype533_copy_6;
  return this;
 }

// config detail: {'name': 'ParkingType533_Copy_6', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugparkingtype533::set_p_parkingtype533_copy_6(uint8_t* data,
    int parkingtype533_copy_6) {
  parkingtype533_copy_6 = ProtocolData::BoundedValue(0, 255, parkingtype533_copy_6);
  int x = parkingtype533_copy_6;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}


Debugparkingtype533* Debugparkingtype533::set_parkingtype533_copy_5(
    int parkingtype533_copy_5) {
  parkingtype533_copy_5_ = parkingtype533_copy_5;
  return this;
 }

// config detail: {'name': 'ParkingType533_Copy_5', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugparkingtype533::set_p_parkingtype533_copy_5(uint8_t* data,
    int parkingtype533_copy_5) {
  parkingtype533_copy_5 = ProtocolData::BoundedValue(0, 255, parkingtype533_copy_5);
  int x = parkingtype533_copy_5;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}


Debugparkingtype533* Debugparkingtype533::set_parkingtype533_copy_4(
    int parkingtype533_copy_4) {
  parkingtype533_copy_4_ = parkingtype533_copy_4;
  return this;
 }

// config detail: {'name': 'ParkingType533_Copy_4', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugparkingtype533::set_p_parkingtype533_copy_4(uint8_t* data,
    int parkingtype533_copy_4) {
  parkingtype533_copy_4 = ProtocolData::BoundedValue(0, 255, parkingtype533_copy_4);
  int x = parkingtype533_copy_4;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}


Debugparkingtype533* Debugparkingtype533::set_parkingtype533_copy_3(
    int parkingtype533_copy_3) {
  parkingtype533_copy_3_ = parkingtype533_copy_3;
  return this;
 }

// config detail: {'name': 'ParkingType533_Copy_3', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugparkingtype533::set_p_parkingtype533_copy_3(uint8_t* data,
    int parkingtype533_copy_3) {
  parkingtype533_copy_3 = ProtocolData::BoundedValue(0, 255, parkingtype533_copy_3);
  int x = parkingtype533_copy_3;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 8);
}


Debugparkingtype533* Debugparkingtype533::set_parkingtype533_copy_2(
    int parkingtype533_copy_2) {
  parkingtype533_copy_2_ = parkingtype533_copy_2;
  return this;
 }

// config detail: {'name': 'ParkingType533_Copy_2', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugparkingtype533::set_p_parkingtype533_copy_2(uint8_t* data,
    int parkingtype533_copy_2) {
  parkingtype533_copy_2 = ProtocolData::BoundedValue(0, 255, parkingtype533_copy_2);
  int x = parkingtype533_copy_2;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}


Debugparkingtype533* Debugparkingtype533::set_parkingtype533_copy_1(
    int parkingtype533_copy_1) {
  parkingtype533_copy_1_ = parkingtype533_copy_1;
  return this;
 }

// config detail: {'name': 'ParkingType533_Copy_1', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugparkingtype533::set_p_parkingtype533_copy_1(uint8_t* data,
    int parkingtype533_copy_1) {
  parkingtype533_copy_1 = ProtocolData::BoundedValue(0, 255, parkingtype533_copy_1);
  int x = parkingtype533_copy_1;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}


Debugparkingtype533* Debugparkingtype533::set_parkingtype533(
    int parkingtype533) {
  parkingtype533_ = parkingtype533;
  return this;
 }

// config detail: {'name': 'ParkingType533', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugparkingtype533::set_p_parkingtype533(uint8_t* data,
    int parkingtype533) {
  parkingtype533 = ProtocolData::BoundedValue(0, 255, parkingtype533);
  int x = parkingtype533;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
