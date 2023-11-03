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

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_extended_mode_736.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Debugextendedmode736::ID = 0x736;

// public
Debugextendedmode736::Debugextendedmode736() { Reset(); }

uint32_t Debugextendedmode736::GetPeriod() const {
  // TODO modify every protocol's period manually
  //事件型报文，周期设为0
  static const uint32_t PERIOD = 0;
  return PERIOD;
}

void Debugextendedmode736::UpdateData(uint8_t* data) {
  set_p_extendedmode736(data, extendedmode736_);
  set_p_extendedmode736_copy_7(data, extendedmode736_copy_7_);
  set_p_extendedmode736_copy_6(data, extendedmode736_copy_6_);
  set_p_extendedmode736_copy_5(data, extendedmode736_copy_5_);
  set_p_extendedmode736_copy_4(data, extendedmode736_copy_4_);
  set_p_extendedmode736_copy_3(data, extendedmode736_copy_3_);
  set_p_extendedmode736_copy_2(data, extendedmode736_copy_2_);
  set_p_extendedmode736_copy_1(data, extendedmode736_copy_1_);
}

void Debugextendedmode736::Reset() {
  // TODO you should check this manually
  extendedmode736_ = 0;
  extendedmode736_copy_7_ = 0;
  extendedmode736_copy_6_ = 0;
  extendedmode736_copy_5_ = 0;
  extendedmode736_copy_4_ = 0;
  extendedmode736_copy_3_ = 0;
  extendedmode736_copy_2_ = 0;
  extendedmode736_copy_1_ = 0;
}

Debugextendedmode736* Debugextendedmode736::set_extendedmode736(
    int extendedmode736) {
  extendedmode736_ = extendedmode736;
  return this;
 }

// config detail: {'name': 'ExtendedMode736', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugextendedmode736::set_p_extendedmode736(uint8_t* data,
    int extendedmode736) {
  extendedmode736 = ProtocolData::BoundedValue(0, 255, extendedmode736);
  int x = extendedmode736;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Debugextendedmode736* Debugextendedmode736::set_extendedmode736_copy_7(
    int extendedmode736_copy_7) {
  extendedmode736_copy_7_ = extendedmode736_copy_7;
  return this;
 }

// config detail: {'name': 'ExtendedMode736_Copy_7', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugextendedmode736::set_p_extendedmode736_copy_7(uint8_t* data,
    int extendedmode736_copy_7) {
  extendedmode736_copy_7 = ProtocolData::BoundedValue(0, 255, extendedmode736_copy_7);
  int x = extendedmode736_copy_7;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Debugextendedmode736* Debugextendedmode736::set_extendedmode736_copy_6(
    int extendedmode736_copy_6) {
  extendedmode736_copy_6_ = extendedmode736_copy_6;
  return this;
 }

// config detail: {'name': 'ExtendedMode736_Copy_6', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugextendedmode736::set_p_extendedmode736_copy_6(uint8_t* data,
    int extendedmode736_copy_6) {
  extendedmode736_copy_6 = ProtocolData::BoundedValue(0, 255, extendedmode736_copy_6);
  int x = extendedmode736_copy_6;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}


Debugextendedmode736* Debugextendedmode736::set_extendedmode736_copy_5(
    int extendedmode736_copy_5) {
  extendedmode736_copy_5_ = extendedmode736_copy_5;
  return this;
 }

// config detail: {'name': 'ExtendedMode736_Copy_5', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugextendedmode736::set_p_extendedmode736_copy_5(uint8_t* data,
    int extendedmode736_copy_5) {
  extendedmode736_copy_5 = ProtocolData::BoundedValue(0, 255, extendedmode736_copy_5);
  int x = extendedmode736_copy_5;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}


Debugextendedmode736* Debugextendedmode736::set_extendedmode736_copy_4(
    int extendedmode736_copy_4) {
  extendedmode736_copy_4_ = extendedmode736_copy_4;
  return this;
 }

// config detail: {'name': 'ExtendedMode736_Copy_4', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugextendedmode736::set_p_extendedmode736_copy_4(uint8_t* data,
    int extendedmode736_copy_4) {
  extendedmode736_copy_4 = ProtocolData::BoundedValue(0, 255, extendedmode736_copy_4);
  int x = extendedmode736_copy_4;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}


Debugextendedmode736* Debugextendedmode736::set_extendedmode736_copy_3(
    int extendedmode736_copy_3) {
  extendedmode736_copy_3_ = extendedmode736_copy_3;
  return this;
 }

// config detail: {'name': 'ExtendedMode736_Copy_3', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugextendedmode736::set_p_extendedmode736_copy_3(uint8_t* data,
    int extendedmode736_copy_3) {
  extendedmode736_copy_3 = ProtocolData::BoundedValue(0, 255, extendedmode736_copy_3);
  int x = extendedmode736_copy_3;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 8);
}


Debugextendedmode736* Debugextendedmode736::set_extendedmode736_copy_2(
    int extendedmode736_copy_2) {
  extendedmode736_copy_2_ = extendedmode736_copy_2;
  return this;
 }

// config detail: {'name': 'ExtendedMode736_Copy_2', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugextendedmode736::set_p_extendedmode736_copy_2(uint8_t* data,
    int extendedmode736_copy_2) {
  extendedmode736_copy_2 = ProtocolData::BoundedValue(0, 255, extendedmode736_copy_2);
  int x = extendedmode736_copy_2;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}


Debugextendedmode736* Debugextendedmode736::set_extendedmode736_copy_1(
    int extendedmode736_copy_1) {
  extendedmode736_copy_1_ = extendedmode736_copy_1;
  return this;
 }

// config detail: {'name': 'ExtendedMode736_Copy_1', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Debugextendedmode736::set_p_extendedmode736_copy_1(uint8_t* data,
    int extendedmode736_copy_1) {
  extendedmode736_copy_1 = ProtocolData::BoundedValue(0, 255, extendedmode736_copy_1);
  int x = extendedmode736_copy_1;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
