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

#include "modules/canbus/vehicle/teshun/protocol/adu_controldrive_0x120_120.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Aducontroldrive0x120120::ID = 0x120;

// public
Aducontroldrive0x120120::Aducontroldrive0x120120() { Reset(); }

uint32_t Aducontroldrive0x120120::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Aducontroldrive0x120120::UpdateData(uint8_t* data) {
  
  set_p_rolling_counter_0x120(data, rolling_counter_0x120_);
  set_p_adu_controtorque_rawflag(data, adu_controtorque_rawflag_);
  set_p_adu_targetdrving_torque(data, adu_targetdrving_torque_);
  set_p_adu_controtorque_enable(data, adu_controtorque_enable_);
  set_p_adu_targetgear_position(data, adu_targetgear_position_);
  set_p_adu_targetgear_enable(data, adu_targetgear_enable_);
set_p_checksum_0x120(data, checksum_0x120_);
}

void Aducontroldrive0x120120::Reset() {
  // TODO you should check this manually
  checksum_0x120_ = 0;
  rolling_counter_0x120_ = 0;
  adu_controtorque_rawflag_ = Adu_controldrive_0x120_120::ADU_CONTROTORQUE_RAWFLAG_NO_FILTERS;
  adu_targetdrving_torque_ = 0;
  adu_controtorque_enable_ = Adu_controldrive_0x120_120::ADU_CONTROTORQUE_ENABLE_DISABLE;
  adu_targetgear_position_ = Adu_controldrive_0x120_120::ADU_TARGETGEAR_POSITION_INITIAL;
  adu_targetgear_enable_ = Adu_controldrive_0x120_120::ADU_TARGETGEAR_ENABLE_DISABLE;
}

Aducontroldrive0x120120* Aducontroldrive0x120120::set_checksum_0x120(
    int checksum_0x120) {
  checksum_0x120_ = checksum_0x120;
  return this;
 }

// config detail: {'name': 'Checksum_0x120', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Aducontroldrive0x120120::set_p_checksum_0x120(uint8_t* data,
    int checksum_0x120) {
checksum_0x120 = (data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]);
  checksum_0x120 = ProtocolData::BoundedValue(0, 255, checksum_0x120);
  int x = checksum_0x120;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Aducontroldrive0x120120* Aducontroldrive0x120120::set_rolling_counter_0x120(
    int rolling_counter_0x120) {
  rolling_counter_0x120_ = rolling_counter_0x120;
  return this;
 }

// config detail: {'name': 'Rolling_counter_0x120', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
void Aducontroldrive0x120120::set_p_rolling_counter_0x120(uint8_t* data,
    int rolling_counter_0x120) {
  rolling_counter_0x120 = ProtocolData::BoundedValue(0, 15, rolling_counter_0x120);
  int x = rolling_counter_0x120;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}


Aducontroldrive0x120120* Aducontroldrive0x120120::set_adu_controtorque_rawflag(
    Adu_controldrive_0x120_120::Adu_controtorque_rawflagType adu_controtorque_rawflag) {
  adu_controtorque_rawflag_ = adu_controtorque_rawflag;
  return this;
 }

// config detail: {'name': 'ADU_ControTorque_RAWFlag', 'enum': {0: 'ADU_CONTROTORQUE_RAWFLAG_NO_FILTERS', 1: 'ADU_CONTROTORQUE_RAWFLAG_FILTERS'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Aducontroldrive0x120120::set_p_adu_controtorque_rawflag(uint8_t* data,
    Adu_controldrive_0x120_120::Adu_controtorque_rawflagType adu_controtorque_rawflag) {
  int x = adu_controtorque_rawflag;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}


Aducontroldrive0x120120* Aducontroldrive0x120120::set_adu_targetdrving_torque(
    int adu_targetdrving_torque) {
  adu_targetdrving_torque_ = adu_targetdrving_torque;
  return this;
 }

// config detail: {'name': 'ADU_TargetDrving_Torque', 'offset': -10000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-10000|10000]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
void Aducontroldrive0x120120::set_p_adu_targetdrving_torque(uint8_t* data,
    int adu_targetdrving_torque) {
  adu_targetdrving_torque = ProtocolData::BoundedValue(-10000, 10000, adu_targetdrving_torque);
  int x = (adu_targetdrving_torque - -10000.000000);
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}


Aducontroldrive0x120120* Aducontroldrive0x120120::set_adu_controtorque_enable(
    Adu_controldrive_0x120_120::Adu_controtorque_enableType adu_controtorque_enable) {
  adu_controtorque_enable_ = adu_controtorque_enable;
  return this;
 }

// config detail: {'name': 'ADU_ControTorque_Enable', 'enum': {0: 'ADU_CONTROTORQUE_ENABLE_DISABLE', 1: 'ADU_CONTROTORQUE_ENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Aducontroldrive0x120120::set_p_adu_controtorque_enable(uint8_t* data,
    Adu_controldrive_0x120_120::Adu_controtorque_enableType adu_controtorque_enable) {
  int x = adu_controtorque_enable;

  Byte to_set(data + 0);
  to_set.set_value(x, 3, 1);
}


Aducontroldrive0x120120* Aducontroldrive0x120120::set_adu_targetgear_position(
    Adu_controldrive_0x120_120::Adu_targetgear_positionType adu_targetgear_position) {
  adu_targetgear_position_ = adu_targetgear_position;
  return this;
 }

// config detail: {'name': 'ADU_TargetGear_Position', 'enum': {0: 'ADU_TARGETGEAR_POSITION_INITIAL', 1: 'ADU_TARGETGEAR_POSITION_P_PARK', 2: 'ADU_TARGETGEAR_POSITION_R_REVERSE', 3: 'ADU_TARGETGEAR_POSITION_N_NEUTRAL', 4: 'ADU_TARGETGEAR_POSITION_D_DRIVE', 5: 'ADU_TARGETGEAR_POSITION_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 6, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Aducontroldrive0x120120::set_p_adu_targetgear_position(uint8_t* data,
    Adu_controldrive_0x120_120::Adu_targetgear_positionType adu_targetgear_position) {
  int x = adu_targetgear_position;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 3);
}


Aducontroldrive0x120120* Aducontroldrive0x120120::set_adu_targetgear_enable(
    Adu_controldrive_0x120_120::Adu_targetgear_enableType adu_targetgear_enable) {
  adu_targetgear_enable_ = adu_targetgear_enable;
  return this;
 }

// config detail: {'name': 'ADU_TargetGear_Enable', 'enum': {0: 'ADU_TARGETGEAR_ENABLE_DISABLE', 1: 'ADU_TARGETGEAR_ENABLE_ENABLE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Aducontroldrive0x120120::set_p_adu_targetgear_enable(uint8_t* data,
    Adu_controldrive_0x120_120::Adu_targetgear_enableType adu_targetgear_enable) {
  int x = adu_targetgear_enable;

  Byte to_set(data + 0);
  to_set.set_value(x, 7, 1);
}

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
