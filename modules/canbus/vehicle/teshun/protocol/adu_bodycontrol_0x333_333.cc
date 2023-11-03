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

#include "modules/canbus/vehicle/teshun/protocol/adu_bodycontrol_0x333_333.h"

#include "modules/drivers/canbus/common/byte.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

const int32_t Adubodycontrol0x333333::ID = 0x333;

// public
Adubodycontrol0x333333::Adubodycontrol0x333333() { Reset(); }

uint32_t Adubodycontrol0x333333::GetPeriod() const {
  // TODO modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Adubodycontrol0x333333::UpdateData(uint8_t* data) {
  set_p_adu_rearfoglampcontrol(data, adu_rearfoglampcontrol_);
  set_p_adu_frontfoglampcontrol(data, adu_frontfoglampcontrol_);
  set_p_adu_washercontrol(data, adu_washercontrol_);
  set_p_adu_wipercontrol(data, adu_wipercontrol_);
  set_p_adu_doorcontrol(data, adu_doorcontrol_);
  set_p_adu_horncontrol(data, adu_horncontrol_);
  set_p_adu_highbeamlampcontrol(data, adu_highbeamlampcontrol_);
  set_p_adu_lowbeamlampcontrol(data, adu_lowbeamlampcontrol_);
  set_p_adu_positionlampcontrol(data, adu_positionlampcontrol_);
  set_p_adu_hazardlampcontrol(data, adu_hazardlampcontrol_);
  set_p_adu_rightturnlampcontrol(data, adu_rightturnlampcontrol_);
  set_p_adu_leftturnlampcontrol(data, adu_leftturnlampcontrol_);
}

void Adubodycontrol0x333333::Reset() {
  // TODO you should check this manually
  adu_rearfoglampcontrol_ = Adu_bodycontrol_0x333_333::ADU_REARFOGLAMPCONTROL_INVALID;
  adu_frontfoglampcontrol_ = Adu_bodycontrol_0x333_333::ADU_FRONTFOGLAMPCONTROL_INVALID;
  adu_washercontrol_ = Adu_bodycontrol_0x333_333::ADU_WASHERCONTROL_INVALID;
  adu_wipercontrol_ = Adu_bodycontrol_0x333_333::ADU_WIPERCONTROL_INVALID;
  adu_doorcontrol_ = Adu_bodycontrol_0x333_333::ADU_DOORCONTROL_INVALID;
  adu_horncontrol_ = Adu_bodycontrol_0x333_333::ADU_HORNCONTROL_INVALID;
  adu_highbeamlampcontrol_ = Adu_bodycontrol_0x333_333::ADU_HIGHBEAMLAMPCONTROL_INVALID;
  adu_lowbeamlampcontrol_ = Adu_bodycontrol_0x333_333::ADU_LOWBEAMLAMPCONTROL_INVALID;
  adu_positionlampcontrol_ = Adu_bodycontrol_0x333_333::ADU_POSITIONLAMPCONTROL_INVALID;
  adu_hazardlampcontrol_ = Adu_bodycontrol_0x333_333::ADU_HAZARDLAMPCONTROL_INVALID;
  adu_rightturnlampcontrol_ = Adu_bodycontrol_0x333_333::ADU_RIGHTTURNLAMPCONTROL_INVALID;
  adu_leftturnlampcontrol_ = Adu_bodycontrol_0x333_333::ADU_LEFTTURNLAMPCONTROL_INVALID;
}

Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_rearfoglampcontrol(
    Adu_bodycontrol_0x333_333::Adu_rearfoglampcontrolType adu_rearfoglampcontrol) {
  adu_rearfoglampcontrol_ = adu_rearfoglampcontrol;
  return this;
 }

// config detail: {'name': 'ADU_RearfogLampControl', 'enum': {0: 'ADU_REARFOGLAMPCONTROL_INVALID', 1: 'ADU_REARFOGLAMPCONTROL_OFF', 2: 'ADU_REARFOGLAMPCONTROL_ON', 3: 'ADU_REARFOGLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_rearfoglampcontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_rearfoglampcontrolType adu_rearfoglampcontrol) {
  int x = adu_rearfoglampcontrol;

  Byte to_set(data + 4);
  to_set.set_value(x, 4, 2);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_frontfoglampcontrol(
    Adu_bodycontrol_0x333_333::Adu_frontfoglampcontrolType adu_frontfoglampcontrol) {
  adu_frontfoglampcontrol_ = adu_frontfoglampcontrol;
  return this;
 }

// config detail: {'name': 'ADU_FrontfogLampControl', 'enum': {0: 'ADU_FRONTFOGLAMPCONTROL_INVALID', 1: 'ADU_FRONTFOGLAMPCONTROL_OFF', 2: 'ADU_FRONTFOGLAMPCONTROL_ON', 3: 'ADU_FRONTFOGLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_frontfoglampcontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_frontfoglampcontrolType adu_frontfoglampcontrol) {
  int x = adu_frontfoglampcontrol;

  Byte to_set(data + 4);
  to_set.set_value(x, 6, 2);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_washercontrol(
    Adu_bodycontrol_0x333_333::Adu_washercontrolType adu_washercontrol) {
  adu_washercontrol_ = adu_washercontrol;
  return this;
 }

// config detail: {'name': 'ADU_WasherControl', 'enum': {0: 'ADU_WASHERCONTROL_INVALID', 1: 'ADU_WASHERCONTROL_OFF', 2: 'ADU_WASHERCONTROL_ON', 3: 'ADU_WASHERCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_washercontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_washercontrolType adu_washercontrol) {
  int x = adu_washercontrol;

  Byte to_set(data + 3);
  to_set.set_value(x, 3, 2);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_wipercontrol(
    Adu_bodycontrol_0x333_333::Adu_wipercontrolType adu_wipercontrol) {
  adu_wipercontrol_ = adu_wipercontrol;
  return this;
 }

// config detail: {'name': 'ADU_WiperControl', 'enum': {0: 'ADU_WIPERCONTROL_INVALID', 1: 'ADU_WIPERCONTROL_OFF', 2: 'ADU_WIPERCONTROL_LOW_SPEED_A3_A8_B5_CD_CB_B5_A3_A9', 3: 'ADU_WIPERCONTROL_HIGH_SPEED_A3_A8_B8_DF_CB_B5_A3_A9', 7: 'ADU_WIPERCONTROL_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_wipercontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_wipercontrolType adu_wipercontrol) {
  int x = adu_wipercontrol;

  Byte to_set(data + 3);
  to_set.set_value(x, 5, 3);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_doorcontrol(
    Adu_bodycontrol_0x333_333::Adu_doorcontrolType adu_doorcontrol) {
  adu_doorcontrol_ = adu_doorcontrol;
  return this;
 }

// config detail: {'name': 'ADU_DoorControl', 'enum': {0: 'ADU_DOORCONTROL_INVALID', 1: 'ADU_DOORCONTROL_LOCK', 2: 'ADU_DOORCONTROL_UNLOCK', 3: 'ADU_DOORCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_doorcontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_doorcontrolType adu_doorcontrol) {
  int x = adu_doorcontrol;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 2);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_horncontrol(
    Adu_bodycontrol_0x333_333::Adu_horncontrolType adu_horncontrol) {
  adu_horncontrol_ = adu_horncontrol;
  return this;
 }

// config detail: {'name': 'ADU_HornControl', 'enum': {0: 'ADU_HORNCONTROL_INVALID', 1: 'ADU_HORNCONTROL_OFF', 2: 'ADU_HORNCONTROL_ON', 3: 'ADU_HORNCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_horncontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_horncontrolType adu_horncontrol) {
  int x = adu_horncontrol;

  Byte to_set(data + 2);
  to_set.set_value(x, 6, 2);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_highbeamlampcontrol(
    Adu_bodycontrol_0x333_333::Adu_highbeamlampcontrolType adu_highbeamlampcontrol) {
  adu_highbeamlampcontrol_ = adu_highbeamlampcontrol;
  return this;
 }

// config detail: {'name': 'ADU_HighBeamLampControl', 'enum': {0: 'ADU_HIGHBEAMLAMPCONTROL_INVALID', 1: 'ADU_HIGHBEAMLAMPCONTROL_OFF', 2: 'ADU_HIGHBEAMLAMPCONTROL_ON', 3: 'ADU_HIGHBEAMLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_highbeamlampcontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_highbeamlampcontrolType adu_highbeamlampcontrol) {
  int x = adu_highbeamlampcontrol;

  Byte to_set(data + 1);
  to_set.set_value(x, 2, 2);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_lowbeamlampcontrol(
    Adu_bodycontrol_0x333_333::Adu_lowbeamlampcontrolType adu_lowbeamlampcontrol) {
  adu_lowbeamlampcontrol_ = adu_lowbeamlampcontrol;
  return this;
 }

// config detail: {'name': 'ADU_LowBeamLampControl', 'enum': {0: 'ADU_LOWBEAMLAMPCONTROL_INVALID', 1: 'ADU_LOWBEAMLAMPCONTROL_OFF', 2: 'ADU_LOWBEAMLAMPCONTROL_ON', 3: 'ADU_LOWBEAMLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_lowbeamlampcontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_lowbeamlampcontrolType adu_lowbeamlampcontrol) {
  int x = adu_lowbeamlampcontrol;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 2);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_positionlampcontrol(
    Adu_bodycontrol_0x333_333::Adu_positionlampcontrolType adu_positionlampcontrol) {
  adu_positionlampcontrol_ = adu_positionlampcontrol;
  return this;
 }

// config detail: {'name': 'ADU_PositionLampControl', 'enum': {0: 'ADU_POSITIONLAMPCONTROL_INVALID', 1: 'ADU_POSITIONLAMPCONTROL_OFF', 2: 'ADU_POSITIONLAMPCONTROL_ON', 3: 'ADU_POSITIONLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_positionlampcontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_positionlampcontrolType adu_positionlampcontrol) {
  int x = adu_positionlampcontrol;

  Byte to_set(data + 1);
  to_set.set_value(x, 6, 2);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_hazardlampcontrol(
    Adu_bodycontrol_0x333_333::Adu_hazardlampcontrolType adu_hazardlampcontrol) {
  adu_hazardlampcontrol_ = adu_hazardlampcontrol;
  return this;
 }

// config detail: {'name': 'ADU_HazardLampControl', 'enum': {0: 'ADU_HAZARDLAMPCONTROL_INVALID', 1: 'ADU_HAZARDLAMPCONTROL_OFF', 2: 'ADU_HAZARDLAMPCONTROL_BLINK', 3: 'ADU_HAZARDLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_hazardlampcontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_hazardlampcontrolType adu_hazardlampcontrol) {
  int x = adu_hazardlampcontrol;

  Byte to_set(data + 0);
  to_set.set_value(x, 2, 2);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_rightturnlampcontrol(
    Adu_bodycontrol_0x333_333::Adu_rightturnlampcontrolType adu_rightturnlampcontrol) {
  adu_rightturnlampcontrol_ = adu_rightturnlampcontrol;
  return this;
 }

// config detail: {'name': 'ADU_RightTurnLampControl', 'enum': {0: 'ADU_RIGHTTURNLAMPCONTROL_INVALID', 1: 'ADU_RIGHTTURNLAMPCONTROL_OFF', 2: 'ADU_RIGHTTURNLAMPCONTROL_BLINK', 3: 'ADU_RIGHTTURNLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_rightturnlampcontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_rightturnlampcontrolType adu_rightturnlampcontrol) {
  int x = adu_rightturnlampcontrol;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 2);
}


Adubodycontrol0x333333* Adubodycontrol0x333333::set_adu_leftturnlampcontrol(
    Adu_bodycontrol_0x333_333::Adu_leftturnlampcontrolType adu_leftturnlampcontrol) {
  adu_leftturnlampcontrol_ = adu_leftturnlampcontrol;
  return this;
 }

// config detail: {'name': 'ADU_LeftTurnLampControl', 'enum': {0: 'ADU_LEFTTURNLAMPCONTROL_INVALID', 1: 'ADU_LEFTTURNLAMPCONTROL_OFF', 2: 'ADU_LEFTTURNLAMPCONTROL_BLINK', 3: 'ADU_LEFTTURNLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
void Adubodycontrol0x333333::set_p_adu_leftturnlampcontrol(uint8_t* data,
    Adu_bodycontrol_0x333_333::Adu_leftturnlampcontrolType adu_leftturnlampcontrol) {
  int x = adu_leftturnlampcontrol;

  Byte to_set(data + 0);
  to_set.set_value(x, 6, 2);
}

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
