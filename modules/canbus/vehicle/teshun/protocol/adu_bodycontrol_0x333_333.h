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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_ADU_BODYCONTROL_0X333_333_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_ADU_BODYCONTROL_0X333_333_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Adubodycontrol0x333333 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Adubodycontrol0x333333();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'ADU_RearfogLampControl', 'enum': {0: 'ADU_REARFOGLAMPCONTROL_INVALID', 1: 'ADU_REARFOGLAMPCONTROL_OFF', 2: 'ADU_REARFOGLAMPCONTROL_ON', 3: 'ADU_REARFOGLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_rearfoglampcontrol(Adu_bodycontrol_0x333_333::Adu_rearfoglampcontrolType adu_rearfoglampcontrol);

  // config detail: {'name': 'ADU_FrontfogLampControl', 'enum': {0: 'ADU_FRONTFOGLAMPCONTROL_INVALID', 1: 'ADU_FRONTFOGLAMPCONTROL_OFF', 2: 'ADU_FRONTFOGLAMPCONTROL_ON', 3: 'ADU_FRONTFOGLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_frontfoglampcontrol(Adu_bodycontrol_0x333_333::Adu_frontfoglampcontrolType adu_frontfoglampcontrol);

  // config detail: {'name': 'ADU_WasherControl', 'enum': {0: 'ADU_WASHERCONTROL_INVALID', 1: 'ADU_WASHERCONTROL_OFF', 2: 'ADU_WASHERCONTROL_ON', 3: 'ADU_WASHERCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_washercontrol(Adu_bodycontrol_0x333_333::Adu_washercontrolType adu_washercontrol);

  // config detail: {'name': 'ADU_WiperControl', 'enum': {0: 'ADU_WIPERCONTROL_INVALID', 1: 'ADU_WIPERCONTROL_OFF', 2: 'ADU_WIPERCONTROL_LOW_SPEED_A3_A8_B5_CD_CB_B5_A3_A9', 3: 'ADU_WIPERCONTROL_HIGH_SPEED_A3_A8_B8_DF_CB_B5_A3_A9', 7: 'ADU_WIPERCONTROL_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_wipercontrol(Adu_bodycontrol_0x333_333::Adu_wipercontrolType adu_wipercontrol);

  // config detail: {'name': 'ADU_DoorControl', 'enum': {0: 'ADU_DOORCONTROL_INVALID', 1: 'ADU_DOORCONTROL_LOCK', 2: 'ADU_DOORCONTROL_UNLOCK', 3: 'ADU_DOORCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_doorcontrol(Adu_bodycontrol_0x333_333::Adu_doorcontrolType adu_doorcontrol);

  // config detail: {'name': 'ADU_HornControl', 'enum': {0: 'ADU_HORNCONTROL_INVALID', 1: 'ADU_HORNCONTROL_OFF', 2: 'ADU_HORNCONTROL_ON', 3: 'ADU_HORNCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_horncontrol(Adu_bodycontrol_0x333_333::Adu_horncontrolType adu_horncontrol);

  // config detail: {'name': 'ADU_HighBeamLampControl', 'enum': {0: 'ADU_HIGHBEAMLAMPCONTROL_INVALID', 1: 'ADU_HIGHBEAMLAMPCONTROL_OFF', 2: 'ADU_HIGHBEAMLAMPCONTROL_ON', 3: 'ADU_HIGHBEAMLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_highbeamlampcontrol(Adu_bodycontrol_0x333_333::Adu_highbeamlampcontrolType adu_highbeamlampcontrol);

  // config detail: {'name': 'ADU_LowBeamLampControl', 'enum': {0: 'ADU_LOWBEAMLAMPCONTROL_INVALID', 1: 'ADU_LOWBEAMLAMPCONTROL_OFF', 2: 'ADU_LOWBEAMLAMPCONTROL_ON', 3: 'ADU_LOWBEAMLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_lowbeamlampcontrol(Adu_bodycontrol_0x333_333::Adu_lowbeamlampcontrolType adu_lowbeamlampcontrol);

  // config detail: {'name': 'ADU_PositionLampControl', 'enum': {0: 'ADU_POSITIONLAMPCONTROL_INVALID', 1: 'ADU_POSITIONLAMPCONTROL_OFF', 2: 'ADU_POSITIONLAMPCONTROL_ON', 3: 'ADU_POSITIONLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_positionlampcontrol(Adu_bodycontrol_0x333_333::Adu_positionlampcontrolType adu_positionlampcontrol);

  // config detail: {'name': 'ADU_HazardLampControl', 'enum': {0: 'ADU_HAZARDLAMPCONTROL_INVALID', 1: 'ADU_HAZARDLAMPCONTROL_OFF', 2: 'ADU_HAZARDLAMPCONTROL_BLINK', 3: 'ADU_HAZARDLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_hazardlampcontrol(Adu_bodycontrol_0x333_333::Adu_hazardlampcontrolType adu_hazardlampcontrol);

  // config detail: {'name': 'ADU_RightTurnLampControl', 'enum': {0: 'ADU_RIGHTTURNLAMPCONTROL_INVALID', 1: 'ADU_RIGHTTURNLAMPCONTROL_OFF', 2: 'ADU_RIGHTTURNLAMPCONTROL_BLINK', 3: 'ADU_RIGHTTURNLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_rightturnlampcontrol(Adu_bodycontrol_0x333_333::Adu_rightturnlampcontrolType adu_rightturnlampcontrol);

  // config detail: {'name': 'ADU_LeftTurnLampControl', 'enum': {0: 'ADU_LEFTTURNLAMPCONTROL_INVALID', 1: 'ADU_LEFTTURNLAMPCONTROL_OFF', 2: 'ADU_LEFTTURNLAMPCONTROL_BLINK', 3: 'ADU_LEFTTURNLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Adubodycontrol0x333333* set_adu_leftturnlampcontrol(Adu_bodycontrol_0x333_333::Adu_leftturnlampcontrolType adu_leftturnlampcontrol);

 private:

  // config detail: {'name': 'ADU_RearfogLampControl', 'enum': {0: 'ADU_REARFOGLAMPCONTROL_INVALID', 1: 'ADU_REARFOGLAMPCONTROL_OFF', 2: 'ADU_REARFOGLAMPCONTROL_ON', 3: 'ADU_REARFOGLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_rearfoglampcontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_rearfoglampcontrolType adu_rearfoglampcontrol);

  // config detail: {'name': 'ADU_FrontfogLampControl', 'enum': {0: 'ADU_FRONTFOGLAMPCONTROL_INVALID', 1: 'ADU_FRONTFOGLAMPCONTROL_OFF', 2: 'ADU_FRONTFOGLAMPCONTROL_ON', 3: 'ADU_FRONTFOGLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_frontfoglampcontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_frontfoglampcontrolType adu_frontfoglampcontrol);

  // config detail: {'name': 'ADU_WasherControl', 'enum': {0: 'ADU_WASHERCONTROL_INVALID', 1: 'ADU_WASHERCONTROL_OFF', 2: 'ADU_WASHERCONTROL_ON', 3: 'ADU_WASHERCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_washercontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_washercontrolType adu_washercontrol);

  // config detail: {'name': 'ADU_WiperControl', 'enum': {0: 'ADU_WIPERCONTROL_INVALID', 1: 'ADU_WIPERCONTROL_OFF', 2: 'ADU_WIPERCONTROL_LOW_SPEED_A3_A8_B5_CD_CB_B5_A3_A9', 3: 'ADU_WIPERCONTROL_HIGH_SPEED_A3_A8_B8_DF_CB_B5_A3_A9', 7: 'ADU_WIPERCONTROL_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_wipercontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_wipercontrolType adu_wipercontrol);

  // config detail: {'name': 'ADU_DoorControl', 'enum': {0: 'ADU_DOORCONTROL_INVALID', 1: 'ADU_DOORCONTROL_LOCK', 2: 'ADU_DOORCONTROL_UNLOCK', 3: 'ADU_DOORCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_doorcontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_doorcontrolType adu_doorcontrol);

  // config detail: {'name': 'ADU_HornControl', 'enum': {0: 'ADU_HORNCONTROL_INVALID', 1: 'ADU_HORNCONTROL_OFF', 2: 'ADU_HORNCONTROL_ON', 3: 'ADU_HORNCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_horncontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_horncontrolType adu_horncontrol);

  // config detail: {'name': 'ADU_HighBeamLampControl', 'enum': {0: 'ADU_HIGHBEAMLAMPCONTROL_INVALID', 1: 'ADU_HIGHBEAMLAMPCONTROL_OFF', 2: 'ADU_HIGHBEAMLAMPCONTROL_ON', 3: 'ADU_HIGHBEAMLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_highbeamlampcontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_highbeamlampcontrolType adu_highbeamlampcontrol);

  // config detail: {'name': 'ADU_LowBeamLampControl', 'enum': {0: 'ADU_LOWBEAMLAMPCONTROL_INVALID', 1: 'ADU_LOWBEAMLAMPCONTROL_OFF', 2: 'ADU_LOWBEAMLAMPCONTROL_ON', 3: 'ADU_LOWBEAMLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_lowbeamlampcontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_lowbeamlampcontrolType adu_lowbeamlampcontrol);

  // config detail: {'name': 'ADU_PositionLampControl', 'enum': {0: 'ADU_POSITIONLAMPCONTROL_INVALID', 1: 'ADU_POSITIONLAMPCONTROL_OFF', 2: 'ADU_POSITIONLAMPCONTROL_ON', 3: 'ADU_POSITIONLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_positionlampcontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_positionlampcontrolType adu_positionlampcontrol);

  // config detail: {'name': 'ADU_HazardLampControl', 'enum': {0: 'ADU_HAZARDLAMPCONTROL_INVALID', 1: 'ADU_HAZARDLAMPCONTROL_OFF', 2: 'ADU_HAZARDLAMPCONTROL_BLINK', 3: 'ADU_HAZARDLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_hazardlampcontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_hazardlampcontrolType adu_hazardlampcontrol);

  // config detail: {'name': 'ADU_RightTurnLampControl', 'enum': {0: 'ADU_RIGHTTURNLAMPCONTROL_INVALID', 1: 'ADU_RIGHTTURNLAMPCONTROL_OFF', 2: 'ADU_RIGHTTURNLAMPCONTROL_BLINK', 3: 'ADU_RIGHTTURNLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_rightturnlampcontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_rightturnlampcontrolType adu_rightturnlampcontrol);

  // config detail: {'name': 'ADU_LeftTurnLampControl', 'enum': {0: 'ADU_LEFTTURNLAMPCONTROL_INVALID', 1: 'ADU_LEFTTURNLAMPCONTROL_OFF', 2: 'ADU_LEFTTURNLAMPCONTROL_BLINK', 3: 'ADU_LEFTTURNLAMPCONTROL_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  void set_p_adu_leftturnlampcontrol(uint8_t* data, Adu_bodycontrol_0x333_333::Adu_leftturnlampcontrolType adu_leftturnlampcontrol);

 private:
  Adu_bodycontrol_0x333_333::Adu_rearfoglampcontrolType adu_rearfoglampcontrol_;
  Adu_bodycontrol_0x333_333::Adu_frontfoglampcontrolType adu_frontfoglampcontrol_;
  Adu_bodycontrol_0x333_333::Adu_washercontrolType adu_washercontrol_;
  Adu_bodycontrol_0x333_333::Adu_wipercontrolType adu_wipercontrol_;
  Adu_bodycontrol_0x333_333::Adu_doorcontrolType adu_doorcontrol_;
  Adu_bodycontrol_0x333_333::Adu_horncontrolType adu_horncontrol_;
  Adu_bodycontrol_0x333_333::Adu_highbeamlampcontrolType adu_highbeamlampcontrol_;
  Adu_bodycontrol_0x333_333::Adu_lowbeamlampcontrolType adu_lowbeamlampcontrol_;
  Adu_bodycontrol_0x333_333::Adu_positionlampcontrolType adu_positionlampcontrol_;
  Adu_bodycontrol_0x333_333::Adu_hazardlampcontrolType adu_hazardlampcontrol_;
  Adu_bodycontrol_0x333_333::Adu_rightturnlampcontrolType adu_rightturnlampcontrol_;
  Adu_bodycontrol_0x333_333::Adu_leftturnlampcontrolType adu_leftturnlampcontrol_;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_ADU_BODYCONTROL_0X333_333_H_
