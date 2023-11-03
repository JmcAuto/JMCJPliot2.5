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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_SWM_BODY_0X31A_31A_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_SWM_BODY_0X31A_31A_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwswmbody0x31a31a : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwswmbody0x31a31a();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'SWM_HighBeam', 'enum': {0: 'SWM_HIGHBEAM_RELEASED', 1: 'SWM_HIGHBEAM_PRESSED', 2: 'SWM_HIGHBEAM_RESERVED', 3: 'SWM_HIGHBEAM_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_highbeamType swm_highbeam(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_FrontWipingMode', 'enum': {0: 'SWM_FRONTWIPINGMODE_FRONT_WIPING_OFF', 1: 'SWM_FRONTWIPINGMODE_FRONT_WIPER_SPEED_LOW_ACTIVATION', 2: 'SWM_FRONTWIPINGMODE_FRONT_WIPER_SPEED_HIGH_ACTIVATION', 3: 'SWM_FRONTWIPINGMODE_FRONT_MIST_WIPING_ACTIVATION', 4: 'SWM_FRONTWIPINGMODE_FRONT_AUTO_INTERVAL_WIPING_ACTIVATION', 7: 'SWM_FRONTWIPINGMODE_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_frontwipingmodeType swm_frontwipingmode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_WipingInterval_Sensitvity', 'enum': {0: 'SWM_WIPINGINTERVAL_SENSITVITY_SENSITIVITY_0', 1: 'SWM_WIPINGINTERVAL_SENSITVITY_SENSITIVITY_1', 2: 'SWM_WIPINGINTERVAL_SENSITVITY_SENSITIVITY_2', 3: 'SWM_WIPINGINTERVAL_SENSITVITY_SENSITIVITY_3', 7: 'SWM_WIPINGINTERVAL_SENSITVITY_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_wipinginterval_sensitvityType swm_wipinginterval_sensitvity(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_RearFogLight', 'enum': {0: 'SWM_REARFOGLIGHT_RELEASED', 1: 'SWM_REARFOGLIGHT_PRESSED', 2: 'SWM_REARFOGLIGHT_RESERVED', 3: 'SWM_REARFOGLIGHT_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_rearfoglightType swm_rearfoglight(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_FrontFogLight', 'enum': {0: 'SWM_FRONTFOGLIGHT_OFF', 1: 'SWM_FRONTFOGLIGHT_ON', 2: 'SWM_FRONTFOGLIGHT_RESERVED', 3: 'SWM_FRONTFOGLIGHT_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_frontfoglightType swm_frontfoglight(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_WasherLimpHomeSts', 'enum': {0: 'SWM_WASHERLIMPHOMESTS_NORMAL', 1: 'SWM_WASHERLIMPHOMESTS_LIMPHOME'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_washerlimphomestsType swm_washerlimphomests(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_RearWipingMode', 'enum': {0: 'SWM_REARWIPINGMODE_REARWIPING_OFF', 1: 'SWM_REARWIPINGMODE_REAR_WIPER_UNIFORMITY_SPEED', 2: 'SWM_REARWIPINGMODE_RESERVED', 3: 'SWM_REARWIPINGMODE_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_rearwipingmodeType swm_rearwipingmode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_TurnIndicationAct', 'enum': {0: 'SWM_TURNINDICATIONACT_DEFAULT', 1: 'SWM_TURNINDICATIONACT_TURN_LEFT', 2: 'SWM_TURNINDICATIONACT_TURN_RIGHT', 3: 'SWM_TURNINDICATIONACT_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_turnindicationactType swm_turnindicationact(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_VOL_Plus', 'enum': {0: 'SWM_VOL_PLUS_RELEASED', 1: 'SWM_VOL_PLUS_PRESSED', 2: 'SWM_VOL_PLUS_RESERVED', 3: 'SWM_VOL_PLUS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_vol_plusType swm_vol_plus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_VOL_Minus', 'enum': {0: 'SWM_VOL_MINUS_RELEASED', 1: 'SWM_VOL_MINUS_PRESSED', 2: 'SWM_VOL_MINUS_RESERVED', 3: 'SWM_VOL_MINUS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_vol_minusType swm_vol_minus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Wechat', 'enum': {0: 'SWM_WECHAT_RELEASED', 1: 'SWM_WECHAT_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_wechatType swm_wechat(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_LampLimpHomeSts', 'enum': {0: 'SWM_LAMPLIMPHOMESTS_NORMAL', 1: 'SWM_LAMPLIMPHOMESTS_LIMPHOME'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_lamplimphomestsType swm_lamplimphomests(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_HeadLamp', 'enum': {0: 'SWM_HEADLAMP_OFF', 1: 'SWM_HEADLAMP_AUTOLIGHT', 2: 'SWM_HEADLAMP_POSITIONLIGHT', 3: 'SWM_HEADLAMP_LOWBEAM', 4: 'SWM_HEADLAMP_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_headlampType swm_headlamp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Next_Song', 'enum': {0: 'SWM_NEXT_SONG_RELEASED', 1: 'SWM_NEXT_SONG_PRESSED', 2: 'SWM_NEXT_SONG_RESERVED', 3: 'SWM_NEXT_SONG_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_next_songType swm_next_song(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Previous_Song', 'enum': {0: 'SWM_PREVIOUS_SONG_RELEASED', 1: 'SWM_PREVIOUS_SONG_PRESSED', 2: 'SWM_PREVIOUS_SONG_RESERVED', 3: 'SWM_PREVIOUS_SONG_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_previous_songType swm_previous_song(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Phone', 'enum': {0: 'SWM_PHONE_RELEASED', 1: 'SWM_PHONE_PRESSED', 2: 'SWM_PHONE_RESERVED', 3: 'SWM_PHONE_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_phoneType swm_phone(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Mute', 'enum': {0: 'SWM_MUTE_RELEASED', 1: 'SWM_MUTE_PRESSED', 2: 'SWM_MUTE_RESERVED', 3: 'SWM_MUTE_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_muteType swm_mute(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Menu_Right', 'enum': {0: 'SWM_MENU_RIGHT_RELEASED', 1: 'SWM_MENU_RIGHT_PRESSED', 2: 'SWM_MENU_RIGHT_RESERVED', 3: 'SWM_MENU_RIGHT_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_menu_rightType swm_menu_right(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Menu_Left', 'enum': {0: 'SWM_MENU_LEFT_RELEASED', 1: 'SWM_MENU_LEFT_PRESSED', 2: 'SWM_MENU_LEFT_RESERVED', 3: 'SWM_MENU_LEFT_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 43, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_menu_leftType swm_menu_left(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Menu_Down', 'enum': {0: 'SWM_MENU_DOWN_RELEASED', 1: 'SWM_MENU_DOWN_PRESSED', 2: 'SWM_MENU_DOWN_RESERVED', 3: 'SWM_MENU_DOWN_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_menu_downType swm_menu_down(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Menu_Up', 'enum': {0: 'SWM_MENU_UP_RELEASED', 1: 'SWM_MENU_UP_PRESSED', 2: 'SWM_MENU_UP_RESERVED', 3: 'SWM_MENU_UP_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_menu_upType swm_menu_up(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_FWasherSts', 'enum': {0: 'SWM_FWASHERSTS_RELEASED', 1: 'SWM_FWASHERSTS_PRESSED', 2: 'SWM_FWASHERSTS_RESERVED', 3: 'SWM_FWASHERSTS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_fwasherstsType swm_fwashersts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x31A', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x31a(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Menu_Return', 'enum': {0: 'SWM_MENU_RETURN_RELEASED', 1: 'SWM_MENU_RETURN_PRESSED', 2: 'SWM_MENU_RETURN_RESERVED', 3: 'SWM_MENU_RETURN_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_menu_returnType swm_menu_return(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_Menu_Confirm', 'enum': {0: 'SWM_MENU_CONFIRM_RELEASED', 1: 'SWM_MENU_CONFIRM_PRESSED', 2: 'SWM_MENU_CONFIRM_RESERVED', 3: 'SWM_MENU_CONFIRM_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_menu_confirmType swm_menu_confirm(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x31A', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x31a(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_RWasherSts', 'enum': {0: 'SWM_RWASHERSTS_RELEASED', 1: 'SWM_RWASHERSTS_PRESSED', 2: 'SWM_RWASHERSTS_RESERVED', 3: 'SWM_RWASHERSTS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_rwasherstsType swm_rwashersts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'SWM_FlashLightSts', 'enum': {0: 'SWM_FLASHLIGHTSTS_RELEASED', 1: 'SWM_FLASHLIGHTSTS_PRESSED', 2: 'SWM_FLASHLIGHTSTS_RESERVED', 3: 'SWM_FLASHLIGHTSTS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_swm_body_0x31a_31a::Swm_flashlightstsType swm_flashlightsts(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_SWM_BODY_0X31A_31A_H_
