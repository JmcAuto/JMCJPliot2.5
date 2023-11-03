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

#include "modules/canbus/vehicle/cx75/protocol/gw_swm_body_0x31a_31a.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwswmbody0x31a31a::Gwswmbody0x31a31a() {}
const int32_t Gwswmbody0x31a31a::ID = 0x31A;

void Gwswmbody0x31a31a::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_highbeam(swm_highbeam(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_frontwipingmode(swm_frontwipingmode(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_wipinginterval_sensitvity(swm_wipinginterval_sensitvity(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_rearfoglight(swm_rearfoglight(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_frontfoglight(swm_frontfoglight(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_washerlimphomests(swm_washerlimphomests(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_rearwipingmode(swm_rearwipingmode(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_turnindicationact(swm_turnindicationact(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_vol_plus(swm_vol_plus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_vol_minus(swm_vol_minus(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_wechat(swm_wechat(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_lamplimphomests(swm_lamplimphomests(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_headlamp(swm_headlamp(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_next_song(swm_next_song(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_previous_song(swm_previous_song(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_phone(swm_phone(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_mute(swm_mute(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_menu_right(swm_menu_right(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_menu_left(swm_menu_left(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_menu_down(swm_menu_down(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_menu_up(swm_menu_up(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_fwashersts(swm_fwashersts(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_rolling_counter_0x31a(rolling_counter_0x31a(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_menu_return(swm_menu_return(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_menu_confirm(swm_menu_confirm(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_checksum_0x31a(checksum_0x31a(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_rwashersts(swm_rwashersts(bytes, length));
  chassis->mutable_cx75()->mutable_gw_swm_body_0x31a_31a()->set_swm_flashlightsts(swm_flashlightsts(bytes, length));
}

// config detail: {'name': 'swm_highbeam', 'enum': {0: 'SWM_HIGHBEAM_RELEASED', 1: 'SWM_HIGHBEAM_PRESSED', 2: 'SWM_HIGHBEAM_RESERVED', 3: 'SWM_HIGHBEAM_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_highbeamType Gwswmbody0x31a31a::swm_highbeam(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Gw_swm_body_0x31a_31a::Swm_highbeamType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_highbeamType>(x);
  return ret;
}

// config detail: {'name': 'swm_frontwipingmode', 'enum': {0: 'SWM_FRONTWIPINGMODE_FRONT_WIPING_OFF', 1: 'SWM_FRONTWIPINGMODE_FRONT_WIPER_SPEED_LOW_ACTIVATION', 2: 'SWM_FRONTWIPINGMODE_FRONT_WIPER_SPEED_HIGH_ACTIVATION', 3: 'SWM_FRONTWIPINGMODE_FRONT_MIST_WIPING_ACTIVATION', 4: 'SWM_FRONTWIPINGMODE_FRONT_AUTO_INTERVAL_WIPING_ACTIVATION', 7: 'SWM_FRONTWIPINGMODE_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 12, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_frontwipingmodeType Gwswmbody0x31a31a::swm_frontwipingmode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 3);

  Gw_swm_body_0x31a_31a::Swm_frontwipingmodeType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_frontwipingmodeType>(x);
  return ret;
}

// config detail: {'name': 'swm_wipinginterval_sensitvity', 'enum': {0: 'SWM_WIPINGINTERVAL_SENSITVITY_SENSITIVITY_0', 1: 'SWM_WIPINGINTERVAL_SENSITVITY_SENSITIVITY_1', 2: 'SWM_WIPINGINTERVAL_SENSITVITY_SENSITIVITY_2', 3: 'SWM_WIPINGINTERVAL_SENSITVITY_SENSITIVITY_3', 7: 'SWM_WIPINGINTERVAL_SENSITVITY_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_wipinginterval_sensitvityType Gwswmbody0x31a31a::swm_wipinginterval_sensitvity(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 3);

  Gw_swm_body_0x31a_31a::Swm_wipinginterval_sensitvityType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_wipinginterval_sensitvityType>(x);
  return ret;
}

// config detail: {'name': 'swm_rearfoglight', 'enum': {0: 'SWM_REARFOGLIGHT_RELEASED', 1: 'SWM_REARFOGLIGHT_PRESSED', 2: 'SWM_REARFOGLIGHT_RESERVED', 3: 'SWM_REARFOGLIGHT_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_rearfoglightType Gwswmbody0x31a31a::swm_rearfoglight(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 2);

  Gw_swm_body_0x31a_31a::Swm_rearfoglightType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_rearfoglightType>(x);
  return ret;
}

// config detail: {'name': 'swm_frontfoglight', 'enum': {0: 'SWM_FRONTFOGLIGHT_OFF', 1: 'SWM_FRONTFOGLIGHT_ON', 2: 'SWM_FRONTFOGLIGHT_RESERVED', 3: 'SWM_FRONTFOGLIGHT_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 19, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_frontfoglightType Gwswmbody0x31a31a::swm_frontfoglight(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 2);

  Gw_swm_body_0x31a_31a::Swm_frontfoglightType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_frontfoglightType>(x);
  return ret;
}

// config detail: {'name': 'swm_washerlimphomests', 'enum': {0: 'SWM_WASHERLIMPHOMESTS_NORMAL', 1: 'SWM_WASHERLIMPHOMESTS_LIMPHOME'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 2, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_washerlimphomestsType Gwswmbody0x31a31a::swm_washerlimphomests(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  Gw_swm_body_0x31a_31a::Swm_washerlimphomestsType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_washerlimphomestsType>(x);
  return ret;
}

// config detail: {'name': 'swm_rearwipingmode', 'enum': {0: 'SWM_REARWIPINGMODE_REARWIPING_OFF', 1: 'SWM_REARWIPINGMODE_REAR_WIPER_UNIFORMITY_SPEED', 2: 'SWM_REARWIPINGMODE_RESERVED', 3: 'SWM_REARWIPINGMODE_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_rearwipingmodeType Gwswmbody0x31a31a::swm_rearwipingmode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 2);

  Gw_swm_body_0x31a_31a::Swm_rearwipingmodeType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_rearwipingmodeType>(x);
  return ret;
}

// config detail: {'name': 'swm_turnindicationact', 'enum': {0: 'SWM_TURNINDICATIONACT_DEFAULT', 1: 'SWM_TURNINDICATIONACT_TURN_LEFT', 2: 'SWM_TURNINDICATIONACT_TURN_RIGHT', 3: 'SWM_TURNINDICATIONACT_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_turnindicationactType Gwswmbody0x31a31a::swm_turnindicationact(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 2);

  Gw_swm_body_0x31a_31a::Swm_turnindicationactType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_turnindicationactType>(x);
  return ret;
}

// config detail: {'name': 'swm_vol_plus', 'enum': {0: 'SWM_VOL_PLUS_RELEASED', 1: 'SWM_VOL_PLUS_PRESSED', 2: 'SWM_VOL_PLUS_RESERVED', 3: 'SWM_VOL_PLUS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 25, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_vol_plusType Gwswmbody0x31a31a::swm_vol_plus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 2);

  Gw_swm_body_0x31a_31a::Swm_vol_plusType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_vol_plusType>(x);
  return ret;
}

// config detail: {'name': 'swm_vol_minus', 'enum': {0: 'SWM_VOL_MINUS_RELEASED', 1: 'SWM_VOL_MINUS_PRESSED', 2: 'SWM_VOL_MINUS_RESERVED', 3: 'SWM_VOL_MINUS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 27, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_vol_minusType Gwswmbody0x31a31a::swm_vol_minus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 2);

  Gw_swm_body_0x31a_31a::Swm_vol_minusType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_vol_minusType>(x);
  return ret;
}

// config detail: {'name': 'swm_wechat', 'enum': {0: 'SWM_WECHAT_RELEASED', 1: 'SWM_WECHAT_PRESSED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_wechatType Gwswmbody0x31a31a::swm_wechat(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(4, 1);

  Gw_swm_body_0x31a_31a::Swm_wechatType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_wechatType>(x);
  return ret;
}

// config detail: {'name': 'swm_lamplimphomests', 'enum': {0: 'SWM_LAMPLIMPHOMESTS_NORMAL', 1: 'SWM_LAMPLIMPHOMESTS_LIMPHOME'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_lamplimphomestsType Gwswmbody0x31a31a::swm_lamplimphomests(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  Gw_swm_body_0x31a_31a::Swm_lamplimphomestsType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_lamplimphomestsType>(x);
  return ret;
}

// config detail: {'name': 'swm_headlamp', 'enum': {0: 'SWM_HEADLAMP_OFF', 1: 'SWM_HEADLAMP_AUTOLIGHT', 2: 'SWM_HEADLAMP_POSITIONLIGHT', 3: 'SWM_HEADLAMP_LOWBEAM', 4: 'SWM_HEADLAMP_INVALID'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_headlampType Gwswmbody0x31a31a::swm_headlamp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 3);

  Gw_swm_body_0x31a_31a::Swm_headlampType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_headlampType>(x);
  return ret;
}

// config detail: {'name': 'swm_next_song', 'enum': {0: 'SWM_NEXT_SONG_RELEASED', 1: 'SWM_NEXT_SONG_PRESSED', 2: 'SWM_NEXT_SONG_RESERVED', 3: 'SWM_NEXT_SONG_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_next_songType Gwswmbody0x31a31a::swm_next_song(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Gw_swm_body_0x31a_31a::Swm_next_songType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_next_songType>(x);
  return ret;
}

// config detail: {'name': 'swm_previous_song', 'enum': {0: 'SWM_PREVIOUS_SONG_RELEASED', 1: 'SWM_PREVIOUS_SONG_PRESSED', 2: 'SWM_PREVIOUS_SONG_RESERVED', 3: 'SWM_PREVIOUS_SONG_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_previous_songType Gwswmbody0x31a31a::swm_previous_song(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 2);

  Gw_swm_body_0x31a_31a::Swm_previous_songType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_previous_songType>(x);
  return ret;
}

// config detail: {'name': 'swm_phone', 'enum': {0: 'SWM_PHONE_RELEASED', 1: 'SWM_PHONE_PRESSED', 2: 'SWM_PHONE_RESERVED', 3: 'SWM_PHONE_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_phoneType Gwswmbody0x31a31a::swm_phone(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 2);

  Gw_swm_body_0x31a_31a::Swm_phoneType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_phoneType>(x);
  return ret;
}

// config detail: {'name': 'swm_mute', 'enum': {0: 'SWM_MUTE_RELEASED', 1: 'SWM_MUTE_PRESSED', 2: 'SWM_MUTE_RESERVED', 3: 'SWM_MUTE_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_muteType Gwswmbody0x31a31a::swm_mute(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 2);

  Gw_swm_body_0x31a_31a::Swm_muteType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_muteType>(x);
  return ret;
}

// config detail: {'name': 'swm_menu_right', 'enum': {0: 'SWM_MENU_RIGHT_RELEASED', 1: 'SWM_MENU_RIGHT_PRESSED', 2: 'SWM_MENU_RIGHT_RESERVED', 3: 'SWM_MENU_RIGHT_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_menu_rightType Gwswmbody0x31a31a::swm_menu_right(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 2);

  Gw_swm_body_0x31a_31a::Swm_menu_rightType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_menu_rightType>(x);
  return ret;
}

// config detail: {'name': 'swm_menu_left', 'enum': {0: 'SWM_MENU_LEFT_RELEASED', 1: 'SWM_MENU_LEFT_PRESSED', 2: 'SWM_MENU_LEFT_RESERVED', 3: 'SWM_MENU_LEFT_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 43, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_menu_leftType Gwswmbody0x31a31a::swm_menu_left(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 2);

  Gw_swm_body_0x31a_31a::Swm_menu_leftType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_menu_leftType>(x);
  return ret;
}

// config detail: {'name': 'swm_menu_down', 'enum': {0: 'SWM_MENU_DOWN_RELEASED', 1: 'SWM_MENU_DOWN_PRESSED', 2: 'SWM_MENU_DOWN_RESERVED', 3: 'SWM_MENU_DOWN_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_menu_downType Gwswmbody0x31a31a::swm_menu_down(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 2);

  Gw_swm_body_0x31a_31a::Swm_menu_downType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_menu_downType>(x);
  return ret;
}

// config detail: {'name': 'swm_menu_up', 'enum': {0: 'SWM_MENU_UP_RELEASED', 1: 'SWM_MENU_UP_PRESSED', 2: 'SWM_MENU_UP_RESERVED', 3: 'SWM_MENU_UP_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_menu_upType Gwswmbody0x31a31a::swm_menu_up(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 2);

  Gw_swm_body_0x31a_31a::Swm_menu_upType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_menu_upType>(x);
  return ret;
}

// config detail: {'name': 'swm_fwashersts', 'enum': {0: 'SWM_FWASHERSTS_RELEASED', 1: 'SWM_FWASHERSTS_PRESSED', 2: 'SWM_FWASHERSTS_RESERVED', 3: 'SWM_FWASHERSTS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_fwasherstsType Gwswmbody0x31a31a::swm_fwashersts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 2);

  Gw_swm_body_0x31a_31a::Swm_fwasherstsType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_fwasherstsType>(x);
  return ret;
}

// config detail: {'name': 'rolling_counter_0x31a', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwswmbody0x31a31a::rolling_counter_0x31a(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'swm_menu_return', 'enum': {0: 'SWM_MENU_RETURN_RELEASED', 1: 'SWM_MENU_RETURN_PRESSED', 2: 'SWM_MENU_RETURN_RESERVED', 3: 'SWM_MENU_RETURN_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_menu_returnType Gwswmbody0x31a31a::swm_menu_return(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 2);

  Gw_swm_body_0x31a_31a::Swm_menu_returnType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_menu_returnType>(x);
  return ret;
}

// config detail: {'name': 'swm_menu_confirm', 'enum': {0: 'SWM_MENU_CONFIRM_RELEASED', 1: 'SWM_MENU_CONFIRM_PRESSED', 2: 'SWM_MENU_CONFIRM_RESERVED', 3: 'SWM_MENU_CONFIRM_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_menu_confirmType Gwswmbody0x31a31a::swm_menu_confirm(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 2);

  Gw_swm_body_0x31a_31a::Swm_menu_confirmType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_menu_confirmType>(x);
  return ret;
}

// config detail: {'name': 'checksum_0x31a', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwswmbody0x31a31a::checksum_0x31a(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'swm_rwashersts', 'enum': {0: 'SWM_RWASHERSTS_RELEASED', 1: 'SWM_RWASHERSTS_PRESSED', 2: 'SWM_RWASHERSTS_RESERVED', 3: 'SWM_RWASHERSTS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_rwasherstsType Gwswmbody0x31a31a::swm_rwashersts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Gw_swm_body_0x31a_31a::Swm_rwasherstsType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_rwasherstsType>(x);
  return ret;
}

// config detail: {'name': 'swm_flashlightsts', 'enum': {0: 'SWM_FLASHLIGHTSTS_RELEASED', 1: 'SWM_FLASHLIGHTSTS_PRESSED', 2: 'SWM_FLASHLIGHTSTS_RESERVED', 3: 'SWM_FLASHLIGHTSTS_INVALID'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_swm_body_0x31a_31a::Swm_flashlightstsType Gwswmbody0x31a31a::swm_flashlightsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Gw_swm_body_0x31a_31a::Swm_flashlightstsType ret =  static_cast<Gw_swm_body_0x31a_31a::Swm_flashlightstsType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
