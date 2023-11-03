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

#include "modules/canbus/vehicle/teshun/protocol/bcm_door_0x310_310.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Bcmdoor0x310310::Bcmdoor0x310310() {}
const int32_t Bcmdoor0x310310::ID = 0x310;

void Bcmdoor0x310310::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_checksum_0x310(bcm_checksum_0x310(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_rolling_counter_0x310(bcm_rolling_counter_0x310(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_rearwipersts(bcm_rearwipersts(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_frontwipersts(bcm_frontwipersts(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_chargeplugsts(bcm_chargeplugsts(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_enginehoodsts(bcm_enginehoodsts(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_trunkdoorpreunlocksts(bcm_trunkdoorpreunlocksts(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_antitheftsts(bcm_antitheftsts(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_lockunlockfeedback(bcm_lockunlockfeedback(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_doorlockststrunk(bcm_doorlockststrunk(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_doorststrunk(bcm_doorststrunk(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_doorlockstsrearri(bcm_doorlockstsrearri(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_doorlockstsrearle(bcm_doorlockstsrearle(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_doorlockstsfrntri(bcm_doorlockstsfrntri(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_doorlockstsfrntle(bcm_doorlockstsfrntle(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_doorstsrearri(bcm_doorstsrearri(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_doorstsrearle(bcm_doorstsrearle(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_doorstsfrntri(bcm_doorstsfrntri(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_door_0x310_310()->set_bcm_doorstsfrntle(bcm_doorstsfrntle(bytes, length));
}

// config detail: {'name': 'bcm_checksum_0x310', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Bcmdoor0x310310::bcm_checksum_0x310(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'bcm_rolling_counter_0x310', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Bcmdoor0x310310::bcm_rolling_counter_0x310(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'bcm_rearwipersts', 'enum': {0: 'BCM_REARWIPERSTS_NO_USED', 1: 'BCM_REARWIPERSTS_OFF', 2: 'BCM_REARWIPERSTS_ON', 3: 'BCM_REARWIPERSTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_rearwiperstsType Bcmdoor0x310310::bcm_rearwipersts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 2);

  Bcm_door_0x310_310::Bcm_rearwiperstsType ret =  static_cast<Bcm_door_0x310_310::Bcm_rearwiperstsType>(x);
  return ret;
}

// config detail: {'name': 'bcm_frontwipersts', 'enum': {0: 'BCM_FRONTWIPERSTS_NO_USED', 1: 'BCM_FRONTWIPERSTS_OFF', 2: 'BCM_FRONTWIPERSTS_LOW', 3: 'BCM_FRONTWIPERSTS_HIGH'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_frontwiperstsType Bcmdoor0x310310::bcm_frontwipersts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  Bcm_door_0x310_310::Bcm_frontwiperstsType ret =  static_cast<Bcm_door_0x310_310::Bcm_frontwiperstsType>(x);
  return ret;
}

// config detail: {'name': 'bcm_chargeplugsts', 'enum': {0: 'BCM_CHARGEPLUGSTS_NO_USED', 1: 'BCM_CHARGEPLUGSTS_CLOSED', 2: 'BCM_CHARGEPLUGSTS_OPEN', 3: 'BCM_CHARGEPLUGSTS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_chargeplugstsType Bcmdoor0x310310::bcm_chargeplugsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 2);

  Bcm_door_0x310_310::Bcm_chargeplugstsType ret =  static_cast<Bcm_door_0x310_310::Bcm_chargeplugstsType>(x);
  return ret;
}

// config detail: {'name': 'bcm_enginehoodsts', 'enum': {0: 'BCM_ENGINEHOODSTS_NO_USED', 1: 'BCM_ENGINEHOODSTS_CLOSED', 2: 'BCM_ENGINEHOODSTS_OPEN', 3: 'BCM_ENGINEHOODSTS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_enginehoodstsType Bcmdoor0x310310::bcm_enginehoodsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 2);

  Bcm_door_0x310_310::Bcm_enginehoodstsType ret =  static_cast<Bcm_door_0x310_310::Bcm_enginehoodstsType>(x);
  return ret;
}

// config detail: {'name': 'bcm_trunkdoorpreunlocksts', 'enum': {0: 'BCM_TRUNKDOORPREUNLOCKSTS_NO_USED_NO_ACTION', 1: 'BCM_TRUNKDOORPREUNLOCKSTS_PREUNLOCKED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_trunkdoorpreunlockstsType Bcmdoor0x310310::bcm_trunkdoorpreunlocksts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 1);

  Bcm_door_0x310_310::Bcm_trunkdoorpreunlockstsType ret =  static_cast<Bcm_door_0x310_310::Bcm_trunkdoorpreunlockstsType>(x);
  return ret;
}

// config detail: {'name': 'bcm_antitheftsts', 'enum': {0: 'BCM_ANTITHEFTSTS_RELIEVE_ANTI_THEFT', 1: 'BCM_ANTITHEFTSTS_ANTI_THEFT', 2: 'BCM_ANTITHEFTSTS_ANTI_THEFT_PASS', 3: 'BCM_ANTITHEFTSTS_ANTI_THEFT_FAULT_ALARM'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_antitheftstsType Bcmdoor0x310310::bcm_antitheftsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 2);

  Bcm_door_0x310_310::Bcm_antitheftstsType ret =  static_cast<Bcm_door_0x310_310::Bcm_antitheftstsType>(x);
  return ret;
}

// config detail: {'name': 'bcm_lockunlockfeedback', 'enum': {0: 'BCM_LOCKUNLOCKFEEDBACK_NO_LOCK_UNLOCK_ACTION', 1: 'BCM_LOCKUNLOCKFEEDBACK_PE_LOCK', 2: 'BCM_LOCKUNLOCKFEEDBACK_RKE_LOCK', 3: 'BCM_LOCKUNLOCKFEEDBACK_LEAVE_VEHICLE_LOCK', 4: 'BCM_LOCKUNLOCKFEEDBACK_MASTER_LOCK_D6_D8_CB_F8', 5: 'BCM_LOCKUNLOCKFEEDBACK_FINGER_LOCK_BB_FA_B3_D7_C9_CF_CB_F8', 6: 'BCM_LOCKUNLOCKFEEDBACK_RKE_30S_AUTOLOCK_BF_D8_D7_AF_BB_C0', 7: 'BCM_LOCKUNLOCKFEEDBACK_PE_30S_AUTOLOCK', 8: 'BCM_LOCKUNLOCKFEEDBACK_NEAR_TO_VEHICLE_30S_AUTOLOCK', 15: 'BCM_LOCKUNLOCKFEEDBACK_ANY_OTHER_LOCK', 17: 'BCM_LOCKUNLOCKFEEDBACK_PE_UNLOCK', 18: 'BCM_LOCKUNLOCKFEEDBACK_RKE_UNLOCK', 19: 'BCM_LOCKUNLOCKFEEDBACK_NEAR_TO_VEHICLE_UNLOCK', 20: 'BCM_LOCKUNLOCKFEEDBACK_MASTER_UNLOCK_D6_E2_CB_F8', 21: 'BCM_LOCKUNLOCKFEEDBACK_FINGER_UNLOCK_BB_FA_B3_E2_CB_F8', 22: 'BCM_LOCKUNLOCKFEEDBACK_CRASHUNLOCK_C5_F6_BD_E2_CB_F8', 23: 'BCM_LOCKUNLOCKFEEDBACK_KEYREMOVALUNLOCK_B0_CE_B3_E2_CB_F8', 24: 'BCM_LOCKUNLOCKFEEDBACK_ENGOFFUNLOCK_BB_F0_BD_E2_CB_F8', 31: 'BCM_LOCKUNLOCKFEEDBACK_ANY_OTHER_UNLOCK'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_lockunlockfeedbackType Bcmdoor0x310310::bcm_lockunlockfeedback(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(3, 5);

  Bcm_door_0x310_310::Bcm_lockunlockfeedbackType ret =  static_cast<Bcm_door_0x310_310::Bcm_lockunlockfeedbackType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorlockststrunk', 'enum': {0: 'BCM_DOORLOCKSTSTRUNK_NO_USED', 1: 'BCM_DOORLOCKSTSTRUNK_LOCKED', 2: 'BCM_DOORLOCKSTSTRUNK_UNLOCKED', 3: 'BCM_DOORLOCKSTSTRUNK_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_doorlockststrunkType Bcmdoor0x310310::bcm_doorlockststrunk(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(1, 2);

  Bcm_door_0x310_310::Bcm_doorlockststrunkType ret =  static_cast<Bcm_door_0x310_310::Bcm_doorlockststrunkType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorststrunk', 'enum': {0: 'BCM_DOORSTSTRUNK_NO_USED', 1: 'BCM_DOORSTSTRUNK_CLOSED', 2: 'BCM_DOORSTSTRUNK_OPEN', 3: 'BCM_DOORSTSTRUNK_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_doorststrunkType Bcmdoor0x310310::bcm_doorststrunk(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 2);

  Bcm_door_0x310_310::Bcm_doorststrunkType ret =  static_cast<Bcm_door_0x310_310::Bcm_doorststrunkType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorlockstsrearri', 'enum': {0: 'BCM_DOORLOCKSTSREARRI_NO_USED', 1: 'BCM_DOORLOCKSTSREARRI_LOCKED', 2: 'BCM_DOORLOCKSTSREARRI_UNLOCKED', 3: 'BCM_DOORLOCKSTSREARRI_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_doorlockstsrearriType Bcmdoor0x310310::bcm_doorlockstsrearri(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Bcm_door_0x310_310::Bcm_doorlockstsrearriType ret =  static_cast<Bcm_door_0x310_310::Bcm_doorlockstsrearriType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorlockstsrearle', 'enum': {0: 'BCM_DOORLOCKSTSREARLE_NO_USED', 1: 'BCM_DOORLOCKSTSREARLE_LOCKED', 2: 'BCM_DOORLOCKSTSREARLE_UNLOCKED', 3: 'BCM_DOORLOCKSTSREARLE_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_doorlockstsrearleType Bcmdoor0x310310::bcm_doorlockstsrearle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  Bcm_door_0x310_310::Bcm_doorlockstsrearleType ret =  static_cast<Bcm_door_0x310_310::Bcm_doorlockstsrearleType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorlockstsfrntri', 'enum': {0: 'BCM_DOORLOCKSTSFRNTRI_NO_USED', 1: 'BCM_DOORLOCKSTSFRNTRI_LOCKED', 2: 'BCM_DOORLOCKSTSFRNTRI_UNLOCKED', 3: 'BCM_DOORLOCKSTSFRNTRI_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_doorlockstsfrntriType Bcmdoor0x310310::bcm_doorlockstsfrntri(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 2);

  Bcm_door_0x310_310::Bcm_doorlockstsfrntriType ret =  static_cast<Bcm_door_0x310_310::Bcm_doorlockstsfrntriType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorlockstsfrntle', 'enum': {0: 'BCM_DOORLOCKSTSFRNTLE_NO_USED', 1: 'BCM_DOORLOCKSTSFRNTLE_LOCKED', 2: 'BCM_DOORLOCKSTSFRNTLE_UNLOCKED', 3: 'BCM_DOORLOCKSTSFRNTLE_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_doorlockstsfrntleType Bcmdoor0x310310::bcm_doorlockstsfrntle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 2);

  Bcm_door_0x310_310::Bcm_doorlockstsfrntleType ret =  static_cast<Bcm_door_0x310_310::Bcm_doorlockstsfrntleType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorstsrearri', 'enum': {0: 'BCM_DOORSTSREARRI_NO_USED', 1: 'BCM_DOORSTSREARRI_CLOSED', 2: 'BCM_DOORSTSREARRI_OPEN', 3: 'BCM_DOORSTSREARRI_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_doorstsrearriType Bcmdoor0x310310::bcm_doorstsrearri(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 2);

  Bcm_door_0x310_310::Bcm_doorstsrearriType ret =  static_cast<Bcm_door_0x310_310::Bcm_doorstsrearriType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorstsrearle', 'enum': {0: 'BCM_DOORSTSREARLE_NO_USED', 1: 'BCM_DOORSTSREARLE_CLOSED', 2: 'BCM_DOORSTSREARLE_OPEN', 3: 'BCM_DOORSTSREARLE_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_doorstsrearleType Bcmdoor0x310310::bcm_doorstsrearle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 2);

  Bcm_door_0x310_310::Bcm_doorstsrearleType ret =  static_cast<Bcm_door_0x310_310::Bcm_doorstsrearleType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorstsfrntri', 'enum': {0: 'BCM_DOORSTSFRNTRI_NO_USED', 1: 'BCM_DOORSTSFRNTRI_CLOSED', 2: 'BCM_DOORSTSFRNTRI_OPEN', 3: 'BCM_DOORSTSFRNTRI_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_doorstsfrntriType Bcmdoor0x310310::bcm_doorstsfrntri(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 2);

  Bcm_door_0x310_310::Bcm_doorstsfrntriType ret =  static_cast<Bcm_door_0x310_310::Bcm_doorstsfrntriType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorstsfrntle', 'enum': {0: 'BCM_DOORSTSFRNTLE_NO_USED', 1: 'BCM_DOORSTSFRNTLE_CLOSED', 2: 'BCM_DOORSTSFRNTLE_OPEN', 3: 'BCM_DOORSTSFRNTLE_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_door_0x310_310::Bcm_doorstsfrntleType Bcmdoor0x310310::bcm_doorstsfrntle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Bcm_door_0x310_310::Bcm_doorstsfrntleType ret =  static_cast<Bcm_door_0x310_310::Bcm_doorstsfrntleType>(x);
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
