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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_BCM_DOOR_0X310_310_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_BCM_DOOR_0X310_310_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Bcmdoor0x310310 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Bcmdoor0x310310();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'BCM_Checksum_0x310', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int bcm_checksum_0x310(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_Rolling_counter_0x310', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int bcm_rolling_counter_0x310(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_RearWiperSts', 'enum': {0: 'BCM_REARWIPERSTS_NO_USED', 1: 'BCM_REARWIPERSTS_OFF', 2: 'BCM_REARWIPERSTS_ON', 3: 'BCM_REARWIPERSTS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_rearwiperstsType bcm_rearwipersts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_FrontWiperSts', 'enum': {0: 'BCM_FRONTWIPERSTS_NO_USED', 1: 'BCM_FRONTWIPERSTS_OFF', 2: 'BCM_FRONTWIPERSTS_LOW', 3: 'BCM_FRONTWIPERSTS_HIGH'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 33, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_frontwiperstsType bcm_frontwipersts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_ChargePlugSts', 'enum': {0: 'BCM_CHARGEPLUGSTS_NO_USED', 1: 'BCM_CHARGEPLUGSTS_CLOSED', 2: 'BCM_CHARGEPLUGSTS_OPEN', 3: 'BCM_CHARGEPLUGSTS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_chargeplugstsType bcm_chargeplugsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_EnginehoodSts', 'enum': {0: 'BCM_ENGINEHOODSTS_NO_USED', 1: 'BCM_ENGINEHOODSTS_CLOSED', 2: 'BCM_ENGINEHOODSTS_OPEN', 3: 'BCM_ENGINEHOODSTS_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_enginehoodstsType bcm_enginehoodsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_TrunkDoorPreUnlockSts', 'enum': {0: 'BCM_TRUNKDOORPREUNLOCKSTS_NO_USED_NO_ACTION', 1: 'BCM_TRUNKDOORPREUNLOCKSTS_PREUNLOCKED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 29, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_trunkdoorpreunlockstsType bcm_trunkdoorpreunlocksts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_AntiTheftSts', 'enum': {0: 'BCM_ANTITHEFTSTS_RELIEVE_ANTI_THEFT', 1: 'BCM_ANTITHEFTSTS_ANTI_THEFT', 2: 'BCM_ANTITHEFTSTS_ANTI_THEFT_PASS', 3: 'BCM_ANTITHEFTSTS_ANTI_THEFT_FAULT_ALARM'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 17, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_antitheftstsType bcm_antitheftsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_LockUnlockFeedback', 'enum': {0: 'BCM_LOCKUNLOCKFEEDBACK_NO_LOCK_UNLOCK_ACTION', 1: 'BCM_LOCKUNLOCKFEEDBACK_PE_LOCK', 2: 'BCM_LOCKUNLOCKFEEDBACK_RKE_LOCK', 3: 'BCM_LOCKUNLOCKFEEDBACK_LEAVE_VEHICLE_LOCK', 4: 'BCM_LOCKUNLOCKFEEDBACK_MASTER_LOCK_D6_D8_CB_F8', 5: 'BCM_LOCKUNLOCKFEEDBACK_FINGER_LOCK_BB_FA_B3_D7_C9_CF_CB_F8', 6: 'BCM_LOCKUNLOCKFEEDBACK_RKE_30S_AUTOLOCK_BF_D8_D7_AF_BB_C0', 7: 'BCM_LOCKUNLOCKFEEDBACK_PE_30S_AUTOLOCK', 8: 'BCM_LOCKUNLOCKFEEDBACK_NEAR_TO_VEHICLE_30S_AUTOLOCK', 15: 'BCM_LOCKUNLOCKFEEDBACK_ANY_OTHER_LOCK', 17: 'BCM_LOCKUNLOCKFEEDBACK_PE_UNLOCK', 18: 'BCM_LOCKUNLOCKFEEDBACK_RKE_UNLOCK', 19: 'BCM_LOCKUNLOCKFEEDBACK_NEAR_TO_VEHICLE_UNLOCK', 20: 'BCM_LOCKUNLOCKFEEDBACK_MASTER_UNLOCK_D6_E2_CB_F8', 21: 'BCM_LOCKUNLOCKFEEDBACK_FINGER_UNLOCK_BB_FA_B3_E2_CB_F8', 22: 'BCM_LOCKUNLOCKFEEDBACK_CRASHUNLOCK_C5_F6_BD_E2_CB_F8', 23: 'BCM_LOCKUNLOCKFEEDBACK_KEYREMOVALUNLOCK_B0_CE_B3_E2_CB_F8', 24: 'BCM_LOCKUNLOCKFEEDBACK_ENGOFFUNLOCK_BB_F0_BD_E2_CB_F8', 31: 'BCM_LOCKUNLOCKFEEDBACK_ANY_OTHER_UNLOCK'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|31]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_lockunlockfeedbackType bcm_lockunlockfeedback(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_DoorLockStsTrunk', 'enum': {0: 'BCM_DOORLOCKSTSTRUNK_NO_USED', 1: 'BCM_DOORLOCKSTSTRUNK_LOCKED', 2: 'BCM_DOORLOCKSTSTRUNK_UNLOCKED', 3: 'BCM_DOORLOCKSTSTRUNK_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_doorlockststrunkType bcm_doorlockststrunk(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_DoorStsTrunk', 'enum': {0: 'BCM_DOORSTSTRUNK_NO_USED', 1: 'BCM_DOORSTSTRUNK_CLOSED', 2: 'BCM_DOORSTSTRUNK_OPEN', 3: 'BCM_DOORSTSTRUNK_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_doorststrunkType bcm_doorststrunk(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_DoorLockStsRearRi', 'enum': {0: 'BCM_DOORLOCKSTSREARRI_NO_USED', 1: 'BCM_DOORLOCKSTSREARRI_LOCKED', 2: 'BCM_DOORLOCKSTSREARRI_UNLOCKED', 3: 'BCM_DOORLOCKSTSREARRI_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_doorlockstsrearriType bcm_doorlockstsrearri(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_DoorLockStsRearLe', 'enum': {0: 'BCM_DOORLOCKSTSREARLE_NO_USED', 1: 'BCM_DOORLOCKSTSREARLE_LOCKED', 2: 'BCM_DOORLOCKSTSREARLE_UNLOCKED', 3: 'BCM_DOORLOCKSTSREARLE_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_doorlockstsrearleType bcm_doorlockstsrearle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_DoorLockStsFrntRi', 'enum': {0: 'BCM_DOORLOCKSTSFRNTRI_NO_USED', 1: 'BCM_DOORLOCKSTSFRNTRI_LOCKED', 2: 'BCM_DOORLOCKSTSFRNTRI_UNLOCKED', 3: 'BCM_DOORLOCKSTSFRNTRI_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_doorlockstsfrntriType bcm_doorlockstsfrntri(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_DoorLockStsFrntLe', 'enum': {0: 'BCM_DOORLOCKSTSFRNTLE_NO_USED', 1: 'BCM_DOORLOCKSTSFRNTLE_LOCKED', 2: 'BCM_DOORLOCKSTSFRNTLE_UNLOCKED', 3: 'BCM_DOORLOCKSTSFRNTLE_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_doorlockstsfrntleType bcm_doorlockstsfrntle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_DoorStsRearRi', 'enum': {0: 'BCM_DOORSTSREARRI_NO_USED', 1: 'BCM_DOORSTSREARRI_CLOSED', 2: 'BCM_DOORSTSREARRI_OPEN', 3: 'BCM_DOORSTSREARRI_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 1, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_doorstsrearriType bcm_doorstsrearri(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_DoorStsRearLe', 'enum': {0: 'BCM_DOORSTSREARLE_NO_USED', 1: 'BCM_DOORSTSREARLE_CLOSED', 2: 'BCM_DOORSTSREARLE_OPEN', 3: 'BCM_DOORSTSREARLE_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_doorstsrearleType bcm_doorstsrearle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_DoorStsFrntRi', 'enum': {0: 'BCM_DOORSTSFRNTRI_NO_USED', 1: 'BCM_DOORSTSFRNTRI_CLOSED', 2: 'BCM_DOORSTSFRNTRI_OPEN', 3: 'BCM_DOORSTSFRNTRI_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_doorstsfrntriType bcm_doorstsfrntri(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_DoorStsFrntLe', 'enum': {0: 'BCM_DOORSTSFRNTLE_NO_USED', 1: 'BCM_DOORSTSFRNTLE_CLOSED', 2: 'BCM_DOORSTSFRNTLE_OPEN', 3: 'BCM_DOORSTSFRNTLE_FAULT'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_door_0x310_310::Bcm_doorstsfrntleType bcm_doorstsfrntle(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_BCM_DOOR_0X310_310_H_
