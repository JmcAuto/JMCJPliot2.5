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

#ifndef MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_BCM_BODYSTS_0X344_344_H_
#define MODULES_CANBUS_VEHICLE_TESHUN_PROTOCOL_BCM_BODYSTS_0X344_344_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

class Bcmbodysts0x344344 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Bcmbodysts0x344344();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'BCM_BrakeLampStatus', 'enum': {0: 'BCM_BRAKELAMPSTATUS_BRAKE_LAMP_INACTIVE', 1: 'BCM_BRAKELAMPSTATUS_BRAKE_LAMP_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_brakelampstatusType bcm_brakelampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_RearfogLampStatus', 'enum': {0: 'BCM_REARFOGLAMPSTATUS_INVALID', 1: 'BCM_REARFOGLAMPSTATUS_OFF', 2: 'BCM_REARFOGLAMPSTATUS_ON', 3: 'BCM_REARFOGLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_rearfoglampstatusType bcm_rearfoglampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_FrontfogLampStatus', 'enum': {0: 'BCM_FRONTFOGLAMPSTATUS_INVALID', 1: 'BCM_FRONTFOGLAMPSTATUS_OFF', 2: 'BCM_FRONTFOGLAMPSTATUS_ON', 3: 'BCM_FRONTFOGLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_frontfoglampstatusType bcm_frontfoglampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_WasherStatus', 'enum': {0: 'BCM_WASHERSTATUS_INVALID', 1: 'BCM_WASHERSTATUS_OFF', 2: 'BCM_WASHERSTATUS_ON', 3: 'BCM_WASHERSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_washerstatusType bcm_washerstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_WiperStatus', 'enum': {0: 'BCM_WIPERSTATUS_INVALID', 1: 'BCM_WIPERSTATUS_OFF', 2: 'BCM_WIPERSTATUS_LOW_SPEED', 3: 'BCM_WIPERSTATUS_HIGH_SPEED', 7: 'BCM_WIPERSTATUS_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_wiperstatusType bcm_wiperstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_Doorlockfeedback', 'enum': {0: 'BCM_DOORLOCKFEEDBACK_INVALID', 1: 'BCM_DOORLOCKFEEDBACK_LOCK_ACTION', 2: 'BCM_DOORLOCKFEEDBACK_UNLOCK_ACTION', 3: 'BCM_DOORLOCKFEEDBACK_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_doorlockfeedbackType bcm_doorlockfeedback(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_HornStatus', 'enum': {0: 'BCM_HORNSTATUS_INVALID', 1: 'BCM_HORNSTATUS_OFF', 2: 'BCM_HORNSTATUS_ON', 3: 'BCM_HORNSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_hornstatusType bcm_hornstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_HighBeamLampStatus', 'enum': {0: 'BCM_HIGHBEAMLAMPSTATUS_INVALID', 1: 'BCM_HIGHBEAMLAMPSTATUS_OFF', 2: 'BCM_HIGHBEAMLAMPSTATUS_ON', 3: 'BCM_HIGHBEAMLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_highbeamlampstatusType bcm_highbeamlampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_LowBeamLampStatus', 'enum': {0: 'BCM_LOWBEAMLAMPSTATUS_INVALID', 1: 'BCM_LOWBEAMLAMPSTATUS_OFF', 2: 'BCM_LOWBEAMLAMPSTATUS_ON', 3: 'BCM_LOWBEAMLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_lowbeamlampstatusType bcm_lowbeamlampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_PositionLampStatus', 'enum': {0: 'BCM_POSITIONLAMPSTATUS_INVALID', 1: 'BCM_POSITIONLAMPSTATUS_OFF', 2: 'BCM_POSITIONLAMPSTATUS_ON', 3: 'BCM_POSITIONLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_positionlampstatusType bcm_positionlampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_HazardLampStatus', 'enum': {0: 'BCM_HAZARDLAMPSTATUS_INVALID', 1: 'BCM_HAZARDLAMPSTATUS_OFF', 2: 'BCM_HAZARDLAMPSTATUS_BLINK', 3: 'BCM_HAZARDLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_hazardlampstatusType bcm_hazardlampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_RightTurnLampStatus', 'enum': {0: 'BCM_RIGHTTURNLAMPSTATUS_INVALID', 1: 'BCM_RIGHTTURNLAMPSTATUS_OFF', 2: 'BCM_RIGHTTURNLAMPSTATUS_BLINK', 3: 'BCM_RIGHTTURNLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_rightturnlampstatusType bcm_rightturnlampstatus(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'BCM_LeftTurnLampStatus', 'enum': {0: 'BCM_LEFTTURNLAMPSTATUS_INVALID', 1: 'BCM_LEFTTURNLAMPSTATUS_OFF', 2: 'BCM_LEFTTURNLAMPSTATUS_BLINK', 3: 'BCM_LEFTTURNLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Bcm_bodysts_0x344_344::Bcm_leftturnlampstatusType bcm_leftturnlampstatus(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_TESHUN_PROTOCOL_BCM_BODYSTS_0X344_344_H_
