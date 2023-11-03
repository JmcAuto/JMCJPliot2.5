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

#include "modules/canbus/vehicle/teshun/protocol/bcm_bodysts_0x344_344.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Bcmbodysts0x344344::Bcmbodysts0x344344() {}
const int32_t Bcmbodysts0x344344::ID = 0x344;

void Bcmbodysts0x344344::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_brakelampstatus(bcm_brakelampstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_rearfoglampstatus(bcm_rearfoglampstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_frontfoglampstatus(bcm_frontfoglampstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_washerstatus(bcm_washerstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_wiperstatus(bcm_wiperstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_doorlockfeedback(bcm_doorlockfeedback(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_hornstatus(bcm_hornstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_highbeamlampstatus(bcm_highbeamlampstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_lowbeamlampstatus(bcm_lowbeamlampstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_positionlampstatus(bcm_positionlampstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_hazardlampstatus(bcm_hazardlampstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_rightturnlampstatus(bcm_rightturnlampstatus(bytes, length));
  chassis->mutable_teshun()->mutable_bcm_bodysts_0x344_344()->set_bcm_leftturnlampstatus(bcm_leftturnlampstatus(bytes, length));
}

// config detail: {'name': 'bcm_brakelampstatus', 'enum': {0: 'BCM_BRAKELAMPSTATUS_BRAKE_LAMP_INACTIVE', 1: 'BCM_BRAKELAMPSTATUS_BRAKE_LAMP_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 35, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_brakelampstatusType Bcmbodysts0x344344::bcm_brakelampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 1);

  Bcm_bodysts_0x344_344::Bcm_brakelampstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_brakelampstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_rearfoglampstatus', 'enum': {0: 'BCM_REARFOGLAMPSTATUS_INVALID', 1: 'BCM_REARFOGLAMPSTATUS_OFF', 2: 'BCM_REARFOGLAMPSTATUS_ON', 3: 'BCM_REARFOGLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 37, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_rearfoglampstatusType Bcmbodysts0x344344::bcm_rearfoglampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 2);

  Bcm_bodysts_0x344_344::Bcm_rearfoglampstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_rearfoglampstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_frontfoglampstatus', 'enum': {0: 'BCM_FRONTFOGLAMPSTATUS_INVALID', 1: 'BCM_FRONTFOGLAMPSTATUS_OFF', 2: 'BCM_FRONTFOGLAMPSTATUS_ON', 3: 'BCM_FRONTFOGLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_frontfoglampstatusType Bcmbodysts0x344344::bcm_frontfoglampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 2);

  Bcm_bodysts_0x344_344::Bcm_frontfoglampstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_frontfoglampstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_washerstatus', 'enum': {0: 'BCM_WASHERSTATUS_INVALID', 1: 'BCM_WASHERSTATUS_OFF', 2: 'BCM_WASHERSTATUS_ON', 3: 'BCM_WASHERSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_washerstatusType Bcmbodysts0x344344::bcm_washerstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 2);

  Bcm_bodysts_0x344_344::Bcm_washerstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_washerstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_wiperstatus', 'enum': {0: 'BCM_WIPERSTATUS_INVALID', 1: 'BCM_WIPERSTATUS_OFF', 2: 'BCM_WIPERSTATUS_LOW_SPEED', 3: 'BCM_WIPERSTATUS_HIGH_SPEED', 7: 'BCM_WIPERSTATUS_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_wiperstatusType Bcmbodysts0x344344::bcm_wiperstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 3);

  Bcm_bodysts_0x344_344::Bcm_wiperstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_wiperstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_doorlockfeedback', 'enum': {0: 'BCM_DOORLOCKFEEDBACK_INVALID', 1: 'BCM_DOORLOCKFEEDBACK_LOCK_ACTION', 2: 'BCM_DOORLOCKFEEDBACK_UNLOCK_ACTION', 3: 'BCM_DOORLOCKFEEDBACK_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 21, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_doorlockfeedbackType Bcmbodysts0x344344::bcm_doorlockfeedback(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(4, 2);

  Bcm_bodysts_0x344_344::Bcm_doorlockfeedbackType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_doorlockfeedbackType>(x);
  return ret;
}

// config detail: {'name': 'bcm_hornstatus', 'enum': {0: 'BCM_HORNSTATUS_INVALID', 1: 'BCM_HORNSTATUS_OFF', 2: 'BCM_HORNSTATUS_ON', 3: 'BCM_HORNSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_hornstatusType Bcmbodysts0x344344::bcm_hornstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(6, 2);

  Bcm_bodysts_0x344_344::Bcm_hornstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_hornstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_highbeamlampstatus', 'enum': {0: 'BCM_HIGHBEAMLAMPSTATUS_INVALID', 1: 'BCM_HIGHBEAMLAMPSTATUS_OFF', 2: 'BCM_HIGHBEAMLAMPSTATUS_ON', 3: 'BCM_HIGHBEAMLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_highbeamlampstatusType Bcmbodysts0x344344::bcm_highbeamlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  Bcm_bodysts_0x344_344::Bcm_highbeamlampstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_highbeamlampstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_lowbeamlampstatus', 'enum': {0: 'BCM_LOWBEAMLAMPSTATUS_INVALID', 1: 'BCM_LOWBEAMLAMPSTATUS_OFF', 2: 'BCM_LOWBEAMLAMPSTATUS_ON', 3: 'BCM_LOWBEAMLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 13, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_lowbeamlampstatusType Bcmbodysts0x344344::bcm_lowbeamlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 2);

  Bcm_bodysts_0x344_344::Bcm_lowbeamlampstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_lowbeamlampstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_positionlampstatus', 'enum': {0: 'BCM_POSITIONLAMPSTATUS_INVALID', 1: 'BCM_POSITIONLAMPSTATUS_OFF', 2: 'BCM_POSITIONLAMPSTATUS_ON', 3: 'BCM_POSITIONLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_positionlampstatusType Bcmbodysts0x344344::bcm_positionlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 2);

  Bcm_bodysts_0x344_344::Bcm_positionlampstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_positionlampstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_hazardlampstatus', 'enum': {0: 'BCM_HAZARDLAMPSTATUS_INVALID', 1: 'BCM_HAZARDLAMPSTATUS_OFF', 2: 'BCM_HAZARDLAMPSTATUS_BLINK', 3: 'BCM_HAZARDLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 3, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_hazardlampstatusType Bcmbodysts0x344344::bcm_hazardlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 2);

  Bcm_bodysts_0x344_344::Bcm_hazardlampstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_hazardlampstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_rightturnlampstatus', 'enum': {0: 'BCM_RIGHTTURNLAMPSTATUS_INVALID', 1: 'BCM_RIGHTTURNLAMPSTATUS_OFF', 2: 'BCM_RIGHTTURNLAMPSTATUS_BLINK', 3: 'BCM_RIGHTTURNLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 5, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_rightturnlampstatusType Bcmbodysts0x344344::bcm_rightturnlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 2);

  Bcm_bodysts_0x344_344::Bcm_rightturnlampstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_rightturnlampstatusType>(x);
  return ret;
}

// config detail: {'name': 'bcm_leftturnlampstatus', 'enum': {0: 'BCM_LEFTTURNLAMPSTATUS_INVALID', 1: 'BCM_LEFTTURNLAMPSTATUS_OFF', 2: 'BCM_LEFTTURNLAMPSTATUS_BLINK', 3: 'BCM_LEFTTURNLAMPSTATUS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|2]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Bcm_bodysts_0x344_344::Bcm_leftturnlampstatusType Bcmbodysts0x344344::bcm_leftturnlampstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Bcm_bodysts_0x344_344::Bcm_leftturnlampstatusType ret =  static_cast<Bcm_bodysts_0x344_344::Bcm_leftturnlampstatusType>(x);
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
