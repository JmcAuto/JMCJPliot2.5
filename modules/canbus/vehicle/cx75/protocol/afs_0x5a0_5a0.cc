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

#include "modules/canbus/vehicle/cx75/protocol/afs_0x5a0_5a0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Afs0x5a05a0::Afs0x5a05a0() {}
const int32_t Afs0x5a05a0::ID = 0x5A0;

void Afs0x5a05a0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_afs_0x5a0_5a0()->set_afs_work_sts(afs_work_sts(bytes, length));
}

// config detail: {'name': 'afs_work_sts', 'enum': {0: 'AFS_WORK_STS_OFF', 1: 'AFS_WORK_STS_ON', 2: 'AFS_WORK_STS_ERROR', 3: 'AFS_WORK_STS_NOT_CALIBRATION'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Afs_0x5a0_5a0::Afs_work_stsType Afs0x5a05a0::afs_work_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 2);

  Afs_0x5a0_5a0::Afs_work_stsType ret =  static_cast<Afs_0x5a0_5a0::Afs_work_stsType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
