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

#include "modules/drivers/radar/tte_conti_radar/protocol/debug_parkingdetection_271.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

using ::jmc_auto::drivers::canbus::Byte;

Debugparkingdetection271::Debugparkingdetection271() {}
const int32_t Debugparkingdetection271::ID = 0x271;

void Debugparkingdetection271::Parse(const std::uint8_t* bytes, int32_t length,
                       TteContiRadar* ttecontiradar) const {
  ttecontiradar->mutable_debug_parkingdetection_271()->set_parkingdetection271(parkingdetection271(bytes, length));
}

// config detail: {'name': 'parkingdetection271', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Debugparkingdetection271::parkingdetection271(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
