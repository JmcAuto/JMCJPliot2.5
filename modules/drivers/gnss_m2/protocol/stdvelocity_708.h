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

#ifndef MODULES_CANBUS_VEHICLE_M2_PROTOCOL_STDVELOCITY_708_H_
#define MODULES_CANBUS_VEHICLE_M2_PROTOCOL_STDVELOCITY_708_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/gnss_m2/proto/m2.pb.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

class Stdvelocity708 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    jmc_auto::drivers::gnss_m2::M2> {
 public:
  static const int32_t ID;
  Stdvelocity708();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     jmc_auto::drivers::gnss_m2::M2* m2) const override;

 private:

  // config detail: {'name': 'VelocityUpStd', 'offset': 0.0, 'precision': 0.0001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double velocityupstd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VelocityNorthStd', 'offset': 0.0, 'precision': 0.0001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double velocitynorthstd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'VelocityEastStd', 'offset': 0.0, 'precision': 0.0001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
  double velocityeaststd(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace m2
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_M2_PROTOCOL_STDVELOCITY_708_H_
