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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_INS_ACC_500_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_INS_ACC_500_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Insacc500 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Insacc500();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ACC_X', 'offset': -4.0, 'precision': 0.0001220703125, 'len': 16, 'is_signed_var': True, 'physical_range': '[-4|4]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'g'}
  double acc_x(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ACC_Y', 'offset': -4.0, 'precision': 0.0001220703125, 'len': 16, 'is_signed_var': True, 'physical_range': '[-4|4]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'g'}
  double acc_y(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ACC_Z', 'offset': -4.0, 'precision': 0.0001220703125, 'len': 16, 'is_signed_var': True, 'physical_range': '[-4|4]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'g'}
  double acc_z(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_INS_ACC_500_H_
