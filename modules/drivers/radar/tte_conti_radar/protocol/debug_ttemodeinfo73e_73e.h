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

#ifndef MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_PROTOCOL_DEBUG_TTEMODEINFO73E_73E_H_
#define MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_PROTOCOL_DEBUG_TTEMODEINFO73E_73E_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/tte_conti_radar.pb.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

class Debugttemodeinfo73e73e : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::drivers::TteContiRadar> {
 public:
  static const int32_t ID;
  Debugttemodeinfo73e73e();
  void Parse(const std::uint8_t* bytes, int32_t length,
                   TteContiRadar* ttecontiradar) const override;

 private:

  // config detail: {'name': 'TteModeInfo73E_Copy_7', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ttemodeinfo73e_copy_7(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TteModeInfo73E_Copy_6', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ttemodeinfo73e_copy_6(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TteModeInfo73E_Copy_5', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ttemodeinfo73e_copy_5(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TteModeInfo73E_Copy_4', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ttemodeinfo73e_copy_4(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TteModeInfo73E_Copy_3', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ttemodeinfo73e_copy_3(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TteModeInfo73E_Copy_2', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ttemodeinfo73e_copy_2(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TteModeInfo73E_Copy_1', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 15, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ttemodeinfo73e_copy_1(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TteModeInfo73E', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int ttemodeinfo73e(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_DEBUG_TTEMODEINFO73E_73E_H_
