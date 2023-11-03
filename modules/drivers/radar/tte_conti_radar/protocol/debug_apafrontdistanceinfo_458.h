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

#ifndef MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_PROTOCOL_DEBUG_APAFRONTDISTANCEINFO_458_H_
#define MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_PROTOCOL_DEBUG_APAFRONTDISTANCEINFO_458_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/tte_conti_radar.pb.h"

namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

class Debugapafrontdistanceinfo458 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::drivers::TteContiRadar> {
 public:
  static const int32_t ID;
  Debugapafrontdistanceinfo458();
  void Parse(const std::uint8_t* bytes, int32_t length,
                   TteContiRadar* ttecontiradar) const override;

 private:

  // config detail: {'name': 'APAFRS_Distance', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
  double apafrs_distance(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'APAFRM_Distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
  double apafrm_distance(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'APAFR_Distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
  double apafr_distance(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'APAFLS_Distance', 'offset': 0.0, 'precision': 0.001, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 55, 'type': 'double', 'order': 'motorola', 'physical_unit': 'm'}
  double apafls_distance(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'APAFLM_Distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 39, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
  double apaflm_distance(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'APAFL_Distance', 'offset': 0.0, 'precision': 2.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': 'cm'}
  double apafl_distance(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_DEBUG_APAFRONTDISTANCEINFO_458_H_
