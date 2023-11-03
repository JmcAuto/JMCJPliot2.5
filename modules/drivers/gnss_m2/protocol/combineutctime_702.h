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

#ifndef MODULES_CANBUS_VEHICLE_M2_PROTOCOL_COMBINEUTCTIME_702_H_
#define MODULES_CANBUS_VEHICLE_M2_PROTOCOL_COMBINEUTCTIME_702_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/gnss_m2/proto/m2.pb.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

class Combineutctime702 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    jmc_auto::drivers::gnss_m2::M2> {
 public:
  static const int32_t ID;
  Combineutctime702();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     jmc_auto::drivers::gnss_m2::M2* m2) const override;

 private:

  // config detail: {'name': 'UtcMilSecond', 'offset': 0.0, 'precision': 0.01, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 63, 'type': 'double', 'order': 'motorola', 'physical_unit': 's'}
  double utcmilsecond(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'UtcSecond', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|60]', 'bit': 55, 'type': 'int', 'order': 'motorola', 'physical_unit': 's'}
  int utcsecond(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'UtcMinute', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|60]', 'bit': 47, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int utcminute(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'UtcHour', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|24]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int utchour(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'UtcDate', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': True, 'physical_range': '[0|31]', 'bit': 31, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int utcdate(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'UtcMonth', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|12]', 'bit': 23, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int utcmonth(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'UtcYear', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int utcyear(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace m2
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_M2_PROTOCOL_COMBINEUTCTIME_702_H_
