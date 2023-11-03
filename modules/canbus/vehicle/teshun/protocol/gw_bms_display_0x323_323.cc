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

#include "modules/canbus/vehicle/teshun/protocol/gw_bms_display_0x323_323.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace teshun {

using ::jmc_auto::drivers::canbus::Byte;

Gwbmsdisplay0x323323::Gwbmsdisplay0x323323() {}
const int32_t Gwbmsdisplay0x323323::ID = 0x323;

void Gwbmsdisplay0x323323::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_teshun()->mutable_gw_bms_display_0x323_323()->set_checksum_0x323(checksum_0x323(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_display_0x323_323()->set_rolling_counter_0x323(rolling_counter_0x323(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_display_0x323_323()->set_bms_consumeq_est(bms_consumeq_est(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_display_0x323_323()->set_bms_availableeq_est(bms_availableeq_est(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_display_0x323_323()->set_bms_soh_est(bms_soh_est(bytes, length));
  chassis->mutable_teshun()->mutable_gw_bms_display_0x323_323()->set_bms_socdisplay_est(bms_socdisplay_est(bytes, length));
}

// config detail: {'name': 'checksum_0x323', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwbmsdisplay0x323323::checksum_0x323(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'rolling_counter_0x323', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwbmsdisplay0x323323::rolling_counter_0x323(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  int ret = x;
  return ret;
}

// config detail: {'name': 'bms_consumeq_est', 'offset': 0.0, 'precision': 0.1, 'len': 14, 'is_signed_var': False, 'physical_range': '[0|800]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kwh'}
double Gwbmsdisplay0x323323::bms_consumeq_est(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(2, 6);
  x <<= 6;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'bms_availableeq_est', 'offset': 0.0, 'precision': 0.1, 'len': 14, 'is_signed_var': False, 'physical_range': '[0|800]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'kwh'}
double Gwbmsdisplay0x323323::bms_availableeq_est(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(2, 6);
  x <<= 6;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'name': 'bms_soh_est', 'offset': 0.0, 'precision': 0.25, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|400]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Gwbmsdisplay0x323323::bms_soh_est(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  double ret = x * 0.250000;
  return ret;
}

// config detail: {'name': 'bms_socdisplay_est', 'offset': 0.0, 'precision': 0.4, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': '%'}
double Gwbmsdisplay0x323323::bms_socdisplay_est(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.400000;
  return ret;
}
}  // namespace teshun
}  // namespace canbus
}  // namespace jmc_auto
