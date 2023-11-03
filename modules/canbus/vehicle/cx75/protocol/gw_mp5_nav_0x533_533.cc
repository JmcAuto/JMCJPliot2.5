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

#include "modules/canbus/vehicle/cx75/protocol/gw_mp5_nav_0x533_533.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwmp5nav0x533533::Gwmp5nav0x533533() {}
const int32_t Gwmp5nav0x533533::ID = 0x533;

void Gwmp5nav0x533533::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_mp5_nav_0x533_533()->set_nav_speedlimitunits(nav_speedlimitunits(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_nav_0x533_533()->set_nav_currroadtype(nav_currroadtype(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_nav_0x533_533()->set_nav_speedlimit(nav_speedlimit(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_nav_0x533_533()->set_mp5_apaactive_cmd(mp5_apaactive_cmd(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_nav_0x533_533()->set_mp5_apa_confirmbutton(mp5_apa_confirmbutton(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_nav_0x533_533()->set_mp5_apa_function_select(mp5_apa_function_select(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_nav_0x533_533()->set_nav_sts(nav_sts(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_nav_0x533_533()->set_nav_countrycode(nav_countrycode(bytes, length));
  chassis->mutable_cx75()->mutable_gw_mp5_nav_0x533_533()->set_nav_speedlimitstatus(nav_speedlimitstatus(bytes, length));
}

// config detail: {'name': 'nav_speedlimitunits', 'enum': {0: 'NAV_SPEEDLIMITUNITS_UNKNOWN', 1: 'NAV_SPEEDLIMITUNITS_MPH', 2: 'NAV_SPEEDLIMITUNITS_KMH', 3: 'NAV_SPEEDLIMITUNITS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_nav_0x533_533::Nav_speedlimitunitsType Gwmp5nav0x533533::nav_speedlimitunits(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 2);

  Gw_mp5_nav_0x533_533::Nav_speedlimitunitsType ret =  static_cast<Gw_mp5_nav_0x533_533::Nav_speedlimitunitsType>(x);
  return ret;
}

// config detail: {'name': 'nav_currroadtype', 'enum': {0: 'NAV_CURRROADTYPE_UNKNOW', 1: 'NAV_CURRROADTYPE_HIGH_SPEED_ROAD', 2: 'NAV_CURRROADTYPE_CITY_EXPRESS_WAY', 3: 'NAV_CURRROADTYPE_DOWNTOWN_ROAD', 4: 'NAV_CURRROADTYPE_COUNTRY_ROAD'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_nav_0x533_533::Nav_currroadtypeType Gwmp5nav0x533533::nav_currroadtype(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 4);

  Gw_mp5_nav_0x533_533::Nav_currroadtypeType ret =  static_cast<Gw_mp5_nav_0x533_533::Nav_currroadtypeType>(x);
  return ret;
}

// config detail: {'name': 'nav_speedlimit', 'enum': {0: 'NAV_SPEEDLIMIT_NO_DISPLAY', 1: 'NAV_SPEEDLIMIT_SPL_5', 2: 'NAV_SPEEDLIMIT_SPL_10', 3: 'NAV_SPEEDLIMIT_SPL_15', 4: 'NAV_SPEEDLIMIT_SPL_20', 5: 'NAV_SPEEDLIMIT_SPL_25', 26: 'NAV_SPEEDLIMIT_SPL_130'}, 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|63]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_nav_0x533_533::Nav_speedlimitType Gwmp5nav0x533533::nav_speedlimit(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(2, 6);

  Gw_mp5_nav_0x533_533::Nav_speedlimitType ret =  static_cast<Gw_mp5_nav_0x533_533::Nav_speedlimitType>(x);
  return ret;
}

// config detail: {'name': 'mp5_apaactive_cmd', 'enum': {0: 'MP5_APAACTIVE_CMD_NO_REQUEST', 1: 'MP5_APAACTIVE_CMD_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_nav_0x533_533::Mp5_apaactive_cmdType Gwmp5nav0x533533::mp5_apaactive_cmd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(2, 1);

  Gw_mp5_nav_0x533_533::Mp5_apaactive_cmdType ret =  static_cast<Gw_mp5_nav_0x533_533::Mp5_apaactive_cmdType>(x);
  return ret;
}

// config detail: {'name': 'mp5_apa_confirmbutton', 'enum': {0: 'MP5_APA_CONFIRMBUTTON_NO_BUTTON_PRESS', 1: 'MP5_APA_CONFIRMBUTTON_COMFIRM_BUTTON_PRESS', 2: 'MP5_APA_CONFIRMBUTTON_TERMINATED_BUTTON_PRESS', 3: 'MP5_APA_CONFIRMBUTTON_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_nav_0x533_533::Mp5_apa_confirmbuttonType Gwmp5nav0x533533::mp5_apa_confirmbutton(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(3, 2);

  Gw_mp5_nav_0x533_533::Mp5_apa_confirmbuttonType ret =  static_cast<Gw_mp5_nav_0x533_533::Mp5_apa_confirmbuttonType>(x);
  return ret;
}

// config detail: {'name': 'mp5_apa_function_select', 'enum': {0: 'MP5_APA_FUNCTION_SELECT_NO_BUTTON_PRESS', 1: 'MP5_APA_FUNCTION_SELECT_PPSC_BUTTON_PRESS', 2: 'MP5_APA_FUNCTION_SELECT_CPSC_BUTTON_PRESS', 3: 'MP5_APA_FUNCTION_SELECT_POC_BUTTON_PRESS'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 30, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_nav_0x533_533::Mp5_apa_function_selectType Gwmp5nav0x533533::mp5_apa_function_select(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(5, 2);

  Gw_mp5_nav_0x533_533::Mp5_apa_function_selectType ret =  static_cast<Gw_mp5_nav_0x533_533::Mp5_apa_function_selectType>(x);
  return ret;
}

// config detail: {'name': 'nav_sts', 'enum': {0: 'NAV_STS_INACTIVE', 1: 'NAV_STS_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_nav_0x533_533::Nav_stsType Gwmp5nav0x533533::nav_sts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(7, 1);

  Gw_mp5_nav_0x533_533::Nav_stsType ret =  static_cast<Gw_mp5_nav_0x533_533::Nav_stsType>(x);
  return ret;
}

// config detail: {'name': 'nav_countrycode', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
int Gwmp5nav0x533533::nav_countrycode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'nav_speedlimitstatus', 'enum': {0: 'NAV_SPEEDLIMITSTATUS_SL_UNKNOWN', 1: 'NAV_SPEEDLIMITSTATUS_SL_EXISTS', 2: 'NAV_SPEEDLIMITSTATUS_SL_NOLIMIT', 3: 'NAV_SPEEDLIMITSTATUS_SL_PLURAL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_mp5_nav_0x533_533::Nav_speedlimitstatusType Gwmp5nav0x533533::nav_speedlimitstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  Gw_mp5_nav_0x533_533::Nav_speedlimitstatusType ret =  static_cast<Gw_mp5_nav_0x533_533::Nav_speedlimitstatusType>(x);
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
