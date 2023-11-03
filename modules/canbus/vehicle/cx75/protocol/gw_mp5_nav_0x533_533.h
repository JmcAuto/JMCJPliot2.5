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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_MP5_NAV_0X533_533_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_GW_MP5_NAV_0X533_533_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwmp5nav0x533533 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwmp5nav0x533533();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'Nav_SpeedLimitUnits', 'enum': {0: 'NAV_SPEEDLIMITUNITS_UNKNOWN', 1: 'NAV_SPEEDLIMITUNITS_MPH', 2: 'NAV_SPEEDLIMITUNITS_KMH', 3: 'NAV_SPEEDLIMITUNITS_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 11, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_nav_0x533_533::Nav_speedlimitunitsType nav_speedlimitunits(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Nav_CurrRoadType', 'enum': {0: 'NAV_CURRROADTYPE_UNKNOW', 1: 'NAV_CURRROADTYPE_HIGH_SPEED_ROAD', 2: 'NAV_CURRROADTYPE_CITY_EXPRESS_WAY', 3: 'NAV_CURRROADTYPE_DOWNTOWN_ROAD', 4: 'NAV_CURRROADTYPE_COUNTRY_ROAD'}, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|15]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_nav_0x533_533::Nav_currroadtypeType nav_currroadtype(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Nav_SpeedLimit', 'enum': {0: 'NAV_SPEEDLIMIT_NO_DISPLAY', 1: 'NAV_SPEEDLIMIT_SPL_5', 2: 'NAV_SPEEDLIMIT_SPL_10', 3: 'NAV_SPEEDLIMIT_SPL_15', 4: 'NAV_SPEEDLIMIT_SPL_20', 5: 'NAV_SPEEDLIMIT_SPL_25', 26: 'NAV_SPEEDLIMIT_SPL_130'}, 'precision': 1.0, 'len': 6, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|63]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_nav_0x533_533::Nav_speedlimitType nav_speedlimit(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_APAActive_Cmd', 'enum': {0: 'MP5_APAACTIVE_CMD_NO_REQUEST', 1: 'MP5_APAACTIVE_CMD_REQUEST'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 26, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_nav_0x533_533::Mp5_apaactive_cmdType mp5_apaactive_cmd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_APA_ConfirmButton', 'enum': {0: 'MP5_APA_CONFIRMBUTTON_NO_BUTTON_PRESS', 1: 'MP5_APA_CONFIRMBUTTON_COMFIRM_BUTTON_PRESS', 2: 'MP5_APA_CONFIRMBUTTON_TERMINATED_BUTTON_PRESS', 3: 'MP5_APA_CONFIRMBUTTON_RESERVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 28, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_nav_0x533_533::Mp5_apa_confirmbuttonType mp5_apa_confirmbutton(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MP5_APA_Function_Select', 'enum': {0: 'MP5_APA_FUNCTION_SELECT_NO_BUTTON_PRESS', 1: 'MP5_APA_FUNCTION_SELECT_PPSC_BUTTON_PRESS', 2: 'MP5_APA_FUNCTION_SELECT_CPSC_BUTTON_PRESS', 3: 'MP5_APA_FUNCTION_SELECT_POC_BUTTON_PRESS'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 30, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_nav_0x533_533::Mp5_apa_function_selectType mp5_apa_function_select(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Nav_Sts', 'enum': {0: 'NAV_STS_INACTIVE', 1: 'NAV_STS_ACTIVE'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_nav_0x533_533::Nav_stsType nav_sts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Nav_CountryCode', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 7, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int nav_countrycode(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Nav_SpeedLimitStatus', 'enum': {0: 'NAV_SPEEDLIMITSTATUS_SL_UNKNOWN', 1: 'NAV_SPEEDLIMITSTATUS_SL_EXISTS', 2: 'NAV_SPEEDLIMITSTATUS_SL_NOLIMIT', 3: 'NAV_SPEEDLIMITSTATUS_SL_PLURAL'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 9, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_mp5_nav_0x533_533::Nav_speedlimitstatusType nav_speedlimitstatus(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_GW_MP5_NAV_0X533_533_H_
