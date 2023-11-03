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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_GW_IC_TIME_0X581_581_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_GW_IC_TIME_0X581_581_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwictime0x581581 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwictime0x581581();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'IC_Month', 'enum': {0: 'IC_MONTH_1', 1: 'IC_MONTH_2', 2: 'IC_MONTH_3', 3: 'IC_MONTH_4', 4: 'IC_MONTH_5', 5: 'IC_MONTH_6', 6: 'IC_MONTH_7', 7: 'IC_MONTH_8', 8: 'IC_MONTH_9', 9: 'IC_MONTH_10', 10: 'IC_MONTH_11', 11: 'IC_MONTH_12'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 1.0, 'physical_range': '[1|12]', 'bit': 15, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ic_time_0x581_581::Ic_monthType ic_month(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_Date', 'enum': {0: 'IC_DATE_1', 1: 'IC_DATE_2', 2: 'IC_DATE_3', 3: 'IC_DATE_4', 4: 'IC_DATE_5', 5: 'IC_DATE_6', 6: 'IC_DATE_7', 7: 'IC_DATE_8', 8: 'IC_DATE_9', 9: 'IC_DATE_10', 10: 'IC_DATE_11', 11: 'IC_DATE_12', 12: 'IC_DATE_13', 13: 'IC_DATE_14', 14: 'IC_DATE_15', 15: 'IC_DATE_16', 16: 'IC_DATE_17', 17: 'IC_DATE_18', 18: 'IC_DATE_19', 19: 'IC_DATE_20', 20: 'IC_DATE_21', 21: 'IC_DATE_22', 22: 'IC_DATE_23', 23: 'IC_DATE_24', 24: 'IC_DATE_25', 25: 'IC_DATE_26', 26: 'IC_DATE_27', 27: 'IC_DATE_28', 28: 'IC_DATE_29', 29: 'IC_DATE_30', 30: 'IC_DATE_31'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 1.0, 'physical_range': '[1|31]', 'bit': 23, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ic_time_0x581_581::Ic_dateType ic_date(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_Hour', 'enum': {0: 'IC_HOUR_0', 1: 'IC_HOUR_1', 2: 'IC_HOUR_2', 3: 'IC_HOUR_3', 4: 'IC_HOUR_4', 5: 'IC_HOUR_5', 6: 'IC_HOUR_6', 7: 'IC_HOUR_7', 8: 'IC_HOUR_8', 9: 'IC_HOUR_9', 10: 'IC_HOUR_10', 11: 'IC_HOUR_11', 12: 'IC_HOUR_12', 13: 'IC_HOUR_13', 14: 'IC_HOUR_14', 15: 'IC_HOUR_15', 16: 'IC_HOUR_16', 17: 'IC_HOUR_17', 18: 'IC_HOUR_18', 19: 'IC_HOUR_19', 20: 'IC_HOUR_20', 21: 'IC_HOUR_21', 22: 'IC_HOUR_22', 23: 'IC_HOUR_23'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|23]', 'bit': 31, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ic_time_0x581_581::Ic_hourType ic_hour(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_Minute', 'enum': {0: 'IC_MINUTE_0', 1: 'IC_MINUTE_1', 2: 'IC_MINUTE_2', 3: 'IC_MINUTE_3', 4: 'IC_MINUTE_4', 5: 'IC_MINUTE_5', 6: 'IC_MINUTE_6', 7: 'IC_MINUTE_7', 8: 'IC_MINUTE_8', 9: 'IC_MINUTE_9', 10: 'IC_MINUTE_10', 11: 'IC_MINUTE_11', 12: 'IC_MINUTE_12', 13: 'IC_MINUTE_13', 14: 'IC_MINUTE_14', 15: 'IC_MINUTE_15', 16: 'IC_MINUTE_16', 17: 'IC_MINUTE_17', 18: 'IC_MINUTE_18', 19: 'IC_MINUTE_19', 20: 'IC_MINUTE_20', 21: 'IC_MINUTE_21', 22: 'IC_MINUTE_22', 23: 'IC_MINUTE_23', 24: 'IC_MINUTE_24', 25: 'IC_MINUTE_25', 26: 'IC_MINUTE_26', 27: 'IC_MINUTE_27', 28: 'IC_MINUTE_28', 29: 'IC_MINUTE_29', 30: 'IC_MINUTE_30', 31: 'IC_MINUTE_31', 32: 'IC_MINUTE_32', 33: 'IC_MINUTE_33', 34: 'IC_MINUTE_34', 35: 'IC_MINUTE_35', 36: 'IC_MINUTE_36', 37: 'IC_MINUTE_37', 38: 'IC_MINUTE_38', 39: 'IC_MINUTE_39', 40: 'IC_MINUTE_40', 41: 'IC_MINUTE_41', 42: 'IC_MINUTE_42', 43: 'IC_MINUTE_43', 44: 'IC_MINUTE_44', 45: 'IC_MINUTE_45', 46: 'IC_MINUTE_46', 47: 'IC_MINUTE_47', 48: 'IC_MINUTE_48', 49: 'IC_MINUTE_49', 50: 'IC_MINUTE_50', 51: 'IC_MINUTE_51', 52: 'IC_MINUTE_52', 53: 'IC_MINUTE_53', 54: 'IC_MINUTE_54', 55: 'IC_MINUTE_55', 56: 'IC_MINUTE_56', 57: 'IC_MINUTE_57', 58: 'IC_MINUTE_58', 59: 'IC_MINUTE_59'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|59]', 'bit': 39, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ic_time_0x581_581::Ic_minuteType ic_minute(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_Second', 'enum': {0: 'IC_SECOND_0', 1: 'IC_SECOND_1', 2: 'IC_SECOND_2', 3: 'IC_SECOND_3', 4: 'IC_SECOND_4', 5: 'IC_SECOND_5', 6: 'IC_SECOND_6', 7: 'IC_SECOND_7', 8: 'IC_SECOND_8', 9: 'IC_SECOND_9', 10: 'IC_SECOND_10', 11: 'IC_SECOND_11', 12: 'IC_SECOND_12', 13: 'IC_SECOND_13', 14: 'IC_SECOND_14', 15: 'IC_SECOND_15', 16: 'IC_SECOND_16', 17: 'IC_SECOND_17', 18: 'IC_SECOND_18', 19: 'IC_SECOND_19', 20: 'IC_SECOND_20', 21: 'IC_SECOND_21', 22: 'IC_SECOND_22', 23: 'IC_SECOND_23', 24: 'IC_SECOND_24', 25: 'IC_SECOND_25', 26: 'IC_SECOND_26', 27: 'IC_SECOND_27', 28: 'IC_SECOND_28', 29: 'IC_SECOND_29', 30: 'IC_SECOND_30', 31: 'IC_SECOND_31', 32: 'IC_SECOND_32', 33: 'IC_SECOND_33', 34: 'IC_SECOND_34', 35: 'IC_SECOND_35', 36: 'IC_SECOND_36', 37: 'IC_SECOND_37', 38: 'IC_SECOND_38', 39: 'IC_SECOND_39', 40: 'IC_SECOND_40', 41: 'IC_SECOND_41', 42: 'IC_SECOND_42', 43: 'IC_SECOND_43', 44: 'IC_SECOND_44', 45: 'IC_SECOND_45', 46: 'IC_SECOND_46', 47: 'IC_SECOND_47', 48: 'IC_SECOND_48', 49: 'IC_SECOND_49', 50: 'IC_SECOND_50', 51: 'IC_SECOND_51', 52: 'IC_SECOND_52', 53: 'IC_SECOND_53', 54: 'IC_SECOND_54', 55: 'IC_SECOND_55', 56: 'IC_SECOND_56', 57: 'IC_SECOND_57', 58: 'IC_SECOND_58', 59: 'IC_SECOND_59'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|59]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ic_time_0x581_581::Ic_secondType ic_second(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'IC_Year', 'enum': {2: 'IC_YEAR_2019', 3: 'IC_YEAR_2020', 4: 'IC_YEAR_2021'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 2017.0, 'physical_range': '[2017|2272]', 'bit': 7, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_ic_time_0x581_581::Ic_yearType ic_year(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_GW_IC_TIME_0X581_581_H_
