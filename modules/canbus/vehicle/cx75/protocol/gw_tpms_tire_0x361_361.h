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

#ifndef MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_GW_TPMS_TIRE_0X361_361_H_
#define MODULES_CANBUS_VEHICLE_CX75_PROTOCOL_GW_TPMS_TIRE_0X361_361_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Gwtpmstire0x361361 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Gwtpmstire0x361361();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'TPMS_RF_Pressure', 'offset': 0.0, 'precision': 2.75, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|698.5]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Kpa'}
  double tpms_rf_pressure(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_RR_Pressure', 'offset': 0.0, 'precision': 2.75, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|698.5]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Kpa'}
  double tpms_rr_pressure(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_LR_Pressure', 'offset': 0.0, 'precision': 2.75, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|698.5]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Kpa'}
  double tpms_lr_pressure(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_Tire_Temperature', 'offset': -60.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-60|194]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg.C'}
  int tpms_tire_temperature(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_LF_Pressure_Warning', 'enum': {0: 'TPMS_LF_PRESSURE_WARNING_NO_WARNING', 1: 'TPMS_LF_PRESSURE_WARNING_HIGH_PRESSURE_WARNING', 2: 'TPMS_LF_PRESSURE_WARNING_LOW_PRESSURE_WARNING', 3: 'TPMS_LF_PRESSURE_WARNING_QUIK_LEAKAGE_RESERVED', 4: 'TPMS_LF_PRESSURE_WARNING_LOST_SENSOR', 5: 'TPMS_LF_PRESSURE_WARNING_SENSOR_BATTERY_LOW', 6: 'TPMS_LF_PRESSURE_WARNING_SENSOR_FAULTURE_RESERVED', 7: 'TPMS_LF_PRESSURE_WARNING_LOW_PRESSURE_WARN_QUIK_LEAK_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tpms_tire_0x361_361::Tpms_lf_pressure_warningType tpms_lf_pressure_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_RF_Pressure_Warning', 'enum': {0: 'TPMS_RF_PRESSURE_WARNING_NO_WARNING', 1: 'TPMS_RF_PRESSURE_WARNING_HIGH_PRESSURE_WARNING', 2: 'TPMS_RF_PRESSURE_WARNING_LOW_PRESSURE_WARNING', 3: 'TPMS_RF_PRESSURE_WARNING_QUIK_LEAKAGE_RESERVED', 4: 'TPMS_RF_PRESSURE_WARNING_LOST_SENSOR', 5: 'TPMS_RF_PRESSURE_WARNING_SENSOR_BATTERY_LOW', 6: 'TPMS_RF_PRESSURE_WARNING_SENSOR_FAULTURE_RESERVED', 7: 'TPMS_RF_PRESSURE_WARNING_LOW_PRESSURE_WARN_QUIK_LEAK_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tpms_tire_0x361_361::Tpms_rf_pressure_warningType tpms_rf_pressure_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_System_Status', 'enum': {0: 'TPMS_SYSTEM_STATUS_NO_ANY_ERROR', 1: 'TPMS_SYSTEM_STATUS_SYSTEM_ERROR', 2: 'TPMS_SYSTEM_STATUS_SYETEM_WAIT_FOR_LEARNING', 3: 'TPMS_SYSTEM_STATUS_WINTER_MODE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tpms_tire_0x361_361::Tpms_system_statusType tpms_system_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_RR_Pressure_Warning', 'enum': {0: 'TPMS_RR_PRESSURE_WARNING_NO_WARNING', 1: 'TPMS_RR_PRESSURE_WARNING_HIGH_PRESSURE_WARNING', 2: 'TPMS_RR_PRESSURE_WARNING_LOW_PRESSURE_WARNING', 3: 'TPMS_RR_PRESSURE_WARNING_QUIK_LEAKAGE_RESERVED', 4: 'TPMS_RR_PRESSURE_WARNING_LOST_SENSOR', 5: 'TPMS_RR_PRESSURE_WARNING_SENSOR_BATTERY_LOW', 6: 'TPMS_RR_PRESSURE_WARNING_SENSOR_FAULTURE_RESERVED', 7: 'TPMS_RR_PRESSURE_WARNING_LOW_PRESSURE_WARN_QUIK_LEAK_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 50, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tpms_tire_0x361_361::Tpms_rr_pressure_warningType tpms_rr_pressure_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_LR_Pressure_Warning', 'enum': {0: 'TPMS_LR_PRESSURE_WARNING_NO_WARNING', 1: 'TPMS_LR_PRESSURE_WARNING_HIGH_PRESSURE_WARNING', 2: 'TPMS_LR_PRESSURE_WARNING_LOW_PRESSURE_WARNING', 3: 'TPMS_LR_PRESSURE_WARNING_QUIK_LEAKAGE_RESERVED', 4: 'TPMS_LR_PRESSURE_WARNING_LOST_SENSOR', 5: 'TPMS_LR_PRESSURE_WARNING_SENSOR_BATTERY_LOW', 6: 'TPMS_LR_PRESSURE_WARNING_SENSOR_FAULTURE_RESERVED', 7: 'TPMS_LR_PRESSURE_WARNING_LOW_PRESSURE_WARN_QUIK_LEAK_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tpms_tire_0x361_361::Tpms_lr_pressure_warningType tpms_lr_pressure_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_Temperature_Warning', 'enum': {0: 'TPMS_TEMPERATURE_WARNING_NO_TEMPERATURE_WARNING', 1: 'TPMS_TEMPERATURE_WARNING_TEMPERATURE_WARNING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tpms_tire_0x361_361::Tpms_temperature_warningType tpms_temperature_warning(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_resrved', 'enum': {0: 'TPMS_RESRVED_RESERVED', 1: 'TPMS_RESRVED_RESERVED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tpms_tire_0x361_361::Tpms_resrvedType tpms_resrved(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Tire_Position', 'enum': {0: 'TIRE_POSITION_NO_ANY_SENSOR_ERROR', 1: 'TIRE_POSITION_LEFT_FRONT_TIRE', 2: 'TIRE_POSITION_RIGHT_FRONT_TIRE', 3: 'TIRE_POSITION_RIGHT_REAR_TIRE', 4: 'TIRE_POSITION_LEFT_REAR_TIRE', 5: 'TIRE_POSITION_RESERVED', 6: 'TIRE_POSITION_RESERVED', 7: 'TIRE_POSITION_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 58, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tpms_tire_0x361_361::Tire_positionType tire_position(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_Lamp_Status', 'enum': {0: 'TPMS_LAMP_STATUS_NO_LAMP_LIGHT', 1: 'TPMS_LAMP_STATUS_LAMP_FLASH', 2: 'TPMS_LAMP_STATUS_LAMP_LIGHT', 3: 'TPMS_LAMP_STATUS_RESEVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 60, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tpms_tire_0x361_361::Tpms_lamp_statusType tpms_lamp_status(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_AL_state', 'enum': {0: 'TPMS_AL_STATE_DEFAULTS', 1: 'TPMS_AL_STATE_SELF_LEARNING', 2: 'TPMS_AL_STATE_SELF_LEARNING_OVER', 3: 'TPMS_AL_STATE_RESERVE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 62, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Gw_tpms_tire_0x361_361::Tpms_al_stateType tpms_al_state(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'TPMS_LF_Pressure', 'offset': 0.0, 'precision': 2.75, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|698.5]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Kpa'}
  double tpms_lf_pressure(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_CX75_PROTOCOL_GW_TPMS_TIRE_0X361_361_H_
