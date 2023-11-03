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

#include "modules/canbus/vehicle/cx75/protocol/gw_tpms_tire_0x361_361.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

using ::jmc_auto::drivers::canbus::Byte;

Gwtpmstire0x361361::Gwtpmstire0x361361() {}
const int32_t Gwtpmstire0x361361::ID = 0x361;

void Gwtpmstire0x361361::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_rf_pressure(tpms_rf_pressure(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_rr_pressure(tpms_rr_pressure(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_lr_pressure(tpms_lr_pressure(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_tire_temperature(tpms_tire_temperature(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_lf_pressure_warning(tpms_lf_pressure_warning(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_rf_pressure_warning(tpms_rf_pressure_warning(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_system_status(tpms_system_status(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_rr_pressure_warning(tpms_rr_pressure_warning(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_lr_pressure_warning(tpms_lr_pressure_warning(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_temperature_warning(tpms_temperature_warning(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_resrved(tpms_resrved(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tire_position(tire_position(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_lamp_status(tpms_lamp_status(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_al_state(tpms_al_state(bytes, length));
  chassis->mutable_cx75()->mutable_gw_tpms_tire_0x361_361()->set_tpms_lf_pressure(tpms_lf_pressure(bytes, length));
}

// config detail: {'name': 'tpms_rf_pressure', 'offset': 0.0, 'precision': 2.75, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|698.5]', 'bit': 15, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Kpa'}
double Gwtpmstire0x361361::tpms_rf_pressure(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.750000;
  return ret;
}

// config detail: {'name': 'tpms_rr_pressure', 'offset': 0.0, 'precision': 2.75, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|698.5]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Kpa'}
double Gwtpmstire0x361361::tpms_rr_pressure(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.750000;
  return ret;
}

// config detail: {'name': 'tpms_lr_pressure', 'offset': 0.0, 'precision': 2.75, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|698.5]', 'bit': 31, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Kpa'}
double Gwtpmstire0x361361::tpms_lr_pressure(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.750000;
  return ret;
}

// config detail: {'name': 'tpms_tire_temperature', 'offset': -60.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-60|194]', 'bit': 39, 'type': 'int', 'order': 'motorola', 'physical_unit': 'deg.C'}
int Gwtpmstire0x361361::tpms_tire_temperature(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -60.000000;
  return ret;
}

// config detail: {'name': 'tpms_lf_pressure_warning', 'enum': {0: 'TPMS_LF_PRESSURE_WARNING_NO_WARNING', 1: 'TPMS_LF_PRESSURE_WARNING_HIGH_PRESSURE_WARNING', 2: 'TPMS_LF_PRESSURE_WARNING_LOW_PRESSURE_WARNING', 3: 'TPMS_LF_PRESSURE_WARNING_QUIK_LEAKAGE_RESERVED', 4: 'TPMS_LF_PRESSURE_WARNING_LOST_SENSOR', 5: 'TPMS_LF_PRESSURE_WARNING_SENSOR_BATTERY_LOW', 6: 'TPMS_LF_PRESSURE_WARNING_SENSOR_FAULTURE_RESERVED', 7: 'TPMS_LF_PRESSURE_WARNING_LOW_PRESSURE_WARN_QUIK_LEAK_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tpms_tire_0x361_361::Tpms_lf_pressure_warningType Gwtpmstire0x361361::tpms_lf_pressure_warning(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 3);

  Gw_tpms_tire_0x361_361::Tpms_lf_pressure_warningType ret =  static_cast<Gw_tpms_tire_0x361_361::Tpms_lf_pressure_warningType>(x);
  return ret;
}

// config detail: {'name': 'tpms_rf_pressure_warning', 'enum': {0: 'TPMS_RF_PRESSURE_WARNING_NO_WARNING', 1: 'TPMS_RF_PRESSURE_WARNING_HIGH_PRESSURE_WARNING', 2: 'TPMS_RF_PRESSURE_WARNING_LOW_PRESSURE_WARNING', 3: 'TPMS_RF_PRESSURE_WARNING_QUIK_LEAKAGE_RESERVED', 4: 'TPMS_RF_PRESSURE_WARNING_LOST_SENSOR', 5: 'TPMS_RF_PRESSURE_WARNING_SENSOR_BATTERY_LOW', 6: 'TPMS_RF_PRESSURE_WARNING_SENSOR_FAULTURE_RESERVED', 7: 'TPMS_RF_PRESSURE_WARNING_LOW_PRESSURE_WARN_QUIK_LEAK_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 45, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tpms_tire_0x361_361::Tpms_rf_pressure_warningType Gwtpmstire0x361361::tpms_rf_pressure_warning(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(3, 3);

  Gw_tpms_tire_0x361_361::Tpms_rf_pressure_warningType ret =  static_cast<Gw_tpms_tire_0x361_361::Tpms_rf_pressure_warningType>(x);
  return ret;
}

// config detail: {'name': 'tpms_system_status', 'enum': {0: 'TPMS_SYSTEM_STATUS_NO_ANY_ERROR', 1: 'TPMS_SYSTEM_STATUS_SYSTEM_ERROR', 2: 'TPMS_SYSTEM_STATUS_SYETEM_WAIT_FOR_LEARNING', 3: 'TPMS_SYSTEM_STATUS_WINTER_MODE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tpms_tire_0x361_361::Tpms_system_statusType Gwtpmstire0x361361::tpms_system_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 2);

  Gw_tpms_tire_0x361_361::Tpms_system_statusType ret =  static_cast<Gw_tpms_tire_0x361_361::Tpms_system_statusType>(x);
  return ret;
}

// config detail: {'name': 'tpms_rr_pressure_warning', 'enum': {0: 'TPMS_RR_PRESSURE_WARNING_NO_WARNING', 1: 'TPMS_RR_PRESSURE_WARNING_HIGH_PRESSURE_WARNING', 2: 'TPMS_RR_PRESSURE_WARNING_LOW_PRESSURE_WARNING', 3: 'TPMS_RR_PRESSURE_WARNING_QUIK_LEAKAGE_RESERVED', 4: 'TPMS_RR_PRESSURE_WARNING_LOST_SENSOR', 5: 'TPMS_RR_PRESSURE_WARNING_SENSOR_BATTERY_LOW', 6: 'TPMS_RR_PRESSURE_WARNING_SENSOR_FAULTURE_RESERVED', 7: 'TPMS_RR_PRESSURE_WARNING_LOW_PRESSURE_WARN_QUIK_LEAK_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 50, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tpms_tire_0x361_361::Tpms_rr_pressure_warningType Gwtpmstire0x361361::tpms_rr_pressure_warning(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 3);

  Gw_tpms_tire_0x361_361::Tpms_rr_pressure_warningType ret =  static_cast<Gw_tpms_tire_0x361_361::Tpms_rr_pressure_warningType>(x);
  return ret;
}

// config detail: {'name': 'tpms_lr_pressure_warning', 'enum': {0: 'TPMS_LR_PRESSURE_WARNING_NO_WARNING', 1: 'TPMS_LR_PRESSURE_WARNING_HIGH_PRESSURE_WARNING', 2: 'TPMS_LR_PRESSURE_WARNING_LOW_PRESSURE_WARNING', 3: 'TPMS_LR_PRESSURE_WARNING_QUIK_LEAKAGE_RESERVED', 4: 'TPMS_LR_PRESSURE_WARNING_LOST_SENSOR', 5: 'TPMS_LR_PRESSURE_WARNING_SENSOR_BATTERY_LOW', 6: 'TPMS_LR_PRESSURE_WARNING_SENSOR_FAULTURE_RESERVED', 7: 'TPMS_LR_PRESSURE_WARNING_LOW_PRESSURE_WARN_QUIK_LEAK_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 53, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tpms_tire_0x361_361::Tpms_lr_pressure_warningType Gwtpmstire0x361361::tpms_lr_pressure_warning(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(3, 3);

  Gw_tpms_tire_0x361_361::Tpms_lr_pressure_warningType ret =  static_cast<Gw_tpms_tire_0x361_361::Tpms_lr_pressure_warningType>(x);
  return ret;
}

// config detail: {'name': 'tpms_temperature_warning', 'enum': {0: 'TPMS_TEMPERATURE_WARNING_NO_TEMPERATURE_WARNING', 1: 'TPMS_TEMPERATURE_WARNING_TEMPERATURE_WARNING'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 54, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tpms_tire_0x361_361::Tpms_temperature_warningType Gwtpmstire0x361361::tpms_temperature_warning(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  Gw_tpms_tire_0x361_361::Tpms_temperature_warningType ret =  static_cast<Gw_tpms_tire_0x361_361::Tpms_temperature_warningType>(x);
  return ret;
}

// config detail: {'name': 'tpms_resrved', 'enum': {0: 'TPMS_RESRVED_RESERVED', 1: 'TPMS_RESRVED_RESERVED'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tpms_tire_0x361_361::Tpms_resrvedType Gwtpmstire0x361361::tpms_resrved(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  Gw_tpms_tire_0x361_361::Tpms_resrvedType ret =  static_cast<Gw_tpms_tire_0x361_361::Tpms_resrvedType>(x);
  return ret;
}

// config detail: {'name': 'tire_position', 'enum': {0: 'TIRE_POSITION_NO_ANY_SENSOR_ERROR', 1: 'TIRE_POSITION_LEFT_FRONT_TIRE', 2: 'TIRE_POSITION_RIGHT_FRONT_TIRE', 3: 'TIRE_POSITION_RIGHT_REAR_TIRE', 4: 'TIRE_POSITION_LEFT_REAR_TIRE', 5: 'TIRE_POSITION_RESERVED', 6: 'TIRE_POSITION_RESERVED', 7: 'TIRE_POSITION_RESERVED'}, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|7]', 'bit': 58, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tpms_tire_0x361_361::Tire_positionType Gwtpmstire0x361361::tire_position(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 3);

  Gw_tpms_tire_0x361_361::Tire_positionType ret =  static_cast<Gw_tpms_tire_0x361_361::Tire_positionType>(x);
  return ret;
}

// config detail: {'name': 'tpms_lamp_status', 'enum': {0: 'TPMS_LAMP_STATUS_NO_LAMP_LIGHT', 1: 'TPMS_LAMP_STATUS_LAMP_FLASH', 2: 'TPMS_LAMP_STATUS_LAMP_LIGHT', 3: 'TPMS_LAMP_STATUS_RESEVED'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 60, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tpms_tire_0x361_361::Tpms_lamp_statusType Gwtpmstire0x361361::tpms_lamp_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 2);

  Gw_tpms_tire_0x361_361::Tpms_lamp_statusType ret =  static_cast<Gw_tpms_tire_0x361_361::Tpms_lamp_statusType>(x);
  return ret;
}

// config detail: {'name': 'tpms_al_state', 'enum': {0: 'TPMS_AL_STATE_DEFAULTS', 1: 'TPMS_AL_STATE_SELF_LEARNING', 2: 'TPMS_AL_STATE_SELF_LEARNING_OVER', 3: 'TPMS_AL_STATE_RESERVE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 62, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
Gw_tpms_tire_0x361_361::Tpms_al_stateType Gwtpmstire0x361361::tpms_al_state(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(5, 2);

  Gw_tpms_tire_0x361_361::Tpms_al_stateType ret =  static_cast<Gw_tpms_tire_0x361_361::Tpms_al_stateType>(x);
  return ret;
}

// config detail: {'name': 'tpms_lf_pressure', 'offset': 0.0, 'precision': 2.75, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|698.5]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'Kpa'}
double Gwtpmstire0x361361::tpms_lf_pressure(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.750000;
  return ret;
}
}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto
