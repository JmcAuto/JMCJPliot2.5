/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_PRESSURE_0X241_241_H_
#define MODULES_CANBUS_VEHICLE_cx75_PROTOCOL_ESP_PRESSURE_0X241_241_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace jmc_auto {
namespace canbus {
namespace cx75 {

class Esppressure0x241241 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    ::jmc_auto::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Esppressure0x241241();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'ESP_TrfCaseTqLmtdByEsp', 'offset': 0.0, 'precision': 1.0, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|4095]', 'bit': 11, 'type': 'int', 'order': 'motorola', 'physical_unit': 'Nm'}
  int esp_trfcasetqlmtdbyesp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_TrfCaseModReqdByEsp', 'enum': {0: 'ESP_TRFCASEMODREQDBYESP_NO_REQUEST', 1: 'ESP_TRFCASEMODREQDBYESP_FAST_OPEN_REQUEST', 2: 'ESP_TRFCASEMODREQDBYESP_TORQUE_UPPER_LIMIT_REQUEST', 3: 'ESP_TRFCASEMODREQDBYESP_FAILURE'}, 'precision': 1.0, 'len': 2, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|3]', 'bit': 41, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_pressure_0x241_241::Esp_trfcasemodreqdbyespType esp_trfcasemodreqdbyesp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_PresOffsetMasterCylinderVali', 'enum': {0: 'ESP_PRESOFFSETMASTERCYLINDERVALI_VALID', 1: 'ESP_PRESOFFSETMASTERCYLINDERVALI_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 42, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_pressure_0x241_241::Esp_presoffsetmastercylindervaliType esp_presoffsetmastercylindervali(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_PresOffsetMasterCylinder', 'enum': {31: 'ESP_PRESOFFSETMASTERCYLINDER_INVALID'}, 'precision': 1.0, 'len': 5, 'is_signed_var': False, 'offset': -15.0, 'physical_range': '[-15|15]', 'bit': 47, 'type': 'enum', 'order': 'motorola', 'physical_unit': 'bar'}
  int esp_presoffsetmastercylinder(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Rolling_counter_0x241', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 51, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int rolling_counter_0x241(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_Master_Cylinder_Pressure_Sta', 'enum': {0: 'ESP_MASTER_CYLINDER_PRESSURE_STA_VALID', 1: 'ESP_MASTER_CYLINDER_PRESSURE_STA_INVALID'}, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 55, 'type': 'enum', 'order': 'motorola', 'physical_unit': ''}
  Esp_pressure_0x241_241::Esp_master_cylinder_pressure_staType esp_master_cylinder_pressure_sta(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'Checksum_0x241', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  int checksum_0x241(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'ESP_Master_Cylinder_Pressure', 'offset': 0.0, 'precision': 0.0625, 'len': 12, 'is_signed_var': False, 'physical_range': '[0|255.9375]', 'bit': 7, 'type': 'double', 'order': 'motorola', 'physical_unit': 'bar'}
  double esp_master_cylinder_pressure(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace cx75
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_cx75_PROTOCOL_ESP_PRESSURE_0X241_241_H_
