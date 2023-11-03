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

#ifndef MODULES_CANBUS_VEHICLE_M2_PROTOCOL_ODOMETER_INPUT_99_H_
#define MODULES_CANBUS_VEHICLE_M2_PROTOCOL_ODOMETER_INPUT_99_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/gnss_m2/proto/m2.pb.h"

namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

class Odometerinput99 : public ::jmc_auto::drivers::canbus::ProtocolData<
                    jmc_auto::drivers::gnss_m2::M2> {
 public:
  static const int32_t ID;

  Odometerinput99();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'description': 'checksum=Sum(Byte0:Byte6)^0xff', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name': 'CheckSum', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Odometerinput99* set_checksum(int checksum);

  // config detail: {'name': 'VehSpd_input', 'offset': 0.0, 'precision': 0.00390625, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|256]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
  Odometerinput99* set_vehspd_input(double vehspd_input);

  // config detail: {'name': 'GearPosition_input', 'offset': 0.0, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'physical_range': '[0|2]', 'bit': 2, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  Odometerinput99* set_gearposition_input(int gearposition_input);

 private:

  // config detail: {'description': 'checksum=Sum(Byte0:Byte6)^0xff', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name': 'CheckSum', 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 63, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_checksum(uint8_t* data, int checksum);

  // config detail: {'name': 'VehSpd_input', 'offset': 0.0, 'precision': 0.00390625, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|256]', 'bit': 23, 'type': 'double', 'order': 'motorola', 'physical_unit': 'km/h'}
  void set_p_vehspd_input(uint8_t* data, double vehspd_input);

  // config detail: {'name': 'GearPosition_input', 'offset': 0.0, 'precision': 1.0, 'len': 3, 'is_signed_var': False, 'physical_range': '[0|2]', 'bit': 2, 'type': 'int', 'order': 'motorola', 'physical_unit': ''}
  void set_p_gearposition_input(uint8_t* data, int gearposition_input);

 private:
  int checksum_;
  double vehspd_input_;
  int gearposition_input_;
};

}  // namespace m2
}  // namespace canbus
}  // namespace jmc_auto

#endif  // MODULES_CANBUS_VEHICL_M2_PROTOCOL_ODOMETER_INPUT_99_H_
